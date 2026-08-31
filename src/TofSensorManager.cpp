#include "TofSensorManager.h"
#include "DisplayService.h"
#include <Arduino.h>
#include <Wire.h>
#include <cstring>

namespace {
constexpr unsigned long kReinitBackoffMs = 2000;
constexpr uint16_t kIndoorMaxReportedDistanceMm = 1000;

bool deadlineReached(unsigned long now, unsigned long deadline)
{
  return static_cast<long>(now - deadline) >= 0;
}
}

TofSensorManager::TofSensorManager(DisplayService *displayService)
    : display(displayService) {}

void TofSensorManager::setStatusPublisher(StatusPublisher publisher)
{
  statusPublisher = std::move(publisher);
}

bool TofSensorManager::begin(bool allowRetry)
{
  if (initialiseSensors(false))
  {
    return true;
  }
  if (!allowRetry)
  {
    return false;
  }

  sensorInitNeeded = true;
  nextReinitAttemptMs = millis() + kReinitBackoffMs;
  if (display)
  {
    display->showStatus("VL53L0X retry pending", true);
  }
  return false;
}

TofSensorManager::UpdateResult TofSensorManager::update()
{
  UpdateResult result;
  unsigned long now = millis();

  if (!sensorInitNeeded && (now - lastSensorInitMs >= Config.timing.sensorReinitIntervalMs))
  {
    sensorInitNeeded = true;
    nextReinitAttemptMs = now;
  }

  const size_t i = nextSensorToPoll;
  const bool withinGrace = !deadlineReached(now, sensorStatusSuppressUntil[i]);
  auto &sensor = sensors[i];
  measurementOk[i] = false;

  if (!sensorReady[i])
  {
    if (!withinGrace)
    {
      publishSensorStatus(static_cast<uint8_t>(i), false, false);
    }
  }
  else
  {
    const uint16_t reading = sensor.readRangeContinuousMillimeters();
    const bool timedOut = sensor.timeoutOccurred();
    const bool invalid = timedOut || sensor.last_status != 0 ||
                         reading == 0 || reading == Config.tof.errorValue;
    ranges[i] = reading;

    if (withinGrace)
    {
      resetMeasurementState(i);
      measurementOk[i] = !invalid;
      if (!invalid)
      {
        publishSensorStatus(static_cast<uint8_t>(i), true, false);
      }
    }
    else if (invalid)
    {
      publishSensorStatus(static_cast<uint8_t>(i), false, false);
      resetMeasurementState(i);
      if (reading == Config.tof.errorValue)
      {
        sensorInitNeeded = true;
        sensorReady[i] = false;
        sensorStatusSuppressUntil[i] = millis() + Config.timing.sensorStatusGracePeriodMs;
      }
    }
    else
    {
      publishSensorStatus(static_cast<uint8_t>(i), true, false);
      measurementOk[i] = true;
      const bool below = reading <= Config.tof.maxConsideredDistanceMm &&
                         reading < Config.tof.rangeThresholdMm[i];
      if (!below)
      {
        resetMeasurementState(i);
      }
      else
      {
        uint8_t &streak = sensorBelowStreak[i];
        if (streak < Config.tof.consecutiveReadingsForTrigger)
        {
          ++streak;
        }
        if (streak >= Config.tof.consecutiveReadingsForTrigger && !sensorBelowThreshold[i])
        {
          sensorBelowThreshold[i] = true;
          result.triggerEvent = true;
          result.triggerSensorId = static_cast<uint8_t>(i + 1);
        }
      }
    }
  }

  nextSensorToPoll = (nextSensorToPoll + 1) % kSensorCount;
  result.anySensorQualified = false;
  for (bool qualified : sensorBelowThreshold)
  {
    result.anySensorQualified = result.anySensorQualified || qualified;
  }

  now = millis();
  if (sensorInitNeeded && deadlineReached(now, nextReinitAttemptMs))
  {
    if (initialiseSensors(true))
    {
      sensorInitNeeded = false;
    }
    else
    {
      nextReinitAttemptMs = millis() + kReinitBackoffMs;
    }
  }

  if (Config.tof.enableDebugDisplay && display)
  {
    auto formatDistance = [](uint16_t mm, char *out, size_t len) {
      if (len == 0)
      {
        return;
      }
      if (mm == 0 || mm == Config.tof.errorValue)
      {
        std::strncpy(out, "--", len - 1);
        out[len - 1] = '\0';
        return;
      }
      float cm = mm / 10.0f;
      char buf[16];
      dtostrf(cm, 0, 1, buf);
      const char *trimmed = buf;
      while (*trimmed == ' ')
      {
        ++trimmed;
      }
      std::strncpy(out, trimmed, len - 1);
      out[len - 1] = '\0';
    };

    char indoor[12];
    char outdoor[12];
    formatDistance(reportedDistanceMm(0), indoor, sizeof(indoor));
    formatDistance(reportedDistanceMm(1), outdoor, sizeof(outdoor));

    char message[64];
    snprintf(message, sizeof(message), "IN:%scm OUT:%scm", indoor, outdoor);
    display->showStatus(message);
  }

  return result;
}

uint16_t TofSensorManager::reportedDistanceMm(size_t index) const
{
  if (index >= kSensorCount)
  {
    return 0;
  }

  uint16_t mm = ranges[index];
  if (mm == 0 || mm == Config.tof.errorValue)
  {
    return mm;
  }

  if (index == 0 && mm > kIndoorMaxReportedDistanceMm)
  {
    return kIndoorMaxReportedDistanceMm;
  }

  return mm;
}

float TofSensorManager::distanceCm(size_t index) const
{
  if (index >= kSensorCount)
  {
    return -1.0f;
  }
  uint16_t mm = reportedDistanceMm(index);
  if (mm == 0 || mm == Config.tof.errorValue)
  {
    return -1.0f;
  }
  return mm / 10.0f;
}

bool TofSensorManager::initialiseSensors(bool isReinit)
{
  Wire.begin(Config.tof.sdaPin, Config.tof.sclPin, 400000);

  for (size_t i = 0; i < kSensorCount; ++i)
  {
    if (isReinit)
    {
      sensors[i].stopContinuous();
    }
    pinMode(Config.tof.xshutPins[i], OUTPUT);
    digitalWrite(Config.tof.xshutPins[i], LOW);
    sensorReady[i] = false;
    resetMeasurementState(i);
    ranges[i] = 0;
    sensorStatusSuppressUntil[i] = millis() + Config.timing.sensorStatusGracePeriodMs;
    measurementOk[i] = false;
  }

  if (isReinit)
  {
    for (size_t i = 0; i < kSensorCount; ++i)
    {
      sensors[i] = VL53L0X();
      sensors[i].setBus(&Wire);
    }
  }

  delay(Config.timing.sensorInitDelayMs);

  bool success = true;

  for (size_t i = 0; i < kSensorCount; ++i)
  {
    pinMode(Config.tof.xshutPins[i], INPUT);
    delay(Config.timing.sensorInitDelayMs);

    auto &sensor = sensors[i];
    sensor.setBus(&Wire);
    sensor.setTimeout(Config.tof.timeoutMs);
    if (!sensor.init())
    {
      publishSensorStatus(static_cast<uint8_t>(i), false, false);
      success = false;
      if (display && !isReinit)
      {
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "ERROR:%s sensor failed!", Config.tof.names[i]);
        display->showStatus(errorMsg, true);
      }
      continue;
    }

    sensor.setAddress(Config.tof.startAddress + i);
    configureSensor(sensor, i);
    sensor.startContinuous();

    sensorReady[i] = true;
    resetMeasurementState(i);
    ranges[i] = 0;
    sensorStatusSuppressUntil[i] = millis() + Config.timing.sensorStatusGracePeriodMs;
    measurementOk[i] = false;

    publishSensorStatus(static_cast<uint8_t>(i), true, true);

    if (display && !isReinit)
    {
      char statusMsg[64];
      snprintf(statusMsg, sizeof(statusMsg), "%s sensor at 0x%X", Config.tof.names[i], sensor.getAddress());
      display->showStatus(statusMsg);
    }
  }

  if (success)
  {
    lastSensorInitMs = millis();
  }

  return success;
}

void TofSensorManager::configureSensor(VL53L0X &sensor, size_t index)
{
  sensor.setMeasurementTimingBudget(Config.tof.timingBudgetUs);

  if (index == static_cast<size_t>(TOFSensor::IndoorSensor))
  {
    sensor.setSignalRateLimit(0.1f);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  }
}

void TofSensorManager::publishSensorStatus(uint8_t sensorIndex, bool ok, bool reportInitEvent)
{
  if (statusPublisher)
  {
    statusPublisher(sensorIndex, ok, reportInitEvent);
  }
}

void TofSensorManager::resetMeasurementState(size_t index)
{
  sensorBelowThreshold[index] = false;
  sensorBelowStreak[index] = 0;
}
