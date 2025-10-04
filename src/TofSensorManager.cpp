#include "TofSensorManager.h"
#include "DisplayService.h"
#include <Arduino.h>
#include <Wire.h>
#include <cstring>

namespace {
constexpr unsigned long kReinitBackoffMs = 2000;
}

TofSensorManager::TofSensorManager(DisplayService *displayService)
    : display(displayService) {}

void TofSensorManager::setDisplayService(DisplayService *service)
{
  display = service;
}

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

  unsigned attempt = 0;
  while (!initialiseSensors(false))
  {
    ++attempt;
    if (display)
    {
      display->showStatus("Retrying VL53L0X init", true);
    }
    delay(Config.timing.setupDelayMs);
  }
  (void)attempt;
  return true;
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

  for (size_t i = 0; i < kSensorCount; ++i)
  {
    bool withinGrace = now < sensorStatusSuppressUntil[i];
    auto &sensor = sensors[i];
    measurementOk[i] = false;

    if (!sensorReady[i])
    {
      if (!withinGrace)
      {
        publishSensorStatus(static_cast<uint8_t>(i), false, false);
      }
      continue;
    }

    uint16_t reading = sensor.readRangeContinuousMillimeters();
    ranges[i] = reading;
    result.rangesMm[i] = reading;

    if (withinGrace)
    {
      (void)sensor.timeoutOccurred();
      sensorBelowThreshold[i] = false;
      sensorBelowStreak[i] = 0;
      measurementOk[i] = true;
      publishSensorStatus(static_cast<uint8_t>(i), true, false);
      continue;
    }

    if (sensor.timeoutOccurred())
    {
      publishSensorStatus(static_cast<uint8_t>(i), false, false);
      sensorBelowThreshold[i] = false;
      sensorBelowStreak[i] = 0;
      continue;
    }

    if (sensor.last_status != 0)
    {
      publishSensorStatus(static_cast<uint8_t>(i), false, false);
      continue;
    }

    if (reading == 0)
    {
      publishSensorStatus(static_cast<uint8_t>(i), false, false);
      continue;
    }

    if (reading == Config.tof.errorValue)
    {
      publishSensorStatus(static_cast<uint8_t>(i), false, false);
      sensorInitNeeded = true;
      sensorReady[i] = false;
      sensorBelowThreshold[i] = false;
      sensorBelowStreak[i] = 0;
      sensorStatusSuppressUntil[i] = millis() + Config.timing.sensorStatusGracePeriodMs;
      continue;
    }

    publishSensorStatus(static_cast<uint8_t>(i), true, false);
    measurementOk[i] = true;

    if (reading > Config.tof.maxConsideredDistanceMm)
    {
      sensorBelowThreshold[i] = false;
      sensorBelowStreak[i] = 0;
      continue;
    }

    bool below = reading < Config.tof.rangeThresholdMm[i];
    if (below)
    {
      uint8_t &streak = sensorBelowStreak[i];
      if (streak < Config.tof.consecutiveReadingsForTrigger)
      {
        ++streak;
      }

      if (streak >= Config.tof.consecutiveReadingsForTrigger)
      {
        result.anySensorQualified = true;
        if (!sensorBelowThreshold[i])
        {
          result.triggerEvent = true;
          result.triggerSensorId = (i == 0) ? 1 : 2;
          sensorBelowThreshold[i] = true;
        }
      }
      else
      {
        sensorBelowThreshold[i] = false;
      }
    }
    else
    {
      sensorBelowThreshold[i] = false;
      sensorBelowStreak[i] = 0;
    }
  }

  if (result.triggerEvent)
  {
    lastSensorTriggered = result.triggerSensorId;
    if (display)
    {
      display->setLastSensorTriggered(lastSensorTriggered);
    }
  }

  if (sensorInitNeeded && millis() >= nextReinitAttemptMs)
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
    formatDistance(ranges[0], indoor, sizeof(indoor));
    formatDistance(ranges[1], outdoor, sizeof(outdoor));

    char message[64];
    snprintf(message, sizeof(message), "IN:%scm OUT:%scm", indoor, outdoor);
    display->showStatus(message);
  }

  return result;
}

float TofSensorManager::distanceCm(size_t index) const
{
  if (index >= kSensorCount)
  {
    return -1.0f;
  }
  uint16_t mm = ranges[index];
  if (mm == 0 || mm == Config.tof.errorValue)
  {
    return -1.0f;
  }
  return mm / 10.0f;
}

void TofSensorManager::requestReinitialisation()
{
  sensorInitNeeded = true;
  nextReinitAttemptMs = millis();
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
    sensorBelowThreshold[i] = false;
    sensorBelowStreak[i] = 0;
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
    sensor.setMeasurementTimingBudget(Config.tof.timingBudgetUs);
    sensor.startContinuous();

    sensorReady[i] = true;
    sensorBelowThreshold[i] = false;
    sensorBelowStreak[i] = 0;
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

void TofSensorManager::publishSensorStatus(uint8_t sensorIndex, bool ok, bool reportInitEvent)
{
  if (statusPublisher)
  {
    statusPublisher(sensorIndex, ok, reportInitEvent);
  }
}
