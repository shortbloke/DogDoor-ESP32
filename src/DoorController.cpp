#include "DoorController.h"
#include "mqtt.h"
#include "DisplayHelpers.h"
#include <Arduino.h>
#include <UMS3.h>
#include <WiFi.h>
#include <cstring>

// Serial debug print macro. Configured via platformio.ini
#ifdef SERIAL_PRINT_ENABLE
  #define SERIAL_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define SERIAL_PRINT(...)
#endif

// Implementation for setting WiFi status
void DoorController::setWiFiConnected(bool connected)
{
  wifiConnected = connected;
  DisplayHelpers::setWiFiConnected(connected);
}

// -----------------------------------------------------------------------------
// Constructor & Destructor
// -----------------------------------------------------------------------------
DoorController::DoorController() {}

DoorController::~DoorController() = default;

// -----------------------------------------------------------------------------
// Public Interface
// -----------------------------------------------------------------------------
void DoorController::setup()
{
  Serial.begin(Config::SerialBaudRate);
  SERIAL_PRINT("Setup Start\n");

  setupStepper();
  DisplayHelpers::showStatus(display, "Stepper ready");
  SERIAL_PRINT("Stepper initialized\n");

  setPixelColor(0, 0, 255);

  while (!setupTOFSensors())
  {
    SERIAL_PRINT("Retrying VL53L0X sensor initialization...\n");
    DisplayHelpers::showStatus(display, "Retrying VL53L0X init");
    setPixelColor(0, 0, 0);
    delay(Config::SetupDelayMs / 2);
    setPixelColor(0, 0, 255);
    delay(Config::SetupDelayMs / 2);
  }
  SERIAL_PRINT("VL53L0X Sensors initialized successfully\n");
  updateSensorStates();
  SERIAL_PRINT("Indoor sensor range: %d mm\n", range[0]);
  SERIAL_PRINT("Outdoor sensor range: %d mm\n", range[1]);
  DisplayHelpers::showStatus(display, "VL53L0X Sensors ready");

  setPixelColor(255, 0, 255); // Purple

  setupLimitSwitches();
  SERIAL_PRINT("Limit switches initialized\n");
  DisplayHelpers::showStatus(display, "Limit switches ready");

  // Setup override toggle switch
  pinMode(Config::overrideKeepOpenSwitchPin, INPUT_PULLDOWN);
  pinMode(Config::overrideKeepClosedSwitchPin, INPUT_PULLDOWN);

  SERIAL_PRINT("Setup END\n");

  refreshStateDisplay();
}

void DoorController::loop()
{
  for (auto &debouncer : limitSwitchDebouncers)
  {
    debouncer.update();
  }

  for (size_t i = 0; i < Config::numLimitSwitches; ++i)
  {
    bool pressed = isLimitSwitchPressed(static_cast<LimitSwitch>(i));
    int8_t pressedVal = pressed ? 1 : 0;
    if (lastLimitSwitchState[i] != pressedVal)
    {
      lastLimitSwitchState[i] = pressedVal;
      if (i == BottomLimitSwitch)
      {
        mqttPublishLimitSwitchBottom(!pressed); // invert for MQTT wiring polarity
      }
      else if (i == TopLimitSwitch)
      {
        mqttPublishLimitSwitchTop(!pressed); // invert for MQTT wiring polarity
      }
    }
  }
  // Periodically check WiFi status for icon (every wifiCheckIntervalMs)
  unsigned long now = millis();
  if (now - lastWiFiCheckMs >= wifiCheckIntervalMs)
  {
    wifiConnected = (WiFi.status() == WL_CONNECTED);
    DisplayHelpers::setWiFiConnected(wifiConnected);
    lastWiFiCheckMs = now;
  }
  if (!sensorInitNeeded && (now - lastSensorInitMs >= sensorReinitIntervalMs))
  {
    SERIAL_PRINT("Scheduling VL53L0X sensor re-initialization...\n");
    sensorInitNeeded = true;
  }

  checkOverrideSwitches();
  updateSensorStates();
  handleState();

  if (sensorInitNeeded)
  {
    if (reinitTOFSensors())
    {
      sensorInitNeeded = false;
    }
  }
}

void DoorController::setUMS3(UMS3 *ums3Ptr)
{
  ums3 = ums3Ptr;
}

void DoorController::setDisplay(Adafruit_SSD1306 *displayPtr)
{
  display = displayPtr;
}

void DoorController::setPixelColor(uint8_t r, uint8_t g, uint8_t b)
{
  if (ums3)
  {
    ums3->setPixelColor(r, g, b);
  }
}

void DoorController::refreshStateDisplay()
{
  showStateOnDisplay();
}

const char* DoorController::getStateString() const {
    if (keepOpen)   return "KEEP OPEN";
    if (keepClosed) return "LOCKED";
    switch (state) {
      case DoorState::Open:     return "OPEN";
      case DoorState::Closed:   return "CLOSED";
      case DoorState::Opening:  return "OPENING";
      case DoorState::Closing:  return "CLOSING";
      default:                  return "UNKNOWN";
    }
}

float DoorController::getDistanceIndoorCm() const {
    constexpr size_t indoorIndex = 0;
    uint16_t mm = range[indoorIndex];
    if (mm == 0 || mm == Config::TOFSensorErrorValue) return -1.0f;
    return mm / 10.0f;
}

float DoorController::getDistanceOutdoorCm() const {
    constexpr size_t outdoorIndex = 1;
    uint16_t mm = range[outdoorIndex];
    if (mm == 0 || mm == Config::TOFSensorErrorValue) return -1.0f;
    return mm / 10.0f;
}

// -----------------------------------------------------------------------------
// Hardware Setup
// -----------------------------------------------------------------------------
void DoorController::setupStepper()
{
  engine.init();
  stepper = engine.stepperConnectToPin(Config::stepPinStepper);
  if (stepper)
  {
    stepper->setDirectionPin(Config::dirPinStepper);
    stepper->setEnablePin(Config::enablePinStepper);
    stepper->setAutoEnable(false);
    stepper->setSpeedInHz(Config::stepsPerSecond);
    stepper->setAcceleration(Config::acceleration);
    stepper->setCurrentPosition(0);
  }
  else
  {
    SERIAL_PRINT("Stepper initialization failed!\n");
    DisplayHelpers::showStatus(display, "ERROR: Stepper init failed!");
  }
}

void DoorController::publishSensorStatus(uint8_t sensorIndex, bool ok, bool reportInitEvent)
{
  mqttPublishTOFSensorStatus(sensorIndex, ok);
  if (ok && reportInitEvent)
  {
    mqttPublishTOFInit(sensorIndex);
  }
}

bool DoorController::setupTOFSensors()
{
  return initializeTOFSensors(false);
}

bool DoorController::reinitTOFSensors()
{
  SERIAL_PRINT("Re-initializing VL53L0X sensors...\n");
  return initializeTOFSensors(true);
}

bool DoorController::initializeTOFSensors(bool isReinit)
{
  Wire.begin(Config::SDA, Config::SCL, 400000);

  for (size_t i = 0; i < Config::numTOFSensors; ++i)
  {
    if (isReinit)
    {
      sensors[i].stopContinuous();
    }
    pinMode(Config::xshutPins[i], OUTPUT);
    digitalWrite(Config::xshutPins[i], LOW);
    sensorReady[i] = false;
    sensorBelowThreshold[i] = false;
    sensorBelowStreak[i] = 0;
    range[i] = 0;
    sensorStatusSuppressUntil[i] = millis() + sensorStatusGracePeriodMs;
  }

  if (isReinit)
  {
    for (size_t i = 0; i < Config::numTOFSensors; ++i)
    {
      sensors[i] = VL53L0X(); // reset stored I2C address back to the default 0x29
      sensors[i].setBus(&Wire);
    }
  }

  delay(Config::SensorInitDelayMs);

  bool success = true;

  for (size_t i = 0; i < Config::numTOFSensors; ++i)
  {
    pinMode(Config::xshutPins[i], INPUT);
    delay(Config::SensorInitDelayMs);

    auto &sensor = sensors[i];
    sensor.setBus(&Wire);
    sensor.setTimeout(Config::TOFSensorTimeout);
    if (!sensor.init())
    {
      SERIAL_PRINT("%s sensor %sinit failed!\n", Config::sensorNames[i], isReinit ? "re-" : "");
      if (!isReinit)
      {
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "ERROR:%s sensor failed!", Config::sensorNames[i]);
        DisplayHelpers::showStatus(display, errorMsg, true, 1);
        mqttPublishTOFSensorStatus(static_cast<uint8_t>(i), false);
        return false;
      }
      sensorReady[i] = false;
      sensorBelowStreak[i] = 0;
      success = false;
      publishSensorStatus(static_cast<uint8_t>(i), false);
      sensorStatusSuppressUntil[i] = 0;
      continue;
    }

    sensor.setAddress(Config::TOFSensorStartAddress + i);
    sensor.setMeasurementTimingBudget(Config::TOFSensorTimeMeasurementBudget);
    sensor.startContinuous();

    sensorReady[i] = true;
    sensorBelowThreshold[i] = false;
    sensorBelowStreak[i] = 0;
    range[i] = 0;
    sensorStatusSuppressUntil[i] = millis() + sensorStatusGracePeriodMs;
    SERIAL_PRINT("%s sensor %sinitialized at address: 0x%X\n", Config::sensorNames[i], isReinit ? "re-" : "", sensor.getAddress());

    if (!isReinit)
    {
      char statusMsg[64];
      snprintf(statusMsg, sizeof(statusMsg), "%s sensor at 0x%X", Config::sensorNames[i], sensor.getAddress());
      DisplayHelpers::showStatus(display, statusMsg);
    }

    publishSensorStatus(static_cast<uint8_t>(i), true, true);
  }

  if (success)
  {
    SERIAL_PRINT("VL53L0X sensors %sinitialized\n", isReinit ? "re-" : "");
    lastSensorInitMs = millis();
  }

  return success;
}
void DoorController::setupLimitSwitches()
{
  pinMode(Config::limitSwitchPins[BottomLimitSwitch], INPUT_PULLUP);
  limitSwitchDebouncers[BottomLimitSwitch].attach(Config::limitSwitchPins[BottomLimitSwitch]);
  limitSwitchDebouncers[BottomLimitSwitch].interval(Config::LimitSwitchDebounceMs);

  pinMode(Config::limitSwitchPins[TopLimitSwitch], INPUT_PULLUP);
  limitSwitchDebouncers[TopLimitSwitch].attach(Config::limitSwitchPins[TopLimitSwitch]);
  limitSwitchDebouncers[TopLimitSwitch].interval(Config::LimitSwitchDebounceMs);
}

// -----------------------------------------------------------------------------
// Sensor Helpers
// -----------------------------------------------------------------------------
void DoorController::updateSensorStates()
{
  bool anySensorQualified = false;
  bool triggerEvent = false;
  uint8_t triggerSensorId = 0;
  unsigned long now = millis();
  for (size_t i = 0; i < Config::numTOFSensors; ++i)
  {
    bool withinGrace = now < sensorStatusSuppressUntil[i];
    if (!sensorReady[i])
    {
      if (!withinGrace)
      {
        publishSensorStatus(static_cast<uint8_t>(i), false);
      }
      continue;
    }

    auto &sensor = sensors[i];
    range[i] = sensor.readRangeContinuousMillimeters();
    if (withinGrace)
    {
      (void)sensor.timeoutOccurred();
      sensorBelowThreshold[i] = false;
      sensorBelowStreak[i] = 0;
      continue;
    }
    if (sensor.timeoutOccurred())
    {
      SERIAL_PRINT("%s sensor timeout, skipping...\n", Config::sensorNames[i]);
      publishSensorStatus(static_cast<uint8_t>(i), false);
      continue;
    }
    else if (sensor.last_status != 0)
    {
      SERIAL_PRINT("%s sensor error %d, skipping...\n", Config::sensorNames[i], sensor.last_status);
      publishSensorStatus(static_cast<uint8_t>(i), false);
      continue;
    }
    else if (range[i] == 0)
    {
      SERIAL_PRINT("%s sensor failed to read, skipping...\n", Config::sensorNames[i]);
      publishSensorStatus(static_cast<uint8_t>(i), false);
      continue;
    }
    else if (range[i] == Config::TOFSensorErrorValue)
    {
      SERIAL_PRINT("%s sensor not found, scheduling re-initialization...\n", Config::sensorNames[i]);
      sensorInitNeeded = true;
      sensorReady[i] = false;
      sensorBelowStreak[i] = 0;
      sensorStatusSuppressUntil[i] = millis() + sensorStatusGracePeriodMs;
      publishSensorStatus(static_cast<uint8_t>(i), false);
      continue;
    }
    publishSensorStatus(static_cast<uint8_t>(i), true);
    if (range[i] > Config::TOFSensorMaxConsideredDistanceMm)
    {
      sensorBelowThreshold[i] = false;
      sensorBelowStreak[i] = 0;
      continue;
    }

    bool below = range[i] < Config::rangeThreshold[i];
    if (below)
    {
      uint8_t &streak = sensorBelowStreak[i];
      if (streak < Config::TOFSensorTriggerConsecutiveReadings)
      {
        ++streak;
      }

      if (streak >= Config::TOFSensorTriggerConsecutiveReadings)
      {
        anySensorQualified = true;
        if (!sensorBelowThreshold[i])
        {
          triggerEvent = true;
          triggerSensorId = (i == 0) ? 1 : 2; // 1 = indoor, 2 = outdoor
          float cm = range[i] / 10.0f;
          if (i == 0)
          {
            mqttPublishDistanceIndoor(cm);
          }
          else
          {
            mqttPublishDistanceOutdoor(cm);
          }
          mqttPublishSensorTrigger(triggerSensorId);
          SERIAL_PRINT("%s sensor detected an object at %d mm\n", Config::sensorNames[i], range[i]);
        }
        sensorBelowThreshold[i] = true;
      }
      else
      {
        sensorBelowThreshold[i] = false;
      }
    }
    else
    {
      sensorBelowStreak[i] = 0;
      sensorBelowThreshold[i] = false;
    }
  }
  if (triggerEvent)
  {
    lastSensorTriggered = triggerSensorId;
    DisplayHelpers::setLastSensorTriggered(lastSensorTriggered);
  }
  if (keepClosed)
  {
    openDoor = false;
  }
  else if (!keepOpen)
  {
    openDoor = anySensorQualified;
  }

  if (Config::enableSensorDebugDisplay && display)
  {
    auto formatDistance = [](uint16_t mm, char *out, size_t len) {
      if (mm == 0 || mm == Config::TOFSensorErrorValue)
      {
        if (len > 0)
        {
          strncpy(out, "--", len - 1);
          out[len - 1] = '\0';
        }
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
      if (len > 0)
      {
        strncpy(out, trimmed, len - 1);
        out[len - 1] = '\0';
      }
    };

    char indoor[12];
    char outdoor[12];
    formatDistance(range[static_cast<size_t>(IndoorSensor)], indoor, sizeof(indoor));
    formatDistance(range[static_cast<size_t>(OutdoorSensor)], outdoor, sizeof(outdoor));

    char message[64];
    snprintf(message, sizeof(message), "IN:%scm OUT:%scm", indoor, outdoor);
    DisplayHelpers::showStatus(display, message, false, 1);
  }
}

// -----------------------------------------------------------------------------
// State Machine and Handlers
// -----------------------------------------------------------------------------
void DoorController::checkOverrideSwitches()
{
  const bool keepOpenActive = digitalRead(Config::overrideKeepOpenSwitchPin) == HIGH;
  if (keepOpenActive && !keepOpen)
  {
    openDoor = true;
    keepOpen = true;
    SERIAL_PRINT("Override keep open switch activated. Door will remain open.\n");
  }
  else if (!keepOpenActive && keepOpen)
  {
    keepOpen = false;
    openDoor = false;
    if (stepper)
    {
      state = DoorState::Closing;
      fastClosing = true;
      displayedSeekBottomHint = false;
      stepper->moveTo(Config::expectedDoorClosePosition, false);
    }
    else
    {
      state = DoorState::Closing;
    }
    SERIAL_PRINT("Override keep open switch deactivated. Resuming normal operation.\n");
  }

  const bool keepClosedActive = digitalRead(Config::overrideKeepClosedSwitchPin) == HIGH;
  if (keepClosedActive && !keepClosed)
  {
    openDoor = false;
    keepClosed = true;
    if (state == DoorState::Open || state == DoorState::Opening)
    {
      if (stepper)
      {
        state = DoorState::Closing;
        fastClosing = true;
        displayedSeekBottomHint = false;
        stepper->moveTo(Config::expectedDoorClosePosition, false);
      }
      else
      {
        state = DoorState::Closing;
      }
    }
    showStateOnDisplay();
    SERIAL_PRINT("Override keep closed switch activated. Door will remain closed.\n");
  }
  else if (!keepClosedActive && keepClosed)
  {
    keepClosed = false;
    handleState();
    showStateOnDisplay();
    SERIAL_PRINT("Override keep closed switch deactivated. Resuming normal operation.\n");
  }
}

void DoorController::handleState()
{
    switch (state)
    {
    case DoorState::Closed:
        handleClosedState();
        break;
    case DoorState::Opening:
        handleOpeningState();
        break;
    case DoorState::Open:
        handleOpenState();
        break;
    case DoorState::Closing:
        handleClosingState();
        break;
    }

    if (previousState != state)
    {
        showStateOnDisplay();
    }
    previousState = state;
}

// --- State Handlers ---
void DoorController::handleClosedState()
{
  if (openDoor && stepper)
  {
    if (isLimitSwitchPressed(TopLimitSwitch)) {
      return;
    }
    SERIAL_PRINT("Opening door\n");
    SERIAL_PRINT("Moving to expectedDoorOpenPosition %u\n", expectedDoorOpenPosition);
    SERIAL_PRINT("Current acceleration: %u\n", stepper->getAcceleration());
    state = DoorState::Opening;
    displayedSeekTopHint = false;
    stepper->enableOutputs();
    stepper->moveTo(expectedDoorOpenPosition, false);
  }
}

void DoorController::handleOpeningState()
{
  if (!stepper)
  {
    return;
  }

  if (isLimitSwitchPressedRaw(TopLimitSwitch))
  {
    int32_t currentPos = stepper->getCurrentPosition();
    expectedDoorOpenPosition = currentPos;
    stepper->forceStopAndNewPosition(currentPos);
    SERIAL_PRINT("Top limit switch hit during opening, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
    state = DoorState::Open;
    openStateFirstEntry = true;
    displayedSeekTopHint = false;
    return;
  }

  if (!stepper->isRunning())
  {
    if (!displayedSeekTopHint)
    {
      DisplayHelpers::showStatus(display, "Seek top limit", true, 1);
      displayedSeekTopHint = true;
    }
    SERIAL_PRINT("Stepper stopped before reaching top limit, re-seeking top limit\n");
    SERIAL_PRINT("Current position: %d\n expected position: %d\n", stepper->getCurrentPosition(), expectedDoorOpenPosition);
    seekLimitSwitch(1, Config::seekIncrementSteps);
  }
}

void DoorController::handleOpenState()
{
  // Record the time when entering Open state
  if (openStateFirstEntry)
  {
    openStateEnteredMs = millis();
    openStateFirstEntry = false;
    SERIAL_PRINT("Entered Open state, starting hold timer.\n");
  }
  // If a sensor is actively triggering, keep extending the hold window.
  if (openDoor)
  {
    openStateEnteredMs = millis();
    return;
  }
  // Wait for the configured hold time before closing
  if (millis() - openStateEnteredMs >= Config::doorOpenHoldMs)
  {
    if (stepper)
    {
      if (isLimitSwitchPressed(TopLimitSwitch))
      {
        expectedDoorOpenPosition = stepper->getCurrentPosition();
        SERIAL_PRINT("Top limit switch hit, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
      }
      stepper->enableOutputs();
      state = DoorState::Closing;
      displayedSeekBottomHint = false;
      stepper->moveTo(Config::expectedDoorClosePosition, false);
      openStateFirstEntry = true; // Reset for next time
    }
  }
}

void DoorController::handleClosingState()
{
  if (!stepper)
  {
    return;
  }

  if (isLimitSwitchPressedRaw(BottomLimitSwitch))
  {
    stepper->forceStopAndNewPosition(0);
    stepper->disableOutputs();
    digitalWrite(Config::enablePinStepper,
                 Config::stepperEnableActiveLow ? HIGH : LOW);
    SERIAL_PRINT("Bottom limit switch hit, stepper position set to 0. Door closed.\n");
    state = DoorState::Closed;
    displayedSeekBottomHint = false;
    fastClosing = false;
    return;
  }

  if (openDoor)
  {
    SERIAL_PRINT("Sensor trigger detected while closing, reopening door\n");
    int32_t currentPos = stepper->getCurrentPosition();
    stepper->forceStopAndNewPosition(currentPos);
    fastClosing = false;
    if (currentPos > expectedDoorOpenPosition)
    {
      expectedDoorOpenPosition = currentPos;
    }
    state = DoorState::Opening;
    displayedSeekTopHint = false;
    openStateFirstEntry = true;
    stepper->enableOutputs();
    stepper->moveTo(expectedDoorOpenPosition, false);
    return;
  }

  if (fastClosing)
  {
    if (stepper->isRunning())
    {
      return;
    }
    fastClosing = false;
  }

  if (!stepper->isRunning())
  {
    if (!displayedSeekBottomHint)
    {
      DisplayHelpers::showStatus(display, "Seek bottom limit", true, 1);
      displayedSeekBottomHint = true;
    }
    seekLimitSwitch(-1, Config::seekIncrementSteps);
  }
}

// -----------------------------------------------------------------------------
// Utility Methods
// -----------------------------------------------------------------------------
void DoorController::showStateOnDisplay()
{
  if (!display)
  {
    return;
  }
  DisplayHelpers::showStatus(display, getStateString(), false, 2);
}

bool DoorController::isLimitSwitchPressed(LimitSwitch sw)
{
  int value = limitSwitchDebouncers[sw].read();
  return value == (Config::limitSwitchActiveHigh ? HIGH : LOW);
}

bool DoorController::isLimitSwitchPressedRaw(LimitSwitch sw) const
{
  int value = digitalRead(Config::limitSwitchPins[sw]);
  return value == (Config::limitSwitchActiveHigh ? HIGH : LOW);
}

void DoorController::seekLimitSwitch(int direction, int steps)
{
  if (!stepper)
  {
    return;
  }
  if (direction > 0 && (isLimitSwitchPressedRaw(TopLimitSwitch) || isLimitSwitchPressed(TopLimitSwitch)))
  {
    SERIAL_PRINT("Top limit switch pressed. Upward seek blocked.\n");
    return;
  }
  if (direction < 0 && (isLimitSwitchPressedRaw(BottomLimitSwitch) || isLimitSwitchPressed(BottomLimitSwitch)))
  {
    SERIAL_PRINT("Bottom limit switch pressed. Downward seek blocked.\n");
    return;
  }
  stepper->move(direction * steps, false);
}
