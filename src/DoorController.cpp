#include "DoorController.h"
#include "mqtt.h"
#include "DisplayHelpers.h"
#include <Arduino.h>
#include <UMS3.h>
#include <WiFi.h>

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
  if (!keepClosed && !keepOpen) {
    updateSensorStates();
  }
  
  checkOverrideSwitches();
  handleState();

  if (sensorInitNeeded)
  {
    if (setupTOFSensors())
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
    if (keepClosed) return "KEEP CLOSED";
    switch (state) {
      case DoorState::Open:     return "OPEN";
      case DoorState::Closed:   return "CLOSED";
      case DoorState::Opening:  return "OPENING";
      case DoorState::Closing:  return "CLOSING";
      default:                  return "UNKNOWN";
    }
}

float DoorController::getDistanceIndoorCm() const {
    // Sensor 0 = indoor
    if (Config::numTOFSensors < 1) return -1.0f;
    uint16_t mm = range[0];
    if (mm == 0 || mm == Config::TOFSensorErrorValue) return -1.0f;
    return mm / 10.0f;
}

float DoorController::getDistanceOutdoorCm() const {
    // Sensor 1 = outdoor
    if (Config::numTOFSensors < 2) return -1.0f;
    uint16_t mm = range[1];
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

bool DoorController::setupTOFSensors()
{
  Wire.begin(Config::SDA, Config::SCL, 400000);
  bool retVal = true;
  for (size_t i = 0; i < Config::numTOFSensors; ++i)
  {
    pinMode(Config::xshutPins[i], OUTPUT);
    digitalWrite(Config::xshutPins[i], LOW);
    sensorReady[i] = false;
  }
  for (size_t i = 0; i < Config::numTOFSensors; ++i)
  {
    pinMode(Config::xshutPins[i], INPUT);
    delay(Config::SensorInitDelayMs);
    auto &sensor = sensors[i];
    sensor.setTimeout(Config::TOFSensorTimeout);
    if (!sensor.init())
    {
      SERIAL_PRINT("%s sensor failed to initialize!\n", Config::sensorNames[i]);
      char errorMsg[64];
      snprintf(errorMsg, sizeof(errorMsg), "ERROR:%s sensor failed!", Config::sensorNames[i]);
      DisplayHelpers::showStatus(display, errorMsg, true, 1);
      return false;
    }
    else
    {
      sensor.setAddress(Config::TOFSensorStartAddress + i);
      sensor.startContinuous();
      SERIAL_PRINT("%s sensor initialized at address: 0x%X\n", Config::sensorNames[i], sensor.getAddress());
      char statusMsg[64];
      snprintf(statusMsg, sizeof(statusMsg), "%s sensor at 0x%X", Config::sensorNames[i], sensor.getAddress());
      DisplayHelpers::showStatus(display, statusMsg);
      sensorReady[i] = true;
    }
  }
  SERIAL_PRINT("VL53L0X sensors initialized\n");
  return retVal;

}
void DoorController::setupLimitSwitches()
{
  pinMode(Config::limitSwitchPins[BottomLimitSwitch], INPUT_PULLUP);
  limitSwitchDebouncers[BottomLimitSwitch].attach(Config::limitSwitchPins[BottomLimitSwitch]);
  limitSwitchDebouncers[BottomLimitSwitch].interval(Config::DebounceIntervalMs);

  pinMode(Config::limitSwitchPins[TopLimitSwitch], INPUT_PULLUP);
  limitSwitchDebouncers[TopLimitSwitch].attach(Config::limitSwitchPins[TopLimitSwitch]);
  limitSwitchDebouncers[TopLimitSwitch].interval(Config::DebounceIntervalMs);
}

// -----------------------------------------------------------------------------
// Sensor Helpers
// -----------------------------------------------------------------------------
void DoorController::updateSensorStates()
{
  openDoor = false;
  for (size_t i = 0; i < Config::numTOFSensors; ++i)
  {
    if (!sensorReady[i])
    {
      continue;
    }

    auto &sensor = sensors[i];
    range[i] = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred())
    {
      SERIAL_PRINT("%s sensor timeout, skipping...\n", Config::sensorNames[i]);
      continue;
    }
    else if (sensor.last_status != 0)
    {
      SERIAL_PRINT("%s sensor error %d, skipping...\n", Config::sensorNames[i], sensor.last_status);
      continue;
    }
    else if (range[i] == 0)
    {
      SERIAL_PRINT("%s sensor failed to read, skipping...\n", Config::sensorNames[i]);
      continue;
    }
    else if (range[i] == Config::TOFSensorErrorValue)
    {
      SERIAL_PRINT("%s sensor not found, scheduling re-initialization...\n", Config::sensorNames[i]);
      sensorInitNeeded = true;
      sensorReady[i] = false;
      continue;
    }
    // Check if the reading is below the threshold to trigger door opening
    if (range[i] < Config::rangeThreshold[i])
    {
      openDoor = true;
      lastSensorTriggered = (i == 0) ? 1 : 2; // 1 = indoor, 2 = outdoor
      DisplayHelpers::setLastSensorTriggered(lastSensorTriggered);
      SERIAL_PRINT("%s sensor detected an object at %d mm\n", Config::sensorNames[i], range[i]);
    }
  }
}

// -----------------------------------------------------------------------------
// State Machine and Handlers
// -----------------------------------------------------------------------------
void DoorController::checkOverrideSwitches() {
    // Keep Open override
    if (digitalRead(Config::overrideKeepOpenSwitchPin) == HIGH && !keepOpen) {
      openDoor = true; // Ensure door stays open
      keepOpen = true;
      SERIAL_PRINT("Override keep open switch activated. Door will remain open.\n");
    }
    if (digitalRead(Config::overrideKeepOpenSwitchPin) == LOW && keepOpen) {
      openDoor = false;
      keepOpen = false;
      if (stepper) {
        state = DoorState::Closing;
        fastClosing = true; // <--- Add this line
        stepper->moveTo(Config::expectedDoorClosePosition, false); // Command fast close
      } else {
        state = DoorState::Closing;
      }
      SERIAL_PRINT("Override keep open switch deactivated. Resuming normal operation.\n");
    }

    // Keep Closed override
    if (digitalRead(Config::overrideKeepClosedSwitchPin) == HIGH && !keepClosed) {
      openDoor = false; // Ensure door stays closed
      keepClosed = true;
      showStateOnDisplay();
      SERIAL_PRINT("Override keep closed switch activated. Door will remain closed.\n");
    }
    if (digitalRead(Config::overrideKeepClosedSwitchPin) == LOW && keepClosed) {
      keepClosed = false;
      handleState(); // Re-evaluate state to possibly open door
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

    if (last_state != state)
    {
        showStateOnDisplay();
    }
    last_state = state;
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
    stepper->enableOutputs();
    stepper->moveTo(expectedDoorOpenPosition, false);
  }
}

void DoorController::handleOpeningState()
{
  static bool seekTopMsgShown = false;
  if (stepper)
  {
    if (isLimitSwitchPressed(TopLimitSwitch))
    {
      expectedDoorOpenPosition = stepper->getCurrentPosition();
      stepper->forceStop();
      stepper->setCurrentPosition(expectedDoorOpenPosition);  // restore the position which is lost on forceStop
      SERIAL_PRINT("Top limit switch hit during opening, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
      state = DoorState::Open;
      openStateFirstEntry = true; // Reset for Open state
      seekTopMsgShown = false;    // Reset flag after switch is hit
      return;
    }
    if (!seekTopMsgShown && !stepper->isRunning())
    {
      DisplayHelpers::showStatus(display, "Seek top limit", true, 1);
      seekTopMsgShown = true;
    }
    if (!stepper->isRunning())
    {
      SERIAL_PRINT("Stepper stopped before reaching top limit, re-seeking top limit\n");
      SERIAL_PRINT("Current position: %d\n expected position: %d\n", stepper->getCurrentPosition(), expectedDoorOpenPosition);
      seekLimitSwitch(1, Config::seekIncrementSteps);
    }
    return;
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
  // Wait for the configured hold time before closing
  if (millis() - openStateEnteredMs >= Config::doorOpenHoldMs)
  {
    if (isLimitSwitchPressed(TopLimitSwitch) && stepper)
    {
      expectedDoorOpenPosition = stepper->getCurrentPosition();
      SERIAL_PRINT("Top limit switch hit, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
      state = DoorState::Closing;
      stepper->moveTo(Config::expectedDoorClosePosition, false);
      openStateFirstEntry = true; // Reset for next time
    }
  }
}

void DoorController::handleClosingState()
{
  static bool seekBottomMsgShown = false;
  if (stepper)
  {
    if (isLimitSwitchPressed(BottomLimitSwitch))
    {
      stepper->forceStop();
      stepper->setCurrentPosition(0);
      stepper->disableOutputs();
      SERIAL_PRINT("Bottom limit switch hit, stepper position set to 0. Door closed.\n");
      state = DoorState::Closed;
      seekBottomMsgShown = false; // Reset flag after switch is hit
      fastClosing = false;        // <--- Reset fastClosing
      return;
    }
    if (!seekBottomMsgShown && !stepper->isRunning())
    {
      DisplayHelpers::showStatus(display, "Seek bottom limit", true, 1);
      seekBottomMsgShown = true;
    }
    if (openDoor)
    {
      SERIAL_PRINT("Reopening door\n");
      int32_t currentPos = stepper->getCurrentPosition();
      stepper->forceStop();
      stepper->setCurrentPosition(currentPos);  // restore the position
      state = DoorState::Closed;
      fastClosing = false; // <--- Reset fastClosing
      return;
    }
    // Only seek incrementally if not in fastClosing mode or if moveTo is done
    if (fastClosing) {
      if (!stepper->isRunning()) {
        fastClosing = false; // Finished fast close, now use incremental seek if needed
      } else {
        return; // Wait for fast close to finish
      }
    }
    seekLimitSwitch(-1, Config::seekIncrementSteps);
    return;
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
  char stateMsg[64];
  snprintf(stateMsg, sizeof(stateMsg), "%s", getStateString());
  DisplayHelpers::showStatus(display, stateMsg, false, 2);
}

bool DoorController::isLimitSwitchPressed(LimitSwitch sw)
{
  return limitSwitchDebouncers[sw].read() == HIGH;
}

void DoorController::seekLimitSwitch(int direction, int steps)
{
    if (direction > 0 && isLimitSwitchPressed(TopLimitSwitch)) {
        SERIAL_PRINT("Top limit switch pressed. Upward seek blocked.\n");
        return;
    }
    if (direction < 0 && isLimitSwitchPressed(BottomLimitSwitch)) {
        SERIAL_PRINT("Bottom limit switch pressed. Downward seek blocked.\n");
        return;
    }
    stepper->move(direction * steps, false);
}
