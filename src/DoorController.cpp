#include "DoorController.h"
#include "IconBitmaps.h"
#include "DisplayHelpers.h"
#include <Arduino.h>
#include <UMS3.h>
#include <WiFi.h>
#include "IconBitmaps.h"
// Implementation for setting WiFi status
void DoorController::setWiFiConnected(bool connected) {
  wifiConnected = connected;
  DisplayHelpers::setWiFiConnected(connected);
}

// -----------------------------------------------------------------------------
// Constructor & Destructor
// -----------------------------------------------------------------------------
DoorController::DoorController() {}

DoorController::~DoorController() {
  for (size_t i = 0; i < Config::numTOFSensors; ++i) {
    delete sensors[i];
    sensors[i] = nullptr;
  }
}

// -----------------------------------------------------------------------------
// Public Interface
// -----------------------------------------------------------------------------
void DoorController::setup() {
  Serial.begin(Config::SerialBaudRate);
  delay(Config::SetupDelayMs);

  DisplayHelpers::showStatus(display, "DoorController Setup");
  Serial.printf("Setup Start\n");

  setupStepper();
  DisplayHelpers::showStatus(display, "Stepper ready");
  Serial.printf("Stepper initialized\n");

  setPixelColor(0, 0, 255);

  while (!setupTOFSensors()) {
    Serial.printf("Retrying VL53L0X sensor initialization...\n");
    DisplayHelpers::showStatus(display, "Retrying VL53L0X init");
    setPixelColor(0, 0, 0);
    delay(Config::SetupDelayMs / 2);
    setPixelColor(0, 0, 255);
    delay(Config::SetupDelayMs / 2);
  }
  Serial.printf("VL53L0X Sensors initialized successfully\n");

  DisplayHelpers::showStatus(display, "VL53L0X Sensors ready");

  setPixelColor(255, 0, 255); // Purple

  setupLimitSwitches();
  Serial.printf("Limit switches initialized\n");

  DisplayHelpers::showStatus(display, "Limit switches ready");

  Serial.printf("Setup END\n");

  DisplayHelpers::showStatus(display, "Ready!", false, 2);
}

void DoorController::loop() {
  for (auto& debouncer : limitSwitchDebouncers) {
    debouncer.update();
  }
  // Periodically check WiFi status for icon (every wifiCheckIntervalMs)
  unsigned long now = millis();
  if (now - lastWiFiCheckMs >= wifiCheckIntervalMs) {
    wifiConnected = (WiFi.status() == WL_CONNECTED);
    DisplayHelpers::setWiFiConnected(wifiConnected);
    lastWiFiCheckMs = now;
  }
  updateSensorStates();
  handleState();

  if (sensorInitNeeded) {
    if (setupTOFSensors()) {
      sensorInitNeeded = false;
    }
  }
}

void DoorController::setUMS3(UMS3* ums3Ptr) {
  ums3 = ums3Ptr;
}

void DoorController::setDisplay(Adafruit_SSD1306* displayPtr) {
  display = displayPtr;
}

void DoorController::setPixelColor(uint8_t r, uint8_t g, uint8_t b) {
  if (ums3) {
    ums3->setPixelColor(r, g, b);
  }
}

const char* DoorController::getStateString() const {
  switch (state) {
    case DoorState::Closed:   return "Closed";
    case DoorState::Opening:  return "Opening";
    case DoorState::Open:     return "Open";
    case DoorState::Closing:  return "Closing";
    default:                  return "Unknown";
  }
}

// -----------------------------------------------------------------------------
// Hardware Setup
// -----------------------------------------------------------------------------
void DoorController::setupStepper() {
  engine.init();
  stepper = engine.stepperConnectToPin(Config::stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(Config::dirPinStepper);
    stepper->setEnablePin(Config::enablePinStepper);
    stepper->setAutoEnable(true);
    stepper->setSpeedInHz(Config::stepsPerSecond);
    stepper->setAcceleration(Config::acceleration);
    stepper->setCurrentPosition(0);
  } else {
    Serial.printf("Stepper initialization failed!\n");
    DisplayHelpers::showStatus(display, "ERROR: Stepper init failed!");
  }
}

bool DoorController::setupTOFSensors() {
  Wire.begin(Config::SDA, Config::SCL, 400000);
  bool retVal = true;
  for (size_t i = 0; i < Config::numTOFSensors; ++i) {
    pinMode(Config::xshutPins[i], OUTPUT);
    digitalWrite(Config::xshutPins[i], LOW);
  }
  for (size_t i = 0; i < Config::numTOFSensors; ++i) {
    pinMode(Config::xshutPins[i], INPUT);
    delay(Config::SensorInitDelayMs);
    if (sensors[i]) {
      delete sensors[i];
      sensors[i] = nullptr;
    }
    sensors[i] = new VL53L0X();
    sensors[i]->setTimeout(Config::TOFSensorTimeout);
    if (!sensors[i]->init()) {
      Serial.printf("%s sensor failed to initialize!\n", Config::sensorNames[i]);
      char errorMsg[64];
      snprintf(errorMsg, sizeof(errorMsg), "ERROR:%s sensor failed!", Config::sensorNames[i]);
      DisplayHelpers::showStatus(display, errorMsg, true, 1);
      return false;
    } else {
      sensors[i]->setAddress(Config::TOFSensorStartAddress + i);
      sensors[i]->startContinuous();
      Serial.printf("%s sensor initialized at address: 0x%X\n", Config::sensorNames[i], sensors[i]->getAddress());
      char statusMsg[64];
      snprintf(statusMsg, sizeof(statusMsg), "%s sensor at 0x%X", Config::sensorNames[i], sensors[i]->getAddress());
      DisplayHelpers::showStatus(display, statusMsg);
    }
  }
  Serial.printf("VL53L0X sensors initialized\n");
  return retVal;
}

void DoorController::setupLimitSwitches() {
  pinMode(Config::limitSwitchPins[BottomLimitSwitch], INPUT_PULLDOWN);
  limitSwitchDebouncers[BottomLimitSwitch].attach(Config::limitSwitchPins[BottomLimitSwitch]);
  limitSwitchDebouncers[BottomLimitSwitch].interval(Config::DebounceIntervalMs);

  pinMode(Config::limitSwitchPins[TopLimitSwitch], INPUT_PULLDOWN);
  limitSwitchDebouncers[TopLimitSwitch].attach(Config::limitSwitchPins[TopLimitSwitch]);
  limitSwitchDebouncers[TopLimitSwitch].interval(Config::DebounceIntervalMs);
}

// -----------------------------------------------------------------------------
// Sensor Helpers
// -----------------------------------------------------------------------------
void DoorController::updateSensorStates() {
  openDoor = false;
  for (size_t i = 0; i < Config::numTOFSensors; ++i) {
    range[i] = sensors[i]->readRangeContinuousMillimeters();
    if (sensors[i]->timeoutOccurred()) {
      Serial.printf("%s sensor timeout, skipping...\n", Config::sensorNames[i]);
      continue;
    } else if (range[i] == 0) {
      Serial.printf("%s sensor failed to read, skipping...\n", Config::sensorNames[i]);
      continue;
    } else if (range[i] == Config::TOFSensorErrorValue) {
      Serial.printf("%s sensor not found, scheduling re-initialization...\n", Config::sensorNames[i]);
      sensorInitNeeded = true;
      continue;
    }
    if (range[i] < Config::rangeThreshold[i]) {
      openDoor = true;
      lastSensorTriggered = (i == 0) ? 1 : 2; // 1 = indoor, 2 = outdoor
      DisplayHelpers::setLastSensorTriggered(lastSensorTriggered);
      Serial.printf("%s sensor detected an object at %d mm\n", Config::sensorNames[i], range[i]);
    }
  }
}

// -----------------------------------------------------------------------------
// State Machine and Handlers
// -----------------------------------------------------------------------------
void DoorController::handleState() {

  switch (state) {
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
  if (last_state != state) {
    char stateMsg[64];
    snprintf(stateMsg, sizeof(stateMsg), "%s", getStateString());
    DisplayHelpers::showStatus(display, stateMsg, false, 2);
  }
  last_state = state;
}

// --- State Handlers ---
void DoorController::handleClosedState() {
  if (openDoor && stepper) {
    Serial.printf("Opening door\n");
    state = DoorState::Opening;
    stepper->enableOutputs();
    stepper->moveTo(expectedDoorOpenPosition, true);
  }
}

void DoorController::handleOpeningState() {
  static bool seekTopMsgShown = false;
  if (stepper) {
    if (isLimitSwitchPressed(TopLimitSwitch)) {
      expectedDoorOpenPosition = stepper->getCurrentPosition();
      Serial.printf("Top limit switch hit during opening, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
      state = DoorState::Open;
      openStateFirstEntry = true; // Reset for Open state
      seekTopMsgShown = false; // Reset flag after switch is hit
      return;
    }
    if (!seekTopMsgShown) {
      DisplayHelpers::showStatus(display, "Seek top limit", true, 1);
      seekTopMsgShown = true;
    }
    seekLimitSwitch(1, Config::seekIncrementSteps);
    return;
  }
}

void DoorController::handleOpenState() {
  // Record the time when entering Open state
  if (openStateFirstEntry) {
    openStateEnteredMs = millis();
    openStateFirstEntry = false;
    Serial.printf("Entered Open state, starting hold timer.\n");
  }
  // Wait for the configured hold time before closing
  if (millis() - openStateEnteredMs >= Config::doorOpenHoldMs) {
    if (isLimitSwitchPressed(TopLimitSwitch) && stepper) {
      expectedDoorOpenPosition = stepper->getCurrentPosition();
      Serial.printf("Top limit switch hit, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
      state = DoorState::Closing;
      stepper->moveTo(Config::expectedDoorClosePosition, false);
      openStateFirstEntry = true; // Reset for next time
    }
  }
}

void DoorController::handleClosingState() {
  static bool seekBottomMsgShown = false;
  if (stepper) {
    if (isLimitSwitchPressed(BottomLimitSwitch)) {
      stepper->setCurrentPosition(0);
      stepper->disableOutputs();
      Serial.printf("Bottom limit switch hit, stepper position set to 0. Door closed.\n");
      state = DoorState::Closed;
      seekBottomMsgShown = false; // Reset flag after switch is hit
      return;
    }
    if (!seekBottomMsgShown) {
      DisplayHelpers::showStatus(display, "Seek bottom limit", true, 1);
      seekBottomMsgShown = true;
    }
    seekLimitSwitch(-1, Config::seekIncrementSteps);
    return;
  }
  if (openDoor) {
    Serial.printf("Reopening door\n");
    state = DoorState::Closed;
  }
}

// -----------------------------------------------------------------------------
// Utility Methods
// -----------------------------------------------------------------------------
bool DoorController::isLimitSwitchPressed(LimitSwitch sw) {
  return limitSwitchDebouncers[sw].read() == HIGH;
}



void DoorController::seekLimitSwitch(int direction, int steps) {
  if (stepper && !stepper->isRunning()) {
    stepper->move(direction * steps, true);
  }
}