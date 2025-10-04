#include "DoorController.h"
#include "DisplayService.h"
#include "mqtt.h"
#include <Arduino.h>
#include <UMS3.h>

// Serial debug print macro. Configured via platformio.ini
#ifdef SERIAL_PRINT_ENABLE
  #define SERIAL_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define SERIAL_PRINT(...)
#endif

DoorController::DoorController()
{
  stateMachine.setTransitionCallback([this](DoorState from, DoorState to, const char *reason) {
    handleStateTransition(from, to, reason);
  });
}

DoorController::~DoorController() = default;

void DoorController::setUMS3(UMS3 *ums3Ptr)
{
  ums3 = ums3Ptr;
}

void DoorController::setDisplayService(DisplayService *service)
{
  displayService = service;
}

void DoorController::setTofSensorManager(TofSensorManager *manager)
{
  tofSensors = manager;
}

void DoorController::setWiFiConnected(bool connected)
{
  wifiConnected = connected;
  if (displayService)
  {
    displayService->setWiFiConnected(connected);
  }
}

void DoorController::handleTofSensorStatus(uint8_t sensorIndex, bool ok, bool reportInitEvent)
{
  mqttPublishTOFSensorStatus(sensorIndex, ok);
  if (ok && reportInitEvent)
  {
    mqttPublishTOFInit(sensorIndex);
  }
}

void DoorController::setup()
{
  Serial.begin(Config.misc.serialBaudRate);
  SERIAL_PRINT("Setup Start\n");

  setupStepper();
  if (displayService)
  {
    displayService->showStatus("Stepper ready");
  }
  SERIAL_PRINT("Stepper initialized\n");

  setPixelColor(0, 0, 255);

  if (tofSensors)
  {
    tofSensors->setDisplayService(displayService);
    tofSensors->setStatusPublisher([this](uint8_t index, bool ok, bool reportInit) {
      handleTofSensorStatus(index, ok, reportInit);
    });

    if (!tofSensors->begin())
    {
      SERIAL_PRINT("ERROR: VL53L0X init failed\n");
      if (displayService)
      {
        displayService->showStatus("ERROR: VL53L0X", true);
      }
    }
    else
    {
      if (displayService)
      {
        displayService->showStatus("VL53 Sensors ready");
      }
    }
  }

  setPixelColor(255, 0, 255); // Purple

  setupLimitSwitches();
  SERIAL_PRINT("Limit switches initialized\n");
  if (displayService)
  {
    displayService->showStatus("Limit switches ready");
  }

  // Setup override toggle switch
  pinMode(Config.overrides.keepOpenPin, INPUT_PULLDOWN);
  pinMode(Config.overrides.keepClosedPin, INPUT_PULLDOWN);

  SERIAL_PRINT("Setup END\n");

  refreshStateDisplay();
}

void DoorController::loop()
{
  for (auto &debouncer : limitSwitchDebouncers)
  {
    debouncer.update();
  }

  for (size_t i = 0; i < Config.limitSwitches.count; ++i)
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

  if (tofSensors)
  {
    auto update = tofSensors->update();
    processSensorUpdate(update);
  }

  checkOverrideSwitches();
  handleState();
}

const char *DoorController::getStateString() const
{
  if (keepOpen)
    return "KEEP OPEN";
  if (keepClosed)
    return "LOCKED";

  switch (stateMachine.current())
  {
  case DoorState::Open:
    return "OPEN";
  case DoorState::Closed:
    return "CLOSED";
  case DoorState::Opening:
    return "OPENING";
  case DoorState::Closing:
    return "CLOSING";
  default:
    return "UNKNOWN";
  }
}

float DoorController::getDistanceIndoorCm() const
{
  if (!tofSensors)
  {
    return -1.0f;
  }
  return tofSensors->distanceCm(0);
}

float DoorController::getDistanceOutdoorCm() const
{
  if (!tofSensors)
  {
    return -1.0f;
  }
  return tofSensors->distanceCm(1);
}

DoorState DoorController::getCurrentState() const
{
  return stateMachine.current();
}

uint8_t DoorController::getLastSensorTriggered() const
{
  return tofSensors ? tofSensors->lastTriggerId() : 0;
}

void DoorController::refreshStateDisplay()
{
  showStateOnDisplay();
}

// -----------------------------------------------------------------------------
// Hardware Setup
// -----------------------------------------------------------------------------
void DoorController::setupStepper()
{
  engine.init();
  stepper = engine.stepperConnectToPin(Config.stepper.stepPin);
  if (stepper)
  {
    stepper->setDirectionPin(Config.stepper.dirPin);
    stepper->setEnablePin(Config.stepper.enablePin);
    stepper->setAutoEnable(false);
    stepper->setSpeedInHz(Config.stepper.stepsPerSecond);
    stepper->setAcceleration(Config.stepper.acceleration);
    stepper->setCurrentPosition(0);
  }
  else
  {
    SERIAL_PRINT("Stepper initialization failed!\n");
    if (displayService)
    {
      displayService->showStatus("ERROR: Stepper init", true);
    }
  }
}

void DoorController::setupLimitSwitches()
{
  pinMode(Config.limitSwitches.pins[BottomLimitSwitch], INPUT_PULLUP);
  limitSwitchDebouncers[BottomLimitSwitch].attach(Config.limitSwitches.pins[BottomLimitSwitch]);
  limitSwitchDebouncers[BottomLimitSwitch].interval(Config.limitSwitches.debounceMs);

  pinMode(Config.limitSwitches.pins[TopLimitSwitch], INPUT_PULLUP);
  limitSwitchDebouncers[TopLimitSwitch].attach(Config.limitSwitches.pins[TopLimitSwitch]);
  limitSwitchDebouncers[TopLimitSwitch].interval(Config.limitSwitches.debounceMs);
}

// -----------------------------------------------------------------------------
// Sensor Helpers
// -----------------------------------------------------------------------------
void DoorController::processSensorUpdate(const TofSensorManager::UpdateResult &update)
{
  if (!tofSensors)
  {
    return;
  }

  if (update.triggerEvent)
  {
    uint8_t triggerId = update.triggerSensorId;
    if (triggerId == 1)
    {
      mqttPublishDistanceIndoor(tofSensors->distanceCm(0));
    }
    else if (triggerId == 2)
    {
      mqttPublishDistanceOutdoor(tofSensors->distanceCm(1));
    }
    mqttPublishSensorTrigger(triggerId);
  }

  if (keepClosed)
  {
    openDoor = false;
  }
  else if (keepOpen)
  {
    openDoor = true;
  }
  else
  {
    openDoor = update.anySensorQualified;
  }
}

void DoorController::checkOverrideSwitches()
{
  const bool keepOpenActive = digitalRead(Config.overrides.keepOpenPin) == HIGH;
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
      stateMachine.transitionTo(DoorState::Closing, "Keep-open deactivated");
      fastClosing = true;
      displayedSeekBottomHint = false;
      stepper->moveTo(Config.stepper.expectedClosePosition, false);
    }
    else
    {
      stateMachine.transitionTo(DoorState::Closing, "Keep-open deactivated");
    }
    SERIAL_PRINT("Override keep open switch deactivated. Resuming normal operation.\n");
  }

  const bool keepClosedActive = digitalRead(Config.overrides.keepClosedPin) == HIGH;
  if (keepClosedActive && !keepClosed)
  {
    openDoor = false;
    keepClosed = true;
    if (stateMachine.current() == DoorState::Open || stateMachine.current() == DoorState::Opening)
    {
      if (stepper)
      {
        stateMachine.transitionTo(DoorState::Closing, "Keep-closed engaged");
        fastClosing = true;
        displayedSeekBottomHint = false;
        stepper->moveTo(Config.stepper.expectedClosePosition, false);
      }
      else
      {
        stateMachine.transitionTo(DoorState::Closing, "Keep-closed engaged");
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
  switch (stateMachine.current())
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
}

void DoorController::handleStateTransition(DoorState from, DoorState to, const char *reason)
{
  (void)from;
  (void)reason;
  if (displayService)
  {
    showStateOnDisplay();
  }
  if (to == DoorState::Open)
  {
    openStateFirstEntry = true;
  }
}

// --- State Handlers ---
void DoorController::handleClosedState()
{
  if (openDoor && stepper)
  {
    if (isLimitSwitchPressed(TopLimitSwitch))
    {
      return;
    }
    SERIAL_PRINT("Opening door\n");
    SERIAL_PRINT("Moving to expectedDoorOpenPosition %u\n", expectedDoorOpenPosition);
    SERIAL_PRINT("Current acceleration: %u\n", stepper->getAcceleration());
    stateMachine.transitionTo(DoorState::Opening, "Sensor trigger");
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
    stateMachine.transitionTo(DoorState::Open, "Top limit reached");
    openStateFirstEntry = true;
    displayedSeekTopHint = false;
    return;
  }

  if (!stepper->isRunning())
  {
    if (!displayedSeekTopHint && displayService)
    {
      displayService->showStatus("Seek top limit", true);
      displayedSeekTopHint = true;
    }
    SERIAL_PRINT("Stepper stopped before reaching top limit, re-seeking top limit\n");
    SERIAL_PRINT("Current position: %d\n expected position: %d\n", stepper->getCurrentPosition(), expectedDoorOpenPosition);
    seekLimitSwitch(1, Config.stepper.seekIncrementSteps);
  }
}

void DoorController::handleOpenState()
{
  if (openStateFirstEntry)
  {
    openStateEnteredMs = millis();
    openStateFirstEntry = false;
    SERIAL_PRINT("Entered Open state, starting hold timer.\n");
  }
  if (openDoor)
  {
    openStateEnteredMs = millis();
    return;
  }
  if (millis() - openStateEnteredMs >= Config.timing.doorOpenHoldMs)
  {
    if (stepper)
    {
      if (isLimitSwitchPressed(TopLimitSwitch))
      {
        expectedDoorOpenPosition = stepper->getCurrentPosition();
        SERIAL_PRINT("Top limit switch hit, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
      }
      stepper->enableOutputs();
      stateMachine.transitionTo(DoorState::Closing, "Hold period expired");
      displayedSeekBottomHint = false;
      stepper->moveTo(Config.stepper.expectedClosePosition, false);
      openStateFirstEntry = true;
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
    digitalWrite(Config.stepper.enablePin,
                 Config.stepper.enableActiveLow ? HIGH : LOW);
    SERIAL_PRINT("Bottom limit switch hit, stepper position set to 0. Door closed.\n");
    stateMachine.transitionTo(DoorState::Closed, "Bottom limit reached");
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
    stateMachine.transitionTo(DoorState::Opening, "Sensor during close");
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
    if (!displayedSeekBottomHint && displayService)
    {
      displayService->showStatus("Seek bottom limit", true);
      displayedSeekBottomHint = true;
    }
    seekLimitSwitch(-1, Config.stepper.seekIncrementSteps);
  }
}

// -----------------------------------------------------------------------------
// Utility Methods
// -----------------------------------------------------------------------------
void DoorController::showStateOnDisplay()
{
  if (!displayService)
  {
    return;
  }
  displayService->showStatus(getStateString(), false, 2);
}

bool DoorController::isLimitSwitchPressed(LimitSwitch sw)
{
  int value = limitSwitchDebouncers[sw].read();
  return value == (Config.limitSwitches.activeHigh ? HIGH : LOW);
}

bool DoorController::isLimitSwitchPressedRaw(LimitSwitch sw) const
{
  int value = digitalRead(Config.limitSwitches.pins[sw]);
  return value == (Config.limitSwitches.activeHigh ? HIGH : LOW);
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

void DoorController::setPixelColor(uint8_t r, uint8_t g, uint8_t b)
{
  if (ums3)
  {
    ums3->setPixelColor(r, g, b);
  }
}
