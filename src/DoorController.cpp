#include "DoorController.h"
#include "DoorSafetyPolicy.h"
#include "MotionSafetyPolicy.h"
#include "DisplayService.h"
#include "StatusPixel.h"
#include "mqtt.h"
#include <Arduino.h>

// Serial debug print macro. Configured via platformio.ini
#ifdef SERIAL_PRINT_ENABLE
  #define SERIAL_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define SERIAL_PRINT(...)
#endif

DoorController::DoorController(StatusPixel &pixel, DisplayService &display, TofSensorManager &sensors)
    : statusPixel(pixel), displayService(display), tofSensors(sensors)
{
  stateMachine.setTransitionCallback([this](DoorState from, DoorState to, const char *reason) {
    handleStateTransition(from, to, reason);
  });
}

void DoorController::setWiFiConnected(bool connected)
{
  wifiConnected = connected;
  displayService.setWiFiConnected(connected);
}

void DoorController::beginMotorHold()
{
  if (motorHoldInitialized)
  {
    return;
  }

  Serial.begin(Config.misc.serialBaudRate);
  loadPersistentCounters();
  motorHoldInitialized = setupStepper();
  if (motorHoldInitialized)
  {
    SERIAL_PRINT("Motor energized for startup hold.\n");
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
  SERIAL_PRINT("Setup Start\n");

  beginMotorHold();
  if (motorHoldInitialized)
  {
    displayService.showStatus("Stepper ready");
  }
  SERIAL_PRINT("Stepper initialized\n");

  statusPixel.setColor(0, 0, 255);

  tofSensors.setStatusPublisher([this](uint8_t index, bool ok, bool reportInit) {
    handleTofSensorStatus(index, ok, reportInit);
  });

  if (!tofSensors.begin())
  {
    SERIAL_PRINT("ERROR: VL53L0X init failed\n");
    displayService.showStatus("ERROR: VL53L0X", true);
  }
  else
  {
    displayService.showStatus("VL53 Sensors ready");
  }

  statusPixel.setColor(255, 0, 255); // Purple

  setupLimitSwitches();
  SERIAL_PRINT("Limit switches initialized\n");
  displayService.showStatus("Limit switches ready");

  // Setup override toggle switch
  pinMode(Config.overrides.keepOpenPin, INPUT_PULLDOWN);
  pinMode(Config.overrides.keepClosedPin, INPUT_PULLDOWN);

  if (!stepper)
  {
    enterFault("Stepper unavailable");
  }
  else if (isLimitSwitchPressedRaw(BottomLimitSwitch) && isLimitSwitchPressedRaw(TopLimitSwitch))
  {
    enterFault("Both limit switches active");
  }
  else if (isLimitSwitchPressedRaw(BottomLimitSwitch))
  {
    if (stepper)
    {
      stepper->setCurrentPosition(Config.stepper.expectedClosePosition);
    }
  }
  else
  {
    startHoming();
  }

  SERIAL_PRINT("Setup END\n");

  refreshStateDisplay();
}

void DoorController::loop()
{
  // Raw limit checks must run before sensor reads or telemetry work. This gives
  // endpoint detection priority over every non-safety task in the controller.
  handleState();
  if (stateMachine.current() == DoorState::Fault)
  {
    return;
  }

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
        mqttPublishLimitSwitchBottom(pressed);
      }
      else if (i == TopLimitSwitch)
      {
        mqttPublishLimitSwitchTop(pressed);
      }
    }
  }

  if (shouldPollTofSensors(stateMachine.current()))
  {
    auto update = tofSensors.update();
    processSensorUpdate(update);
  }

  checkOverrideSwitches();
  handleState();
  persistCountersIfStationary();
}

const char *DoorController::getStateString() const
{
  if (stateMachine.current() == DoorState::Fault)
    return faultReason ? faultReason : "FAULT";
  if (keepOpen)
    return "KEEP OPEN";
  if (keepClosed)
    return "LOCKED";

  switch (stateMachine.current())
  {
  case DoorState::Homing:
    return "HOMING";
  case DoorState::Open:
    return "OPEN";
  case DoorState::Closed:
    return "CLOSED";
  case DoorState::Opening:
    return "OPENING";
  case DoorState::Closing:
    return "CLOSING";
  case DoorState::Fault:
    return "FAULT";
  default:
    return "UNKNOWN";
  }
}

float DoorController::getDistanceIndoorCm() const
{
  return distanceCmForSensor(IndoorSensor);
}

float DoorController::getDistanceOutdoorCm() const
{
  return distanceCmForSensor(OutdoorSensor);
}

uint32_t DoorController::getDoorOpenCount() const
{
  return doorOpenCount;
}

uint32_t DoorController::getSensorTriggerCount(size_t index) const
{
  return index < sensorTriggerCounts.size() ? sensorTriggerCounts[index] : 0;
}

uint8_t DoorController::getLastSensorTriggered() const
{
  return lastSensorTriggeredClosedOrClosing;
}

void DoorController::refreshStateDisplay()
{
  showStateOnDisplay();
}

// -----------------------------------------------------------------------------
// Hardware Setup
// -----------------------------------------------------------------------------
bool DoorController::setupStepper()
{
  // Keep the active-low driver disabled throughout reset-time initialization.
  pinMode(Config.stepper.enablePin, OUTPUT);
  digitalWrite(Config.stepper.enablePin,
               Config.stepper.enableActiveLow ? HIGH : LOW);

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
    // Do not command any movement yet. Energizing the coils holds the current
    // mechanical position while the rest of startup determines door position.
    stepper->enableOutputs();
    return true;
  }
  else
  {
    SERIAL_PRINT("Stepper initialization failed!\n");
    displayService.showStatus("ERROR: Stepper init", true);
    return false;
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
  DoorState currentState = stateMachine.current();
  bool doorClosedOrClosing = (currentState == DoorState::Closed || currentState == DoorState::Closing);

  if (update.triggerEvent)
  {
    uint8_t triggerId = update.triggerSensorId;
    publishDistanceForSensor(triggerId);
    recordSensorTrigger(triggerId);

    if (doorClosedOrClosing)
    {
      recordClosedOrClosingTrigger(triggerId);
    }
  }

  openDoor = shouldRequestDoorOpen(currentState, update.anySensorQualified,
                                   keepOpen, keepClosed);
}

float DoorController::distanceCmForSensor(size_t index) const
{
  return tofSensors.distanceCm(index);
}

void DoorController::publishDistanceForSensor(uint8_t triggerId)
{
  switch (triggerId)
  {
  case 1:
    mqttPublishDistanceIndoor(tofSensors.distanceCm(0));
    break;
  case 2:
    mqttPublishDistanceOutdoor(tofSensors.distanceCm(1));
    break;
  default:
    break;
  }
}

void DoorController::recordClosedOrClosingTrigger(uint8_t triggerId)
{
  lastSensorTriggeredClosedOrClosing = triggerId;
  displayService.setLastSensorTriggered(triggerId);
}

void DoorController::recordSensorTrigger(uint8_t triggerId)
{
  if (triggerId == 0 || triggerId > sensorTriggerCounts.size())
  {
    return;
  }

  const size_t index = triggerId - 1;
  ++sensorTriggerCounts[index];
  sensorTriggerCountDirty[index] = true;
  mqttPublishSensorTrigger(triggerId);
  mqttPublishSensorTriggerCount(index, sensorTriggerCounts[index]);
}

void DoorController::loadPersistentCounters()
{
  if (countersLoaded)
  {
    return;
  }
  countersLoaded = preferences.begin("dogdoor", false);
  if (!countersLoaded)
  {
    SERIAL_PRINT("WARN: Persistent counters unavailable\n");
    return;
  }
  doorOpenCount = preferences.getUInt("open_count", 0);
  sensorTriggerCounts[0] = preferences.getUInt("indoor_count", 0);
  sensorTriggerCounts[1] = preferences.getUInt("outdoor_count", 0);
}

void DoorController::persistCounter(const char *key, uint32_t value)
{
  if (countersLoaded && preferences.putUInt(key, value) == 0)
  {
    SERIAL_PRINT("WARN: Failed to persist counter %s\n", key);
  }
}

void DoorController::persistCountersIfStationary()
{
  const DoorState state = stateMachine.current();
  if (state != DoorState::Closed && state != DoorState::Open)
  {
    return;
  }

  if (doorOpenCountDirty)
  {
    persistCounter("open_count", doorOpenCount);
    doorOpenCountDirty = false;
  }
  for (size_t i = 0; i < sensorTriggerCounts.size(); ++i)
  {
    if (sensorTriggerCountDirty[i])
    {
      persistCounter(i == 0 ? "indoor_count" : "outdoor_count", sensorTriggerCounts[i]);
      sensorTriggerCountDirty[i] = false;
    }
  }
}

void DoorController::setOverrideCommand(OverrideMode mode)
{
  // Only record the request here; it may arrive from the MQTT callback outside
  // the door loop. checkOverrideSwitches() applies it on the next iteration so
  // every motion decision still happens in one place.
  mqttOverrideMode = mode;
}

void DoorController::checkOverrideSwitches()
{
  if (stateMachine.current() == DoorState::Homing)
  {
    return;
  }

  const bool physKeepOpen = digitalRead(Config.overrides.keepOpenPin) == HIGH;
  const bool physKeepClosed = digitalRead(Config.overrides.keepClosedPin) == HIGH;
  if (physKeepOpen && physKeepClosed)
  {
    if (!overrideConflict)
    {
      overrideConflict = true;
      enterFault("Conflicting overrides");
    }
    return;
  }
  overrideConflict = false;

  // Returning the 3-position toggle to its centre (off) position clears any
  // active MQTT override, so centre-off always means Auto. Operating the physical
  // switch therefore takes control back from Home Assistant.
  const bool physicalOverrideActive = physKeepOpen || physKeepClosed;
  if (physicalOverrideWasActive && !physicalOverrideActive)
  {
    mqttOverrideMode = OverrideMode::Auto;
  }
  physicalOverrideWasActive = physicalOverrideActive;

  // Hardwired override switches are the master control. The remote (MQTT)
  // override only takes effect when neither physical switch is engaged, so a
  // local switch can never be overridden from Home Assistant.
  bool desiredKeepOpen = physKeepOpen;
  bool desiredKeepClosed = physKeepClosed;
  if (!physicalOverrideActive)
  {
    desiredKeepOpen = (mqttOverrideMode == OverrideMode::KeepOpen);
    desiredKeepClosed = (mqttOverrideMode == OverrideMode::KeepClosed);
  }

  applyOverrideState(desiredKeepOpen, desiredKeepClosed);
}

void DoorController::applyOverrideState(bool desiredKeepOpen, bool desiredKeepClosed)
{
  if (desiredKeepOpen && !keepOpen)
  {
    openDoor = true;
    keepOpen = true;
    SERIAL_PRINT("Keep-open override engaged. Door will remain open.\n");
  }
  else if (!desiredKeepOpen && keepOpen)
  {
    keepOpen = false;
    openDoor = false;
    startClosing("Keep-open deactivated");
    SERIAL_PRINT("Keep-open override released. Resuming normal operation.\n");
  }

  if (desiredKeepClosed && !keepClosed)
  {
    openDoor = false;
    keepClosed = true;
    if (stateMachine.current() == DoorState::Open || stateMachine.current() == DoorState::Opening)
    {
      startClosing("Keep-closed engaged");
    }
    showStateOnDisplay();
    SERIAL_PRINT("Keep-closed override engaged. Door will remain closed.\n");
  }
  else if (!desiredKeepClosed && keepClosed)
  {
    keepClosed = false;
    handleState();
    showStateOnDisplay();
    SERIAL_PRINT("Keep-closed override released. Resuming normal operation.\n");
  }
}

void DoorController::handleState()
{
  // Faults are latched until restart. Avoid repeatedly stopping and
  // re-publishing the same fault on every loop iteration.
  if (stateMachine.current() == DoorState::Fault)
  {
    return;
  }

  const bool bottomPressed = isLimitSwitchPressedRaw(BottomLimitSwitch);
  const bool topPressed = isLimitSwitchPressedRaw(TopLimitSwitch);
  switch (limitSafetyAction(stateMachine.current(), bottomPressed, topPressed))
  {
  case LimitSafetyAction::FaultConflictingLimits:
    enterFault("Both limit switches active");
    return;
  case LimitSafetyAction::CompleteHoming:
    handleHomingState();
    return;
  case LimitSafetyAction::CompleteOpening:
    handleOpeningState();
    return;
  case LimitSafetyAction::CompleteClosing:
    handleClosingState();
    return;
  case LimitSafetyAction::None:
    break;
  }
  switch (stateMachine.current())
  {
  case DoorState::Homing:
    handleHomingState();
    break;
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
  case DoorState::Fault:
    break;
  }
}

void DoorController::handleStateTransition(DoorState from, DoorState to, const char *reason)
{
  (void)from;
  (void)reason;
  showStateOnDisplay();
  if (to == DoorState::Open)
  {
    openStateEnteredMs = millis();
    ++doorOpenCount;
    doorOpenCountDirty = true;
    mqttPublishDoorOpenCount(doorOpenCount);
  }
}

// --- State Handlers ---
void DoorController::handleHomingState()
{
  if (!stepper)
  {
    enterFault("Stepper unavailable");
    return;
  }

  if (isLimitSwitchPressedRaw(BottomLimitSwitch))
  {
    stopAtLimit(BottomLimitSwitch);
    stepper->setSpeedInHz(Config.stepper.stepsPerSecond);
    stepper->setAcceleration(Config.stepper.acceleration);
    stateMachine.transitionTo(DoorState::Closed, "Homing complete");
    SERIAL_PRINT("Bottom limit reached; homing complete.\n");
    return;
  }

  if (motionLimitExceeded())
  {
    enterFault("Homing limit exceeded");
    return;
  }

  if (!stepper->isRunning())
  {
    enterFault("Bottom limit not found");
  }
}

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
    startOpening("Sensor trigger");
  }
}

void DoorController::handleOpeningState()
{
  if (!stepper)
  {
    return;
  }

  if (motionLimitExceeded())
  {
    enterFault("Opening limit exceeded");
    return;
  }

  if (isLimitSwitchPressedRaw(TopLimitSwitch))
  {
    stopAtLimit(TopLimitSwitch);
    SERIAL_PRINT("Top limit switch hit during opening, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
    stateMachine.transitionTo(DoorState::Open, "Top limit reached");
    displayedSeekTopHint = false;
    return;
  }

  if (!stepper->isRunning())
  {
    if (!displayedSeekTopHint)
    {
      displayService.showStatus("Seek top limit", true);
      displayedSeekTopHint = true;
    }
    SERIAL_PRINT("Stepper stopped before reaching top limit, re-seeking top limit\n");
    SERIAL_PRINT("Current position: %d\n expected position: %d\n", stepper->getCurrentPosition(), expectedDoorOpenPosition);
    seekLimitSwitch(1, Config.stepper.seekIncrementSteps);
  }
}

void DoorController::handleOpenState()
{
  if (openDoor)
  {
    openStateEnteredMs = millis();
    return;
  }
  if (durationElapsed(static_cast<uint32_t>(millis()),
                      static_cast<uint32_t>(openStateEnteredMs),
                      static_cast<uint32_t>(Config.timing.doorOpenHoldMs)))
  {
    if (stepper)
    {
      if (isLimitSwitchPressed(TopLimitSwitch))
      {
        expectedDoorOpenPosition = stepper->getCurrentPosition();
        SERIAL_PRINT("Top limit switch hit, updating expectedDoorOpenPosition to %u\n", expectedDoorOpenPosition);
      }
      startClosing("Hold period expired");
    }
  }
}

void DoorController::handleClosingState()
{
  if (!stepper)
  {
    return;
  }

  if (motionLimitExceeded())
  {
    enterFault("Closing limit exceeded");
    return;
  }

  if (isLimitSwitchPressedRaw(BottomLimitSwitch))
  {
    stopAtLimit(BottomLimitSwitch);
    SERIAL_PRINT("Bottom limit switch hit, stepper position set to 0. Door closed.\n");
    stateMachine.transitionTo(DoorState::Closed, "Bottom limit reached");
    displayedSeekBottomHint = false;
    return;
  }

  if (openDoor)
  {
    SERIAL_PRINT("Sensor trigger detected while closing, reopening door\n");
    int32_t currentPos = stepper->getCurrentPosition();
    stepper->forceStopAndNewPosition(currentPos);
    if (currentPos > expectedDoorOpenPosition)
    {
      expectedDoorOpenPosition = currentPos;
    }
    startOpening("Sensor during close");
    return;
  }

  if (!stepper->isRunning())
  {
    if (!displayedSeekBottomHint)
    {
      displayService.showStatus("Seek bottom limit", true);
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
  displayService.showStatus(getStateString(), false, 2);
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

void DoorController::startMotionWatch()
{
  motionStartedMs = millis();
  motionStartedPosition = stepper ? stepper->getCurrentPosition() : 0;
}

bool DoorController::startOpening(const char *reason)
{
  if (!stepper)
  {
    enterFault("Stepper unavailable");
    return false;
  }
  displayedSeekTopHint = false;
  stepper->enableOutputs();
  startMotionWatch();
  stateMachine.transitionTo(DoorState::Opening, reason);
  stepper->moveTo(expectedDoorOpenPosition, false);
  return true;
}

bool DoorController::startHoming()
{
  if (!stepper)
  {
    enterFault("Stepper unavailable");
    return false;
  }

  openDoor = false;
  stepper->setSpeedInHz(Config.stepper.homingStepsPerSecond);
  stepper->setAcceleration(Config.stepper.homingAcceleration);
  stepper->enableOutputs();
  startMotionWatch();
  stateMachine.transitionTo(DoorState::Homing, "Position unknown");
  stepper->move(-Config.stepper.maximumTravelSteps, false);
  SERIAL_PRINT("Door position unknown; homing toward bottom limit.\n");
  return true;
}

bool DoorController::startClosing(const char *reason)
{
  if (!stepper)
  {
    enterFault("Stepper unavailable");
    return false;
  }
  displayedSeekBottomHint = false;
  stepper->enableOutputs();
  startMotionWatch();
  stateMachine.transitionTo(DoorState::Closing, reason);
  stepper->moveTo(Config.stepper.expectedClosePosition, false);
  return true;
}

void DoorController::stopAtLimit(LimitSwitch limit)
{
  if (!stepper)
  {
    return;
  }
  const int32_t position = limit == BottomLimitSwitch
                               ? Config.stepper.expectedClosePosition
                               : stepper->getCurrentPosition();
  stepper->forceStopAndNewPosition(position);
  if (limit == TopLimitSwitch)
  {
    expectedDoorOpenPosition = position;
  }
  else
  {
    stepper->disableOutputs();
  }
}

bool DoorController::motionLimitExceeded() const
{
  if (!stepper)
  {
    return true;
  }
  const unsigned long timeout = stateMachine.current() == DoorState::Homing
                                    ? Config.stepper.maximumHomingMs
                                    : Config.stepper.maximumMotionMs;
  return motionBoundsExceeded(stepper->getCurrentPosition(), motionStartedPosition,
                              Config.stepper.maximumTravelSteps,
                              static_cast<uint32_t>(millis()),
                              static_cast<uint32_t>(motionStartedMs),
                              static_cast<uint32_t>(timeout));
}

void DoorController::enterFault(const char *reason)
{
  if (stateMachine.current() == DoorState::Fault)
  {
    return;
  }

  openDoor = false;
  faultReason = reason;
  faultMotorHolding = false;
  if (stepper)
  {
    const int32_t position = stepper->getCurrentPosition();
    stepper->forceStopAndNewPosition(position);
    // A stopped but de-energized motor can allow an elevated door to fall.
    // Keep full holding torque in every fault; only power loss or a deliberate
    // restart/recovery should release it.
    stepper->enableOutputs();
    faultMotorHolding = true;
  }
  stateMachine.transitionTo(DoorState::Fault, reason);
  char message[64];
  snprintf(message, sizeof(message), "FAULT HOLD: %s", reason ? reason : "Door fault");
  displayService.showStatus(message, true);
  SERIAL_PRINT("FAULT: %s; motor %s\n",
               reason ? reason : "Door fault",
               faultMotorHolding ? "holding" : "unavailable");
}
