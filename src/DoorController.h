#pragma once
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <Preferences.h>
#include <array>
#include <cstdint>
#include "Config.h"
#include "DoorCommandSink.h"
#include "DoorStateMachine.h"
#include "DoorTelemetryProvider.h"
#include "TofSensorManager.h"

class DisplayService;
class StatusPixel;

// Controls the automated dog door hardware and logic.
class DoorController : public DoorTelemetryProvider, public DoorCommandSink
{
public:
  DoorController(StatusPixel &pixel, DisplayService &display, TofSensorManager &sensors);

  // Energize the motor as early as possible so an elevated door cannot fall
  // while the remaining hardware and network services initialize.
  void beginMotorHold();

  // Initialize all hardware and sensors.
  void setup();

  // Main loop: updates sensors, debounces switches, and runs state machine.
  void loop();

  const char *getStateString() const;
  DoorState getDoorState() const override { return stateMachine.current(); }
  const char *getDoorStateString() const override { return getStateString(); }
  float getDistanceIndoorCm() const override;
  float getDistanceOutdoorCm() const override;
  bool isWiFiConnected() const { return wifiConnected; }
  bool isFaultMotorHolding() const { return faultMotorHolding; }

  uint32_t getDoorOpenCount() const override;
  uint32_t getSensorTriggerCount(size_t index) const override;
  void refreshStateDisplay();

  // Returns debounced state of the given limit switch.
  bool isLimitSwitchPressed(LimitSwitch sw) override;
  bool isLimitSwitchPressedRaw(LimitSwitch sw) const;

  uint8_t getLastSensorTriggered() const override;

  OverrideMode getOverrideMode() const override { return mqttOverrideMode; }

  // Records a remote override request (MQTT). Applied in checkOverrideSwitches()
  // only while no physical override switch is engaged.
  void setOverrideCommand(OverrideMode mode) override;

  // Update WiFi connection status for display
  void setWiFiConnected(bool connected);

  void handleTofSensorStatus(uint8_t sensorIndex, bool ok, bool reportInitEvent);

private:
  // --- State for display icons ---
  bool wifiConnected = false;

  // Hardware setup helpers
  bool setupStepper();
  void setupLimitSwitches();

  // Sensor and state helpers
  void processSensorUpdate(const TofSensorManager::UpdateResult &update);
  void checkOverrideSwitches();
  // Drives keep-open/keep-closed edges from the combined (physical + remote)
  // override request. The two desired flags are mutually exclusive.
  void applyOverrideState(bool desiredKeepOpen, bool desiredKeepClosed);
  void handleState();
  void showStateOnDisplay();
  void handleStateTransition(DoorState from, DoorState to, const char *reason);
  void publishDistanceForSensor(uint8_t triggerId);
  void recordClosedOrClosingTrigger(uint8_t triggerId);
  void recordSensorTrigger(uint8_t triggerId);
  void loadPersistentCounters();
  void persistCounter(const char *key, uint32_t value);
  void persistCountersIfStationary();
  float distanceCmForSensor(size_t index) const;

  // State machine handlers
  void handleHomingState();
  void handleClosedState();
  void handleOpeningState();
  void handleOpenState();
  void handleClosingState();
  void enterFault(const char *reason);
  bool startHoming();
  bool startOpening(const char *reason);
  bool startClosing(const char *reason);
  void stopAtLimit(LimitSwitch limit);
  void startMotionWatch();
  bool motionLimitExceeded() const;

  void seekLimitSwitch(int direction, int steps);

  // --- Hardware interfaces ---
  FastAccelStepperEngine engine;
  FastAccelStepper *stepper = nullptr;

  // --- Sensors and switches ---
  std::array<Bounce, LimitSwitchConfig::count> limitSwitchDebouncers;

  // --- State variables ---
  bool openDoor = false;
  bool keepClosed = false;
  bool keepOpen = false;
  // Latest remote override request. Physical switches override it; defaults to
  // Auto on boot so a reboot always resumes sensor control.
  OverrideMode mqttOverrideMode = OverrideMode::Auto;
  // Tracks whether a physical override switch was engaged last loop, so a return
  // to the centre (off) position can reset the MQTT override back to Auto.
  bool physicalOverrideWasActive = false;
  bool displayedSeekTopHint = false;
  bool displayedSeekBottomHint = false;
  DoorStateMachine stateMachine;
  int32_t expectedDoorOpenPosition = 100000; // Updated at runtime based on limit switches
  StatusPixel &statusPixel;
  DisplayService &displayService;
  TofSensorManager &tofSensors;
  unsigned long openStateEnteredMs = 0;
  const char *faultReason = nullptr;
  std::array<int8_t, LimitSwitchConfig::count> lastLimitSwitchState{{-1, -1}};
  uint8_t lastSensorTriggeredClosedOrClosing = 0;
  uint32_t doorOpenCount = 0;
  std::array<uint32_t, TofSensorConfig::count> sensorTriggerCounts{{0, 0}};
  std::array<bool, TofSensorConfig::count> sensorTriggerCountDirty{{false, false}};
  bool doorOpenCountDirty = false;
  Preferences preferences;
  bool countersLoaded = false;
  unsigned long motionStartedMs = 0;
  int32_t motionStartedPosition = 0;
  bool overrideConflict = false;
  bool motorHoldInitialized = false;
  bool faultMotorHolding = false;

};
