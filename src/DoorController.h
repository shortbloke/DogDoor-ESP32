#pragma once
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <array>
#include <cstdint>
#include "Config.h"
#include "DoorStateMachine.h"
#include "TofSensorManager.h"

// Forward declaration for UMS3
class UMS3;
class DisplayService;

// Controls the automated dog door hardware and logic.
class DoorController
{
public:
  DoorController();
  ~DoorController();

  // Initialize all hardware and sensors.
  void setup();

  // Main loop: updates sensors, debounces switches, and runs state machine.
  void loop();

  void setUMS3(UMS3 *ums3Ptr);
  void setDisplayService(DisplayService *service);
  void setTofSensorManager(TofSensorManager *manager);

  const char *getStateString() const;
  float getDistanceIndoorCm() const;
  float getDistanceOutdoorCm() const;
  DoorState getCurrentState() const;
  bool isWiFiConnected() const { return wifiConnected; }

  void refreshStateDisplay();

  // Returns debounced state of the given limit switch.
  bool isLimitSwitchPressed(LimitSwitch sw);
  bool isLimitSwitchPressedRaw(LimitSwitch sw) const;

  uint8_t getLastSensorTriggered() const;

  // Update WiFi connection status for display
  void setWiFiConnected(bool connected);

  void handleTofSensorStatus(uint8_t sensorIndex, bool ok, bool reportInitEvent);

private:
  // --- State for display icons ---
  bool wifiConnected = false;

  // Hardware setup helpers
  void setupStepper();
  void setupLimitSwitches();

  // Sensor and state helpers
  void processSensorUpdate(const TofSensorManager::UpdateResult &update);
  void checkOverrideSwitches();
  void handleState();
  void showStateOnDisplay();
  void handleStateTransition(DoorState from, DoorState to, const char *reason);
  void publishDistanceForSensor(uint8_t triggerId);
  void recordClosedOrClosingTrigger(uint8_t triggerId);

  // State machine handlers
  void handleClosedState();
  void handleOpeningState();
  void handleOpenState();
  void handleClosingState();

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
  bool fastClosing = false;
  bool displayedSeekTopHint = false;
  bool displayedSeekBottomHint = false;
  DoorStateMachine stateMachine;
  int32_t expectedDoorOpenPosition = 100000; // Updated at runtime based on limit switches
  UMS3 *ums3 = nullptr;
  DisplayService *displayService = nullptr;
  TofSensorManager *tofSensors = nullptr;
  unsigned long openStateEnteredMs = 0;
  bool openStateFirstEntry = true;
  std::array<int8_t, LimitSwitchConfig::count> lastLimitSwitchState{{-1, -1}};
  uint8_t lastSensorTriggeredClosedOrClosing = 0;

  void setPixelColor(uint8_t r, uint8_t g, uint8_t b);
};
