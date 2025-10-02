#pragma once
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>
#include <array>
#include <cstdint>
#include "Config.h"

// Forward declaration for UMS3
class UMS3;

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

  void setDisplay(Adafruit_SSD1306 *displayPtr);

  const char *getStateString() const;
  float getDistanceIndoorCm() const;
  float getDistanceOutdoorCm() const;

  void refreshStateDisplay();

  // Returns debounced state of the given limit switch.
  bool isLimitSwitchPressed(LimitSwitch sw);

  uint8_t getLastSensorTriggered() const { return lastSensorTriggered; }

  // Update WiFi connection status for display
  void setWiFiConnected(bool connected);

private:
  // WiFi status check timing
  unsigned long lastWiFiCheckMs = 0;
  static constexpr unsigned long wifiCheckIntervalMs = 30000; // 30 seconds
  // Periodic VL53L0X re-initialization timing
  unsigned long lastSensorInitMs = 0;
  static constexpr unsigned long sensorReinitIntervalMs = 5UL * 60UL * 1000UL; // 5 minutes
  // --- State for display icons ---
  bool wifiConnected = false;
  // 0 = none, 1 = indoor, 2 = outdoor
  uint8_t lastSensorTriggered = 0;
  // Hardware setup helpers
  void setupStepper();
  bool setupTOFSensors();
  void setupLimitSwitches();

  // Sensor and state helpers
  void updateSensorStates(bool allowDoorTrigger);
  void checkOverrideSwitches();
  void handleState();
  void showStateOnDisplay();

  // State machine handlers
  void handleClosedState();
  void handleOpeningState();
  void handleOpenState();
  void handleClosingState();

  // Display helpers moved to DisplayHelpers namespace

  void seekLimitSwitch(int direction, int steps);

  // --- Hardware interfaces ---
  FastAccelStepperEngine engine;
  FastAccelStepper *stepper = nullptr;

  // --- Sensors and switches ---
  std::array<VL53L0X, Config::numTOFSensors> sensors;
  std::array<bool, Config::numTOFSensors> sensorReady{{false, false}};
  std::array<bool, Config::numTOFSensors> sensorBelowThreshold{{false, false}};
  std::array<uint16_t, Config::numTOFSensors> range{{0, 0}};
  std::array<Bounce, Config::numLimitSwitches> limitSwitchDebouncers;

  // --- State variables ---
  bool openDoor = false;
  bool keepClosed = false;
  bool keepOpen = false;
  bool fastClosing = false; 
  DoorState state = DoorState::Closed;
  DoorState last_state = DoorState::Closed;
  int32_t expectedDoorOpenPosition = 100000; // Updated at runtime based on limit switches
  UMS3 *ums3 = nullptr;
  Adafruit_SSD1306 *display = nullptr;
  unsigned long openStateEnteredMs = 0;
  bool openStateFirstEntry = true;
  bool sensorInitNeeded = false;
  std::array<int8_t, Config::numLimitSwitches> lastLimitSwitchState{{-1, -1}};

  void setPixelColor(uint8_t r, uint8_t g, uint8_t b);
};
