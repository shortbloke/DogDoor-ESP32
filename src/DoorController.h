#pragma once
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>
#include <array>
#include "Config.h"
#include "IconBitmaps.h"

// Forward declaration for UMS3
class UMS3;

// Controls the automated dog door hardware and logic.
class DoorController {
public:
  DoorController();
  ~DoorController();

  // Initialize all hardware and sensors.
  void setup();

  // Main loop: updates sensors, debounces switches, and runs state machine.
  void loop();

  void setUMS3(UMS3* ums3Ptr);

  void setDisplay(Adafruit_SSD1306* displayPtr);

  const char *getStateString() const;


#include "IconBitmaps.h"

  // Update WiFi connection status for display
  void setWiFiConnected(bool connected);

private:

  // WiFi status check timing
  unsigned long lastWiFiCheckMs = 0;
  static constexpr unsigned long wifiCheckIntervalMs = 30000; // 30 seconds
  // --- State for display icons ---
  bool wifiConnected = false;
  // MQTT support will be added later, placeholder for now
  bool mqttConnected = false;
  // 0 = none, 1 = indoor, 2 = outdoor
  uint8_t lastSensorTriggered = 0;
  // Hardware setup helpers
  void setupStepper();
  bool setupTOFSensors();
  void setupLimitSwitches();

  // Sensor and state helpers
  void updateSensorStates();
  void handleState();

  // State machine handlers
  void handleClosedState();
  void handleOpeningState();
  void handleOpenState();
  void handleClosingState();

  // Returns debounced state of the given limit switch.
  bool isLimitSwitchPressed(LimitSwitch sw);

  // Display helpers moved to DisplayHelpers namespace

  void seekLimitSwitch(int direction, int steps);

  // --- Hardware interfaces ---
  FastAccelStepperEngine engine;
  FastAccelStepper* stepper = nullptr;

  // --- Sensors and switches ---
  std::array<VL53L0X*, Config::numTOFSensors> sensors{{nullptr, nullptr}};
  std::array<uint16_t, Config::numTOFSensors> range{{0, 0}};
  std::array<Bounce, Config::numLimitSwitches> limitSwitchDebouncers;

  // --- State variables ---
  bool openDoor = false;
  DoorState state = DoorState::Closed;
  DoorState last_state = DoorState::Closed;
  uint16_t expectedDoorOpenPosition = 10000; // Updated at runtime
  bool seekingTopLimit = false;
  UMS3* ums3 = nullptr;
  Adafruit_SSD1306* display = nullptr;
  unsigned long openStateEnteredMs = 0;
  bool openStateFirstEntry = true;
  bool sensorInitNeeded = false;

  void setPixelColor(uint8_t r, uint8_t g, uint8_t b);
};