#pragma once
// Config.h - Central configuration for DogDoor-ESP32 hardware and logic.

#include <array>
#include <cstdint>

struct LimitSwitchConfig
{
  static constexpr size_t count = 2;
  std::array<uint8_t, count> pins{1, 2};
  bool activeHigh = true;
  int debounceMs = 2;
};

struct OverrideSwitchConfig
{
  uint8_t keepOpenPin = 3;
  uint8_t keepClosedPin = 4;
};

struct TofSensorConfig
{
  static constexpr size_t count = 2;
  std::array<uint8_t, count> xshutPins{7, 6};
  uint8_t sdaPin = 8;
  uint8_t sclPin = 9;
  std::array<uint16_t, count> rangeThresholdMm{600, 300};
  uint16_t timeoutMs = 500;
  uint32_t timingBudgetUs = 80000; // microseconds
  uint8_t startAddress = 0x2A;
  std::array<const char *, count> names{"Indoor", "Outdoor"};
  uint16_t errorValue = 65535;
  uint16_t maxConsideredDistanceMm = 1000;
  uint8_t consecutiveReadingsForTrigger = 4;
  bool enableDebugDisplay = false;
};

struct StepperConfig
{
  uint8_t dirPin = 41;
  uint8_t enablePin = 42;
  uint8_t stepPin = 40;
  bool enableActiveLow = true;
  int32_t stepsPerSecond = 17500;
  int32_t acceleration = 17500;
  int32_t expectedClosePosition = 0;
  int32_t seekIncrementSteps = 10;
};

struct TimingConfig
{
  unsigned long doorOpenHoldMs = 5000;
  unsigned long sensorReinitIntervalMs = 5UL * 60UL * 1000UL;
  unsigned long sensorStatusGracePeriodMs = 250;
  int sensorInitDelayMs = 10;
  int setupDelayMs = 2000;
};

struct MiscConfig
{
  unsigned long serialBaudRate = 115200;
};

struct ConfigData
{
  LimitSwitchConfig limitSwitches{};
  OverrideSwitchConfig overrides{};
  TofSensorConfig tof{};
  StepperConfig stepper{};
  TimingConfig timing{};
  MiscConfig misc{};
};

extern const ConfigData Config;

// --- Door State Enum ---
enum class DoorState
{
  Closed,
  Opening,
  Open,
  Closing
};

// --- Limit Switch Enum ---
enum LimitSwitch
{
  BottomLimitSwitch = 0,
  TopLimitSwitch = 1
};

// --- VL53L0X Sensor Enum ---
enum TOFSensor
{
  IndoorSensor = 0,
  OutdoorSensor = 1
};
