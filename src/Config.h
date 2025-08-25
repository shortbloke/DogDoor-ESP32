#pragma once
// Config.h - Central configuration for DogDoor-ESP32 hardware and logic.

#include <array>

namespace Config
{
  // --- Limit Switches ---
  constexpr size_t numLimitSwitches = 2;
  constexpr std::array<uint8_t, numLimitSwitches> limitSwitchPins = {1, 2};

  // --- VL53L0X Time-of-Flight Sensors ---
  constexpr size_t numTOFSensors = 2;
  constexpr std::array<uint8_t, numTOFSensors> xshutPins = {7, 6};
  constexpr uint8_t SDA = 8;
  constexpr uint8_t SCL = 9;
  constexpr std::array<uint16_t, numTOFSensors> rangeThreshold = {100, 100};
  constexpr uint16_t TOFSensorTimeout = 500;
  constexpr uint32_t TOFSensorTimingBudget = 100000;
  constexpr uint8_t TOFSensorStartAddress = 0x2A;
  constexpr std::array<const char *, numTOFSensors> sensorNames = {"Indoor", "Outdoor"};
  constexpr uint16_t TOFSensorErrorValue = 65535;

  // --- Stepper Motor ---
  constexpr uint8_t dirPinStepper = 41;
  constexpr uint8_t enablePinStepper = 42;
  constexpr uint8_t stepPinStepper = 40;
  constexpr uint32_t stepsPerSecond = 200000;
  constexpr uint32_t acceleration = 100000;
  constexpr uint16_t expectedDoorClosePosition = 0;
  constexpr uint16_t seekIncrementSteps = 100;

  // --- Misc ---
  constexpr unsigned long SerialBaudRate = 115200;
  constexpr int DebounceIntervalMs = 10;
  constexpr int SensorInitDelayMs = 10;
  constexpr int SetupDelayMs = 2000;

  // --- Door Control ---
  static constexpr unsigned long doorOpenHoldMs = 10000; // 10 seconds
}

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