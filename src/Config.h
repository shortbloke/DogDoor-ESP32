#pragma once
// Config.h - Central configuration for DogDoor-ESP32 hardware and logic.

#include <array>

namespace Config
{
  // --- Limit Switches ---
  constexpr size_t numLimitSwitches = 2;
  constexpr std::array<uint8_t, numLimitSwitches> limitSwitchPins = {1, 2};

  // --- GPIO pins for override switches ---
  constexpr uint8_t overrideKeepOpenSwitchPin = 3;
  constexpr uint8_t overrideKeepClosedSwitchPin = 4;

  // --- VL53L0X Time-of-Flight Sensors ---
  constexpr size_t numTOFSensors = 2;
  constexpr std::array<uint8_t, numTOFSensors> xshutPins = {7, 6};
  constexpr uint8_t SDA = 8;
  constexpr uint8_t SCL = 9;
  constexpr std::array<uint16_t, numTOFSensors> rangeThreshold = {600, 300};
  constexpr uint16_t TOFSensorTimeout = 500;
  constexpr uint32_t TOFSensorTimeMeasurementBudget = 80000; // Values in micro seconds 80ms
  constexpr uint8_t TOFSensorStartAddress = 0x2A;
  constexpr std::array<const char *, numTOFSensors> sensorNames = {"Indoor", "Outdoor"};
  constexpr uint16_t TOFSensorErrorValue = 65535;

  // --- Stepper Motor ---
  constexpr uint8_t dirPinStepper = 41;
  constexpr uint8_t enablePinStepper = 42;
  constexpr uint8_t stepPinStepper = 40;
  constexpr bool stepperEnableActiveLow = true;
  constexpr bool limitSwitchActiveHigh = true;
  constexpr int32_t stepsPerSecond = 17500;
  constexpr int32_t acceleration = 17500;
  constexpr int32_t expectedDoorClosePosition = 0;
  constexpr int32_t seekIncrementSteps = 10;

  // --- Misc ---
  constexpr unsigned long SerialBaudRate = 115200;
  constexpr int LimitSwitchDebounceMs = 2;
  constexpr int SensorInitDelayMs = 10;
  constexpr int SetupDelayMs = 2000;

  // --- Door Control ---
  static constexpr unsigned long doorOpenHoldMs = 5000; // 5 seconds
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
