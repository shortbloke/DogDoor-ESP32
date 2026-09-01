#pragma once

#include <cstdint>

// Tracks whether one distance sensor has produced enough consecutive readings
// below its threshold to be considered occupied.
class SensorQualification
{
public:
  struct Result
  {
    bool qualified;
    bool triggered;
  };

  explicit constexpr SensorQualification(uint8_t readingsRequired)
      : required(readingsRequired == 0 ? 1 : readingsRequired) {}

  constexpr Result update(bool valid, uint16_t readingMm,
                          uint16_t thresholdMm, uint16_t maximumDistanceMm)
  {
    const bool belowThreshold = valid && readingMm <= maximumDistanceMm &&
                                readingMm < thresholdMm;
    if (!belowThreshold)
    {
      reset();
      return {false, false};
    }

    if (streak < required)
    {
      ++streak;
    }

    const bool triggered = streak >= required && !isQualified;
    if (triggered)
    {
      isQualified = true;
    }
    return {isQualified, triggered};
  }

  constexpr void reset()
  {
    streak = 0;
    isQualified = false;
  }

  constexpr bool qualified() const { return isQualified; }
  constexpr uint8_t consecutiveReadings() const { return streak; }

private:
  uint8_t required;
  uint8_t streak = 0;
  bool isQualified = false;
};
