#pragma once

#include <cstdint>

// uint32_t arithmetic deliberately preserves Arduino millis() wraparound.
constexpr bool durationElapsed(uint32_t now, uint32_t started, uint32_t duration)
{
  return static_cast<uint32_t>(now - started) >= duration;
}

constexpr bool motionBoundsExceeded(int32_t currentPosition, int32_t startedPosition,
                                    int32_t maximumTravel, uint32_t now,
                                    uint32_t startedAt, uint32_t timeout)
{
  const int64_t travelled = static_cast<int64_t>(currentPosition) - startedPosition;
  const uint64_t distance = travelled < 0 ? static_cast<uint64_t>(-travelled)
                                          : static_cast<uint64_t>(travelled);
  return distance >= static_cast<uint64_t>(maximumTravel) ||
         durationElapsed(now, startedAt, timeout);
}
