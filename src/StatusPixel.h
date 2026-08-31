#pragma once

#include <cstdint>

// Controls the ProS3 onboard RGB status pixel using the current Arduino API.
class StatusPixel
{
public:
  explicit StatusPixel(uint8_t brightnessValue) : brightness(brightnessValue) {}

  void begin() const;
  void setColor(uint8_t red, uint8_t green, uint8_t blue) const;

private:
  const uint8_t brightness;
};
