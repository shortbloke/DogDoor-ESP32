#include "StatusPixel.h"

#include <Arduino.h>

void StatusPixel::begin() const
{
  pinMode(RGB_PWR, OUTPUT);
  digitalWrite(RGB_PWR, HIGH);
  setColor(0, 0, 0);
}

void StatusPixel::setColor(uint8_t red, uint8_t green, uint8_t blue) const
{
  const auto scale = [this](uint8_t channel) {
    return static_cast<uint8_t>((static_cast<uint16_t>(channel) * brightness) / 255U);
  };
  rgbLedWrite(RGB_BUILTIN, scale(red), scale(green), scale(blue));
}
