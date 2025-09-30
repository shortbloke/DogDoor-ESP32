#pragma once

#include "Config.h"

// Provides read-only access to door telemetry for consumers like MQTT.
class DoorTelemetryProvider {
public:
  virtual ~DoorTelemetryProvider() = default;
  virtual const char *getDoorStateString() const = 0;
  virtual float getDistanceIndoorCm() const = 0;
  virtual float getDistanceOutdoorCm() const = 0;
  virtual bool isLimitSwitchPressed(LimitSwitch sw) const = 0;
  virtual uint8_t getLastSensorTriggered() const = 0;
};
