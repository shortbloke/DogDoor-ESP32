#pragma once

#include "Config.h"

// Provides read-only access to door telemetry for consumers like MQTT.
class DoorTelemetryProvider {
public:
  virtual ~DoorTelemetryProvider() = default;
  virtual DoorState getDoorState() const = 0;
  virtual const char *getDoorStateString() const = 0;
  virtual float getDistanceIndoorCm() const = 0;
  virtual float getDistanceOutdoorCm() const = 0;
  virtual bool isLimitSwitchPressed(LimitSwitch sw) = 0;
  virtual uint8_t getLastSensorTriggered() const = 0;
  virtual uint32_t getDoorOpenCount() const = 0;
  virtual uint32_t getSensorTriggerCount(size_t index) const = 0;
  // Current remote override mode, as applied by the controller. May change
  // without an MQTT command (e.g. the physical toggle returning to centre resets
  // it to Auto), so the network layer polls this to keep the HA select in sync.
  virtual OverrideMode getOverrideMode() const = 0;
};
