#pragma once

#include "Config.h"

// Receives remote (MQTT) override commands. Implemented by the door controller
// so the network layer can request a mode without reaching into motion state.
// The receiver only records the request; it is applied inside the controller's
// own loop, where the physical override switches keep precedence.
class DoorCommandSink {
public:
  virtual ~DoorCommandSink() = default;
  virtual void setOverrideCommand(OverrideMode mode) = 0;
};
