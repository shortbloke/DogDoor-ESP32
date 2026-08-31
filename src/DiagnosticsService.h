#pragma once

#include <Arduino.h>
#include <cstddef>

class DoorController;
class TofSensorManager;
class ConnectivityManager;

// Provides a simple serial diagnostics channel. Sending "diag" or "d" followed by
// newline over the USB serial port prints current state, sensor health, and last
// trigger information.
class DiagnosticsService {
public:
  DiagnosticsService(DoorController &doorController,
                     TofSensorManager &tofManager,
                     ConnectivityManager &connectivityManager)
      : door(doorController), tof(tofManager), connectivity(connectivityManager) {}

  void loop();

private:
  void handleCommand();
  void printStatus() const;

  static constexpr size_t kInputBufferSize = 64;

  DoorController &door;
  TofSensorManager &tof;
  ConnectivityManager &connectivity;

  char inputBuffer[kInputBufferSize] = {0};
  size_t inputLength = 0;
};
