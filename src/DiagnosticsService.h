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
  DiagnosticsService() = default;

  void begin(DoorController *doorController,
             TofSensorManager *tofManager,
             ConnectivityManager *connectivityManager);

  void loop();

private:
  void handleCommand();
  void printStatus() const;

  static constexpr size_t kInputBufferSize = 64;

  DoorController *door = nullptr;
  TofSensorManager *tof = nullptr;
  ConnectivityManager *connectivity = nullptr;

  char inputBuffer[kInputBufferSize] = {0};
  size_t inputLength = 0;
};
