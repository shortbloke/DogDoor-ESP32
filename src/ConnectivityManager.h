#pragma once

#include <Arduino.h>
#include <WiFi.h>

class DoorController;
class DisplayService;

// Coordinates WiFi connection retries and OTA initialisation without blocking loop().
class ConnectivityManager {
public:
  ConnectivityManager(DoorController &doorController, DisplayService &displayService)
      : door(doorController), display(displayService) {}

  void begin();
  void loop();
  bool isConnected() const { return wifiConnected; }

private:
  static constexpr unsigned long attemptIntervalMs = 500;
  static constexpr int maxAttemptsPerCycle = 20;
  static constexpr unsigned long retryDelayMs = 10000; // wait before retrying after a failed cycle

  void startConnectCycle();
  void handleConnected();
  void handleDisconnected();
  void handleConnectionFailure(wl_status_t status);
  void scheduleRetry();
  void showStatus(const char *message, bool warning = false, int textSize = 1);
  static const char *statusDescription(wl_status_t status);

  DoorController &door;
  DisplayService &display;

  unsigned long lastAttemptMs = 0;
  unsigned long retryStartMs = 0;
  int attemptCount = 0;
  bool wifiConnected = false;
  bool otaReady = false;
  bool waitingForRetry = false;
};
