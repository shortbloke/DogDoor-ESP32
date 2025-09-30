#pragma once

#include <Arduino.h>
#include <WiFi.h>

class DoorController;
class Adafruit_SSD1306;

// Coordinates WiFi connection retries and OTA initialisation without blocking loop().
class ConnectivityManager {
public:
  void begin(DoorController *doorController, Adafruit_SSD1306 *displayPtr);
  void loop();
  bool isConnected() const { return wifiConnected; }

private:
  static constexpr unsigned long attemptIntervalMs = 500;
  static constexpr int maxAttemptsPerCycle = 20;
  static constexpr unsigned long retryDelayMs = 10000; // wait before retrying after a failed cycle

  void startConnectCycle();
  void handleConnected();
  void handleDisconnected();
  void scheduleRetry();
  void showStatus(const char *message, bool warning = false, int textSize = 1);

  DoorController *door = nullptr;
  Adafruit_SSD1306 *display = nullptr;

  unsigned long lastAttemptMs = 0;
  unsigned long retryStartMs = 0;
  int attemptCount = 0;
  bool wifiConnected = false;
  bool otaReady = false;
  bool waitingForRetry = false;
};
