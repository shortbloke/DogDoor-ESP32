#include "ConnectivityManager.h"
#include <ArduinoOTA.h>
#include "DoorController.h"
#include "DisplayHelpers.h"
#include "Secrets.h"

void ConnectivityManager::begin(DoorController *doorController, Adafruit_SSD1306 *displayPtr)
{
  door = doorController;
  display = displayPtr;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  startConnectCycle();
}

void ConnectivityManager::loop()
{
  if (waitingForRetry)
  {
    if (millis() - retryStartMs >= retryDelayMs)
    {
      waitingForRetry = false;
      startConnectCycle();
    }
    return;
  }

  if (wifiConnected)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      handleDisconnected();
    }
    return;
  }

  // Not connected yet, check for success or retry window expiry.
  if (WiFi.status() == WL_CONNECTED)
  {
    handleConnected();
    return;
  }

  unsigned long now = millis();
  if (now - lastAttemptMs >= attemptIntervalMs)
  {
    lastAttemptMs = now;
    attemptCount++;
    if (attemptCount >= maxAttemptsPerCycle)
    {
      showStatus("WiFi timeout", true);
      Serial.println("WARN: WiFi connection timed out");
      scheduleRetry();
    }
  }
}

void ConnectivityManager::startConnectCycle()
{
  attemptCount = 0;
  lastAttemptMs = millis();
  showStatus("Connecting to WiFi...");
  Serial.println("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (door)
  {
    door->setWiFiConnected(false);
  }
}

void ConnectivityManager::handleConnected()
{
  wifiConnected = true;
  showStatus("WiFi connected");
  Serial.println("WiFi connected.");
  Serial.print("IP address : ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address : ");
  Serial.println(WiFi.macAddress());
  if (door)
  {
    door->setWiFiConnected(true);
  }
  if (!otaReady)
  {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    showStatus("OTA Ready");
    otaReady = true;
  }
}

void ConnectivityManager::handleDisconnected()
{
  wifiConnected = false;
  showStatus("WiFi lost", true);
  Serial.println("WARN: WiFi disconnected");
  if (door)
  {
    door->setWiFiConnected(false);
  }
  scheduleRetry();
}

void ConnectivityManager::scheduleRetry()
{
  WiFi.disconnect();
  waitingForRetry = true;
  retryStartMs = millis();
}

void ConnectivityManager::showStatus(const char *message, bool warning, int textSize)
{
  DisplayHelpers::showStatus(display, message, warning, textSize);
}
