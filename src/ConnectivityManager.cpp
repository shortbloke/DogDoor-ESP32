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

  wl_status_t status = WiFi.status();

  if (wifiConnected)
  {
    if (status != WL_CONNECTED)
    {
      handleDisconnected();
    }
    return;
  }

  // Not connected yet, check for success or retry window expiry.
  if (status == WL_CONNECTED)
  {
    handleConnected();
    return;
  }

  switch (status)
  {
    case WL_NO_SSID_AVAIL:
    case WL_CONNECT_FAILED:
#ifdef WL_WRONG_PASSWORD
    case WL_WRONG_PASSWORD:
#endif
#ifdef WL_CONNECTION_LOST
    case WL_CONNECTION_LOST:
#endif
      handleConnectionFailure(status);
      return;
    default:
      break;
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
  if (door)
  {
    door->refreshStateDisplay();
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

void ConnectivityManager::handleConnectionFailure(wl_status_t status)
{
  wifiConnected = false;
  const char *reason = statusDescription(status);
  showStatus(reason, true);
  Serial.printf("WARN: WiFi connection failure: %s (code=%d)\n", reason, static_cast<int>(status));
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

const char *ConnectivityManager::statusDescription(wl_status_t status)
{
  switch (status)
  {
    case WL_IDLE_STATUS:
      return "WiFi idle";
    case WL_NO_SSID_AVAIL:
      return "WiFi SSID not found";
    case WL_SCAN_COMPLETED:
      return "WiFi scan complete";
    case WL_CONNECTED:
      return "WiFi connected";
    case WL_CONNECT_FAILED:
      return "WiFi auth failed";
#ifdef WL_WRONG_PASSWORD
    case WL_WRONG_PASSWORD:
      return "WiFi wrong password";
#endif
#ifdef WL_CONNECTION_LOST
    case WL_CONNECTION_LOST:
      return "WiFi connection lost";
#endif
    case WL_DISCONNECTED:
      return "WiFi disconnected";
#ifdef WL_NO_SHIELD
    case WL_NO_SHIELD:
      return "WiFi shield missing";
#endif
    default:
      break;
  }
  return "WiFi error";
}
