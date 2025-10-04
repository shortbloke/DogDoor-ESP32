#pragma once

#include <Adafruit_SSD1306.h>
#include <cstddef>
#include <cstdint>

// Provides status and message rendering for the OLED display without relying on
// module-level globals. Instances can be injected into components that need to
// update UI state, making the display easier to mock in tests.
class DisplayService {
public:
  DisplayService() = default;

  void attach(Adafruit_SSD1306 *displayPtr);
  Adafruit_SSD1306 *display() const { return oled; }

  void setWiFiConnected(bool connected);
  void setMQTTConnected(bool connected);
  void setLastSensorTriggered(uint8_t sensorId);

  void showStatus(const char *msg, bool isWarning = false, int textSize = 1);
  void refresh();

private:
  void render(const char *msg, bool isWarning, int textSize);
  void drawStatusIcons();

  static constexpr size_t kMaxMessageLength = 64;

  Adafruit_SSD1306 *oled = nullptr;
  bool wifiConnected = false;
  bool mqttConnected = false;
  uint8_t lastSensorTriggered = 0;
  bool lastMessageValid = false;
  bool lastMessageWasWarning = false;
  int lastMessageTextSize = 1;
  char lastMessage[kMaxMessageLength] = {0};
};
