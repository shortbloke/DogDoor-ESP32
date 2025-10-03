#pragma once
#include <Adafruit_SSD1306.h>
#include <cstddef>
#include <cstdint>
#include "IconBitmaps.h"

class DisplayHelpers
{
public:
    static void showStatus(Adafruit_SSD1306 *display, const char *msg, bool isWarning = false, int textSize = 1);
    static void drawStatusIcons(Adafruit_SSD1306 *display);
    static void refreshStatus();
    static void setWiFiConnected(bool connected);
    static void setMQTTConnected(bool connected);
    static void setLastSensorTriggered(uint8_t sensorId);

private:
    static bool wifiConnected;
    static bool mqttConnected;
    static uint8_t lastSensorTriggered;
    static Adafruit_SSD1306 *activeDisplay;
    static bool lastMessageValid;
    static bool lastMessageWasWarning;
    static int lastMessageTextSize;
    static constexpr size_t maxMessageLength = 64;
    static char lastMessage[maxMessageLength];

    static void renderStatus(Adafruit_SSD1306 *display, const char *msg, bool isWarning, int textSize);
};
