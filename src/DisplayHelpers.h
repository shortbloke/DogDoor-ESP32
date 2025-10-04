#pragma once
#include <Adafruit_SSD1306.h>
#include <cstdint>

class DisplayService;

class DisplayHelpers
{
public:
    static void setService(DisplayService *service);

    static void showStatus(Adafruit_SSD1306 *display, const char *msg, bool isWarning = false, int textSize = 1);
    static void drawStatusIcons(Adafruit_SSD1306 *display);
    static void refreshStatus();
    static void setWiFiConnected(bool connected);
    static void setMQTTConnected(bool connected);
    static void setLastSensorTriggered(uint8_t sensorId);

private:
    static DisplayService *service;
};
