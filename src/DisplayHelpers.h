#pragma once
#include <Adafruit_SSD1306.h>
#include "IconBitmaps.h"

class DisplayHelpers {
public:
    static void showStatus(Adafruit_SSD1306* display, const char* msg, bool isWarning = false, int textSize = 1);
    static void drawStatusIcons(Adafruit_SSD1306* display);
    static void setWiFiConnected(bool connected);
    static void setMQTTConnected(bool connected);
    static void setLastSensorTriggered(int sensor);
private:
    static bool wifiConnected;
    static bool mqttConnected;
    static int lastSensorTriggered;
};
