#include "DisplayHelpers.h"

// Static member definitions
bool DisplayHelpers::wifiConnected = false;
bool DisplayHelpers::mqttConnected = false;
int DisplayHelpers::lastSensorTriggered = 0;

void DisplayHelpers::setWiFiConnected(bool connected)
{
    wifiConnected = connected;
}
void DisplayHelpers::setMQTTConnected(bool connected)
{
    mqttConnected = connected;
}
void DisplayHelpers::setLastSensorTriggered(int sensor)
{
    lastSensorTriggered = sensor;
}

void DisplayHelpers::showStatus(Adafruit_SSD1306 *display, const char *msg, bool isWarning, int textSize)
{
    if (!display)
        return;
    Serial.printf("showstatus called with: %s\n", msg);
    drawStatusIcons(display);
    display->setTextSize(textSize);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 12); // Set cursor for message
    if (isWarning)
    {
        display->drawBitmap(0, 12, IconBitmaps::large_warning_icon, 16, 16, SSD1306_WHITE);
        display->setCursor(18, 12); // Move cursor right to avoid overlapping icon
    }
    display->print(msg);
    display->display();
}

void DisplayHelpers::drawStatusIcons(Adafruit_SSD1306 *display)
{
    if (!display)
        return;
    display->clearDisplay();
    int iconX = 0;
    int iconY = 0;
    if (wifiConnected)
    {
        display->drawBitmap(iconX, iconY, IconBitmaps::wifi_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
    }
    else
    {
        display->drawBitmap(iconX, iconY, IconBitmaps::nowifi_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
    }
    iconX += IconBitmaps::iconWidth + 2;
    if (mqttConnected)
    {
        display->drawBitmap(iconX, iconY, IconBitmaps::mqtt_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
    }
    iconX += IconBitmaps::iconWidth + 2;
    if (lastSensorTriggered == 1)
    {
        display->drawBitmap(iconX, iconY, IconBitmaps::in_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
    }
    else
    {
        display->drawBitmap(iconX, iconY, IconBitmaps::out_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
    }
    display->display();
}
