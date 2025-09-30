#include "DisplayHelpers.h"
#include <WiFi.h>
#include <cstring>

// Static member definitions
bool DisplayHelpers::wifiConnected = false;
bool DisplayHelpers::mqttConnected = false;
int DisplayHelpers::lastSensorTriggered = 0;
Adafruit_SSD1306 *DisplayHelpers::activeDisplay = nullptr;
bool DisplayHelpers::lastMessageValid = false;
bool DisplayHelpers::lastMessageWasWarning = false;
int DisplayHelpers::lastMessageTextSize = 1;
char DisplayHelpers::lastMessage[DisplayHelpers::maxMessageLength] = {0};

void DisplayHelpers::setWiFiConnected(bool connected)
{
    if (wifiConnected == connected)
        return;
    wifiConnected = connected;
    refreshStatus();
}
void DisplayHelpers::setMQTTConnected(bool connected)
{
    if (mqttConnected == connected)
        return;
    mqttConnected = connected;
    refreshStatus();
}
void DisplayHelpers::setLastSensorTriggered(int sensor)
{
    if (lastSensorTriggered == sensor)
        return;
    lastSensorTriggered = sensor;
    refreshStatus();
}

void DisplayHelpers::showStatus(Adafruit_SSD1306 *display, const char *msg, bool isWarning, int textSize)
{
    if (!display)
        return;
    activeDisplay = display;
    lastMessageValid = (msg != nullptr);
    if (lastMessageValid)
    {
        std::strncpy(lastMessage, msg, maxMessageLength - 1);
        lastMessage[maxMessageLength - 1] = '\0';
    }
    else
    {
        lastMessage[0] = '\0';
    }
    lastMessageWasWarning = isWarning;
    lastMessageTextSize = textSize;
    renderStatus(display, lastMessageValid ? lastMessage : nullptr, isWarning, textSize);
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
    else if (lastSensorTriggered == 2)
    {
        display->drawBitmap(iconX, iconY, IconBitmaps::out_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
    }
    iconX += IconBitmaps::iconWidth + 2;
    iconX += IconBitmaps::iconWidth + 2;
    if (wifiConnected)
    {
        // print the ip address
        display->setTextSize(1);
        display->setTextColor(SSD1306_WHITE);
        display->setCursor(iconX, iconY);
        display->print(WiFi.localIP());
    }
    display->display();
}

void DisplayHelpers::refreshStatus()
{
    if (!activeDisplay)
        return;
    renderStatus(activeDisplay, lastMessageValid ? lastMessage : nullptr, lastMessageWasWarning, lastMessageTextSize);
}

void DisplayHelpers::renderStatus(Adafruit_SSD1306 *display, const char *msg, bool isWarning, int textSize)
{
    drawStatusIcons(display);
    display->setTextSize(textSize);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 12);
    if (isWarning)
    {
        display->drawBitmap(0, 12, IconBitmaps::large_warning_icon, 16, 16, SSD1306_WHITE);
        display->setCursor(18, 12);
    }
    if (msg && msg[0] != '\0')
    {
        display->print(msg);
    }
    display->display();
}
