#include "DisplayHelpers.h"
#include "DisplayService.h"

namespace {
DisplayService *gDisplayService = nullptr;
}

void DisplayHelpers::setService(DisplayService *service)
{
    gDisplayService = service;
}

void DisplayHelpers::showStatus(Adafruit_SSD1306 *display, const char *msg, bool isWarning, int textSize)
{
    if (!gDisplayService)
    {
        return;
    }
    if (display)
    {
        gDisplayService->attach(display);
    }
    gDisplayService->showStatus(msg, isWarning, textSize);
}

void DisplayHelpers::drawStatusIcons(Adafruit_SSD1306 *display)
{
    if (!gDisplayService)
    {
        return;
    }
    if (display)
    {
        gDisplayService->attach(display);
    }
    gDisplayService->refresh();
}

void DisplayHelpers::refreshStatus()
{
    if (!gDisplayService)
    {
        return;
    }
    gDisplayService->refresh();
}

void DisplayHelpers::setWiFiConnected(bool connected)
{
    if (!gDisplayService)
    {
        return;
    }
    gDisplayService->setWiFiConnected(connected);
}

void DisplayHelpers::setMQTTConnected(bool connected)
{
    if (!gDisplayService)
    {
        return;
    }
    gDisplayService->setMQTTConnected(connected);
}

void DisplayHelpers::setLastSensorTriggered(uint8_t sensorId)
{
    if (!gDisplayService)
    {
        return;
    }
    gDisplayService->setLastSensorTriggered(sensorId);
}
