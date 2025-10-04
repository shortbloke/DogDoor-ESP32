#include "DisplayService.h"
#include "IconBitmaps.h"
#include <WiFi.h>
#include <cstring>

void DisplayService::attach(Adafruit_SSD1306 *displayPtr)
{
  if (oled == displayPtr)
  {
    return;
  }
  oled = displayPtr;
  refresh();
}

void DisplayService::setWiFiConnected(bool connected)
{
  if (wifiConnected == connected)
  {
    return;
  }
  wifiConnected = connected;
  refresh();
}

void DisplayService::setMQTTConnected(bool connected)
{
  if (mqttConnected == connected)
  {
    return;
  }
  mqttConnected = connected;
  refresh();
}

void DisplayService::setLastSensorTriggered(uint8_t sensorId)
{
  if (lastSensorTriggered == sensorId)
  {
    return;
  }
  lastSensorTriggered = sensorId;
  refresh();
}

void DisplayService::showStatus(const char *msg, bool isWarning, int textSize)
{
  if (!oled)
  {
    return;
  }

  const bool messageValid = (msg != nullptr) && (msg[0] != '\0');
  bool messageChanged = (lastMessageValid != messageValid);
  if (!messageChanged && messageValid)
  {
    messageChanged = std::strncmp(lastMessage, msg, kMaxMessageLength - 1) != 0;
  }
  const bool warningChanged = (lastMessageWasWarning != isWarning);
  const bool textSizeChanged = (lastMessageTextSize != textSize);

  lastMessageValid = messageValid;
  if (messageValid)
  {
    std::strncpy(lastMessage, msg, kMaxMessageLength - 1);
    lastMessage[kMaxMessageLength - 1] = '\0';
  }
  else
  {
    lastMessage[0] = '\0';
  }
  lastMessageWasWarning = isWarning;
  lastMessageTextSize = textSize;

  if (!messageChanged && !warningChanged && !textSizeChanged)
  {
    return;
  }

  render(lastMessageValid ? lastMessage : nullptr, isWarning, textSize);
}

void DisplayService::refresh()
{
  if (!oled)
  {
    return;
  }
  render(lastMessageValid ? lastMessage : nullptr, lastMessageWasWarning, lastMessageTextSize);
}

void DisplayService::render(const char *msg, bool isWarning, int textSize)
{
  drawStatusIcons();
  oled->setTextSize(textSize);
  oled->setTextColor(SSD1306_WHITE);
  oled->setCursor(0, 12);
  if (isWarning)
  {
    oled->drawBitmap(0, 12, IconBitmaps::large_warning_icon, 16, 16, SSD1306_WHITE);
    oled->setCursor(18, 12);
  }
  if (msg && msg[0] != '\0')
  {
    oled->print(msg);
  }
  oled->display();
}

void DisplayService::drawStatusIcons()
{
  if (!oled)
  {
    return;
  }

  oled->clearDisplay();
  int iconX = 0;
  int iconY = 0;
  if (wifiConnected)
  {
    oled->drawBitmap(iconX, iconY, IconBitmaps::wifi_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
  }
  else
  {
    oled->drawBitmap(iconX, iconY, IconBitmaps::nowifi_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
  }
  iconX += IconBitmaps::iconWidth + 2;
  if (mqttConnected)
  {
    oled->drawBitmap(iconX, iconY, IconBitmaps::mqtt_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
  }
  iconX += IconBitmaps::iconWidth + 2;
  if (lastSensorTriggered == 1)
  {
    oled->drawBitmap(iconX, iconY, IconBitmaps::in_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
  }
  else if (lastSensorTriggered == 2)
  {
    oled->drawBitmap(iconX, iconY, IconBitmaps::out_icon, IconBitmaps::iconWidth, IconBitmaps::iconHeight, SSD1306_WHITE);
  }
  iconX += IconBitmaps::iconWidth + 2;
  if (wifiConnected)
  {
    oled->setTextSize(1);
    oled->setTextColor(SSD1306_WHITE);
    oled->setCursor(iconX, iconY);
    oled->print(WiFi.localIP());
  }
}
