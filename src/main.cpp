#include <Arduino.h>
#include "DoorController.h"
#include <UMS3.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Secrets.h"
#include "DisplayHelpers.h"

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

UMS3 ums3;
DoorController door;

void setup()
{
  door.setUMS3(&ums3);               // Initialize all Unexpected Maker Pro S3 board peripherals
  door.setDisplay(&display);         // Pass the display reference to the DoorController
  ums3.begin();                      // Brightness is 0-255. We set it to 1/10 brightness here
  ums3.setPixelBrightness(255 / 10); // Enable the power to the RGB LED, as Off by default so it doesn't use current when the LED is not required.
  ums3.setPixelPower(true);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  DisplayHelpers::setWiFiConnected(false);
  DisplayHelpers::setMQTTConnected(false);
  DisplayHelpers::setLastSensorTriggered(0);
  DisplayHelpers::showStatus(&display, "Connecting to WiFi...");

  ums3.setPixelColor(255, 255, 0); // Yellow during WiFi connection

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Connect to WiFi
  int wifi_attempts = 0;
  bool wifi_connected = false;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20)
  {
    delay(500);
    wifi_attempts++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    wifi_connected = true;
    DisplayHelpers::setWiFiConnected(true);
    char buf[64];
    snprintf(buf, sizeof(buf), "Connected to: %s", WIFI_SSID);
    DisplayHelpers::showStatus(&display, buf);
    Serial.println("WiFi connected.");
    Serial.print("IP address : ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC address : ");
    Serial.println(WiFi.macAddress());
  }
  else
  {
    DisplayHelpers::showStatus(&display, "Failed to connect", true);
    Serial.println("WARN: Failed to connect to WiFi");
  }

  // OTA setup
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  DisplayHelpers::showStatus(&display, "OTA Ready");

  ums3.setPixelColor(255, 0, 0); // Set the pixel color to red

  door.setWiFiConnected(wifi_connected); // Update WiFi icon status for display
  door.setup();                          // Setup Door Controller

  ums3.setPixelColor(0, 255, 0); // Set the pixel to Green at end of Setup
}

void loop()
{
  ArduinoOTA.handle(); // Handle OTA updates
  door.loop();
}