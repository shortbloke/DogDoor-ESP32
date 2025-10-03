#include <Arduino.h>
#include "DoorController.h"
#include <UMS3.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Secrets.h"
#include "DisplayHelpers.h"
#include "DoorTelemetryProvider.h"
#include "mqtt.h"
#include "ConnectivityManager.h"
#include <ArduinoOTA.h>

namespace
{
constexpr int kScreenWidth = 128;     // OLED display width, in pixels
constexpr int kScreenHeight = 32;     // OLED display height, in pixels
constexpr uint8_t kScreenAddress = 0x3C; // 0x3D for 128x64, 0x3C for 128x32
constexpr uint8_t kPixelBrightness = 255 / 10;
}

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire);

UMS3 ums3;
DoorController door;
ConnectivityManager connectivity;

class DoorControllerTelemetry final : public DoorTelemetryProvider
{
public:
  explicit DoorControllerTelemetry(DoorController &controller) : door(controller) {}

  const char *getDoorStateString() const override { return door.getStateString(); }
  float getDistanceIndoorCm() const override { return door.getDistanceIndoorCm(); }
  float getDistanceOutdoorCm() const override { return door.getDistanceOutdoorCm(); }
  bool isLimitSwitchPressed(LimitSwitch sw) const override { return door.isLimitSwitchPressed(sw); }
  uint8_t getLastSensorTriggered() const override { return door.getLastSensorTriggered(); }

private:
  DoorController &door;
};

DoorControllerTelemetry doorTelemetry(door);

void setup()
{
  door.setUMS3(&ums3);               // Initialize all Unexpected Maker Pro S3 board peripherals
  door.setDisplay(&display);         // Pass the display reference to the DoorController
  ums3.begin();                      // Brightness is 0-255. We set it to 1/10 brightness here
  ums3.setPixelBrightness(kPixelBrightness); // Enable the power to the RGB LED, as Off by default so it doesn't use current when the LED is not required.
  ums3.setPixelPower(true);

  display.begin(SSD1306_SWITCHCAPVCC, kScreenAddress);
  DisplayHelpers::setWiFiConnected(false);
  DisplayHelpers::setMQTTConnected(false);
  DisplayHelpers::setLastSensorTriggered(0);
  DisplayHelpers::showStatus(&display, "Connecting to WiFi...");

  ums3.setPixelColor(255, 255, 0); // Yellow during WiFi connection

  connectivity.begin(&door, &display);

  ums3.setPixelColor(255, 0, 0); // Set the pixel color to red during hardware setup
  door.setup();

  mqttSetup(&doorTelemetry);  // MQTT client initialisation (connection handled in loop)

  // Optional: change distance publish interval (e.g. 60s)
  // mqttSetDistancePublishInterval(60000);

  ums3.setPixelColor(0, 255, 0);
}

void loop()
{
  connectivity.loop();
  ArduinoOTA.handle();
  door.loop();
  mqttLoop();
}
