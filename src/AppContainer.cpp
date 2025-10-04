#include "AppContainer.h"
#include "DisplayHelpers.h"
#include "mqtt.h"
#include <ArduinoOTA.h>
#include <Wire.h>

AppContainer::AppContainer()
    : display(kScreenWidth, kScreenHeight, &Wire),
      tofSensors(&displayService),
      telemetry(door)
{
}

void AppContainer::begin()
{
  door.setUMS3(&ums3);
  door.setDisplayService(&displayService);
  door.setTofSensorManager(&tofSensors);

  ums3.begin();
  ums3.setPixelBrightness(kPixelBrightness);
  ums3.setPixelPower(true);

  display.begin(SSD1306_SWITCHCAPVCC, kScreenAddress);
  displayService.attach(&display);
  DisplayHelpers::setService(&displayService);
  displayService.setWiFiConnected(false);
  displayService.setMQTTConnected(false);
  displayService.setLastSensorTriggered(0);
  displayService.showStatus("Connecting to WiFi...");

  ums3.setPixelColor(255, 255, 0); // Yellow during WiFi connection
  connectivity.begin(&door, &display);

  ums3.setPixelColor(255, 0, 0);
  door.setup();

  mqttSetup(&telemetry);  // MQTT client initialisation (connection handled in loop)

  ums3.setPixelColor(0, 255, 0);
}

void AppContainer::loop()
{
  connectivity.loop();
  ArduinoOTA.handle();
  door.loop();
  mqttLoop();
}
