#include "AppContainer.h"
#include "mqtt.h"
#include <ArduinoOTA.h>
#include <Wire.h>

AppContainer::AppContainer()
    : display(kScreenWidth, kScreenHeight, &Wire),
      tofSensors(displayService),
      door(statusPixel, displayService, tofSensors),
      connectivity(door, displayService),
      diagnostics(door, tofSensors, connectivity)
{
}

void AppContainer::begin()
{
  // This must remain the first hardware action: hold the door against gravity
  // before display, sensor, Wi-Fi, MQTT, or OTA initialization can delay it.
  door.beginMotorHold();

  statusPixel.begin();

  if (display.begin(SSD1306_SWITCHCAPVCC, kScreenAddress))
  {
    displayService.attach(&display);
    displayService.setWiFiConnected(false);
    displayService.setMQTTConnected(false);
    displayService.setLastSensorTriggered(0);
    displayService.showStatus("Connecting to WiFi...");
  }
  else
  {
    Serial.println("OLED initialization failed; continuing without display");
  }

  statusPixel.setColor(255, 255, 0); // Yellow during WiFi connection
  connectivity.begin();

  statusPixel.setColor(255, 0, 0);
  door.setup();

  mqttSetup(&door, &door, &displayService);  // connection handled in loop

  statusPixel.setColor(0, 255, 0);

}

void AppContainer::loop()
{
  // Door motion and raw limit switches take priority over network services.
  door.loop();
  connectivity.loop();
  ArduinoOTA.handle();
  mqttLoop();
  diagnostics.loop();
}
