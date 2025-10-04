#pragma once

#include <Adafruit_SSD1306.h>
#include <UMS3.h>
#include "DisplayService.h"
#include "TofSensorManager.h"
#include "DoorController.h"
#include "ConnectivityManager.h"
#include "DoorTelemetryProvider.h"

// Aggregates application components so setup()/loop() can stay lean.
class AppContainer {
public:
  AppContainer();

  void begin();
  void loop();

private:
  static constexpr int kScreenWidth = 128;
  static constexpr int kScreenHeight = 32;
  static constexpr uint8_t kScreenAddress = 0x3C;
  static constexpr uint8_t kPixelBrightness = 255 / 10;

  class DoorControllerTelemetry : public DoorTelemetryProvider {
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

  Adafruit_SSD1306 display;
  UMS3 ums3;
  DisplayService displayService;
  TofSensorManager tofSensors;
  DoorController door;
  ConnectivityManager connectivity;
  DoorControllerTelemetry telemetry;
};
