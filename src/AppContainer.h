#pragma once

#include <Adafruit_SSD1306.h>
#include "DisplayService.h"
#include "StatusPixel.h"
#include "TofSensorManager.h"
#include "DoorController.h"
#include "ConnectivityManager.h"
#include "DiagnosticsService.h"

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

  Adafruit_SSD1306 display;
  StatusPixel statusPixel{kPixelBrightness};
  DisplayService displayService;
  TofSensorManager tofSensors;
  DoorController door;
  ConnectivityManager connectivity;
  DiagnosticsService diagnostics;
};
