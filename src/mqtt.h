#pragma once
#include <Arduino.h>
#include "DoorTelemetryProvider.h"

void mqttSetup(DoorTelemetryProvider *provider);
void mqttLoop();

void mqttPublishDoorState(const char* state);
void mqttPublishDistanceIndoor(float cm);
void mqttPublishDistanceOutdoor(float cm);
void mqttSetDistancePublishInterval(uint32_t ms);
void mqttPublishLimitSwitchTop(bool pressed);
void mqttPublishLimitSwitchBottom(bool pressed);
void mqttPublishSensorTrigger(uint8_t triggerId);
