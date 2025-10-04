#pragma once
#include <Arduino.h>
#include "DoorTelemetryProvider.h"

void mqttSetup(DoorTelemetryProvider *provider);
void mqttLoop();

void mqttPublishDoorState(const char* state);
void mqttPublishDistanceIndoor(float cm);
void mqttPublishDistanceOutdoor(float cm);
void mqttPublishLimitSwitchTop(bool pressed);
void mqttPublishLimitSwitchBottom(bool pressed);
void mqttPublishSensorTrigger(uint8_t triggerId);
void mqttPublishTOFSensorStatus(uint8_t sensorIndex, bool measurementOk);
void mqttPublishTOFInit(uint8_t sensorIndex);
