#pragma once
#include <Arduino.h>
#include "DoorTelemetryProvider.h"

class DisplayService;

void mqttSetup(DoorTelemetryProvider *provider, DisplayService *displayService);
void mqttLoop();

void mqttPublishDoorState(DoorState state, const char *label);
void mqttPublishDistanceIndoor(float cm);
void mqttPublishDistanceOutdoor(float cm);
void mqttPublishLimitSwitchTop(bool pressed);
void mqttPublishLimitSwitchBottom(bool pressed);
void mqttPublishSensorTrigger(uint8_t triggerId);
void mqttPublishTOFSensorStatus(uint8_t sensorIndex, bool measurementOk);
void mqttPublishTOFInit(uint8_t sensorIndex);
void mqttPublishDoorOpenCount(uint32_t count);
