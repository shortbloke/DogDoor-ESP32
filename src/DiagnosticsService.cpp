#include "DiagnosticsService.h"
#include "Config.h"
#include "DoorController.h"
#include "TofSensorManager.h"
#include "ConnectivityManager.h"
#include "Version.h"
#include <cctype>
#include <cstring>

void DiagnosticsService::loop()
{
  while (Serial.available() > 0)
  {
    char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n')
    {
      if (inputLength > 0)
      {
        inputBuffer[inputLength] = '\0';
        handleCommand();
        inputLength = 0;
      }
    }
    else if (inputLength < kInputBufferSize - 1)
    {
      inputBuffer[inputLength++] = c;
    }
  }
}

void DiagnosticsService::handleCommand()
{
  char *command = inputBuffer;
  while (*command != '\0' && std::isspace(static_cast<unsigned char>(*command)))
  {
    ++command;
  }

  char *end = command + std::strlen(command);
  while (end > command && std::isspace(static_cast<unsigned char>(end[-1])))
  {
    *--end = '\0';
  }
  for (char *p = command; *p != '\0'; ++p)
  {
    *p = static_cast<char>(std::tolower(static_cast<unsigned char>(*p)));
  }

  if (*command == '\0')
  {
    return;
  }

  if (std::strcmp(command, "d") == 0 || std::strcmp(command, "diag") == 0 ||
      std::strcmp(command, "status") == 0)
  {
    printStatus();
  }
  else if (std::strcmp(command, "help") == 0)
  {
    Serial.println(F("Diagnostics commands:"));
    Serial.println(F("  diag | d | status  - print current door diagnostics"));
    Serial.println(F("  help               - show this list"));
  }
  else
  {
    Serial.print(F("Unknown diagnostics command: "));
    Serial.println(command);
    Serial.println(F("Type 'help' for available commands."));
  }
}

void DiagnosticsService::printStatus() const
{
  Serial.println();
  Serial.println(F("=== Dog Door Diagnostics ==="));

  Serial.print(F("Firmware version: "));
  Serial.println(AppVersion::kVersionWithDate);
  Serial.print(F("Git hash: "));
  Serial.print(AppVersion::kGitHash);
  if (AppVersion::kDirty)
  {
    Serial.print(F(" (dirty)"));
  }
  Serial.println();

  const char *state = door.getStateString();
  Serial.print(F("Door state: "));
  Serial.println(state ? state : "UNKNOWN");
  Serial.print(F("Fault motor hold: "));
  Serial.println(door.isFaultMotorHolding() ? F("active") : F("inactive"));

  Serial.print(F("WiFi connected: "));
  bool wifiOk = connectivity.isConnected();
  Serial.println(wifiOk ? F("yes") : F("no"));

  Serial.print(F("Last trigger: "));
  Serial.println(door.getLastSensorTriggered());
  Serial.print(F("Completed opening count: "));
  Serial.println(door.getDoorOpenCount());
  Serial.print(F("Indoor trigger count: "));
  Serial.println(door.getSensorTriggerCount(0));
  Serial.print(F("Outdoor trigger count: "));
  Serial.println(door.getSensorTriggerCount(1));

  Serial.print(F("Indoor distance (cm): "));
  Serial.println(door.getDistanceIndoorCm(), 2);
  Serial.print(F("Outdoor distance (cm): "));
  Serial.println(door.getDistanceOutdoorCm(), 2);

  Serial.print(F("Limit bottom pressed: "));
  Serial.println(door.isLimitSwitchPressed(BottomLimitSwitch) ? F("yes") : F("no"));
  Serial.print(F("Limit top pressed: "));
  Serial.println(door.isLimitSwitchPressed(TopLimitSwitch) ? F("yes") : F("no"));

  const auto &ranges = tof.rangesMm();
  const auto &ready = tof.readyFlags();
  const auto &measurement = tof.measurementFlags();

  for (size_t i = 0; i < TofSensorConfig::count; ++i)
  {
    Serial.print(F("Sensor "));
    Serial.print(i);
    Serial.print(F(" ("));
    Serial.print(Config.tof.names[i]);
    Serial.print(F(") ready="));
    Serial.print(ready[i] ? F("yes") : F("no"));
    Serial.print(F(", measurement="));
    Serial.print(measurement[i] ? F("ok") : F("error"));
    Serial.print(F(", range(mm)="));
    Serial.println(ranges[i]);
  }

  Serial.println(F("============================"));
  Serial.println();
}
