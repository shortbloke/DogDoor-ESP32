#include "DiagnosticsService.h"
#include "Config.h"
#include "DoorController.h"
#include "TofSensorManager.h"
#include "ConnectivityManager.h"

void DiagnosticsService::begin(DoorController *doorController,
                               TofSensorManager *tofManager,
                               ConnectivityManager *connectivityManager)
{
  door = doorController;
  tof = tofManager;
  connectivity = connectivityManager;
  inputLength = 0;
}

void DiagnosticsService::loop()
{
  if (!door || !tof)
  {
    return;
  }

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
  String command(inputBuffer);
  command.trim();
  command.toLowerCase();

  if (command.length() == 0)
  {
    return;
  }

  if (command == "d" || command == "diag" || command == "status")
  {
    printStatus();
  }
  else if (command == "help")
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

  const char *state = door->getStateString();
  Serial.print(F("Door state: "));
  Serial.println(state ? state : "UNKNOWN");

  Serial.print(F("WiFi connected: "));
  bool wifiOk = connectivity ? connectivity->isConnected() : door->isWiFiConnected();
  Serial.println(wifiOk ? F("yes") : F("no"));

  Serial.print(F("Last trigger: "));
  Serial.println(door->getLastSensorTriggered());

  Serial.print(F("Indoor distance (cm): "));
  Serial.println(door->getDistanceIndoorCm(), 2);
  Serial.print(F("Outdoor distance (cm): "));
  Serial.println(door->getDistanceOutdoorCm(), 2);

  Serial.print(F("Limit bottom pressed: "));
  Serial.println(door->isLimitSwitchPressed(BottomLimitSwitch) ? F("yes") : F("no"));
  Serial.print(F("Limit top pressed: "));
  Serial.println(door->isLimitSwitchPressed(TopLimitSwitch) ? F("yes") : F("no"));

  const auto &ranges = tof->rangesMm();
  const auto &ready = tof->readyFlags();
  const auto &measurement = tof->measurementFlags();

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
