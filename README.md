# DogDoor-ESP32

DogDoor-ESP32 is the firmware that drives an automated dog door built around an
Unexpected Maker Pro S3 (ESP32-S3) board. It manages stepper-driven door
movement, dual VL53L0X time-of-flight sensors, limit switches, Wi-Fi
connectivity, MQTT telemetry, and an OLED status display. The project is built
with PlatformIO on top of the Arduino framework.

This is based on my earlier [Particle Photon DogDoor](https://github.com/shortbloke/DogDoor) project.

## Features

- **Sensor-aware door control** – polls indoor/outdoor VL53L0X sensors,
  debounces limit switches, and coordinates a FastAccelStepper-driven door state
  machine.
- **Connectivity services** – Wi-Fi bring-up with OTA support, MQTT publishing,
  Home Assistant discovery payloads, and live display of connection state.
- **OLED UX** – Wi-Fi/MQTT icons, trigger indicators, status text, and live
  distance metrics when debug display mode is enabled.
- **Serial diagnostics** – lightweight REPL (`diag` / `d` / `status`) that dumps
  door state, sensor health, and the latest readings over USB serial.
- **Structured configuration** – all hardware timings/pins live in `Config.h`
  (`ConfigData` aggregate), simplifying reuse and variant boards.

## Repository Layout

```
├── src/
│   ├── AppContainer.cpp/h        # AppContainer ownership graph and lifecycle hooks
│   ├── Config.cpp/h              # ConfigData defaults, persistent structs, pin maps
│   ├── ConnectivityManager.cpp/h # Wi-Fi, OTA, and status propagation interface
│   ├── DiagnosticsService.cpp/h  # Serial diagnostics command handling
│   ├── DisplayHelpers.cpp/h      # OLED drawing helper implementations and APIs
│   ├── DisplayService.cpp/h      # Display service logic, fonts, and interface
│   ├── DoorController.cpp/h      # Door controller orchestration and collaborators
│   ├── DoorStateMachine.cpp/h    # Door state transitions and event dispatch
│   ├── DoorTelemetryProvider.h   # Shared telemetry payload structures
│   ├── IconBitmaps.h             # Bitmap assets for status icons
│   ├── mqtt.cpp/h                # MQTT topics, payload builders, and interface
│   ├── TofSensorManager.cpp/h    # VL53L0X manager coordination and sensor structs
│   ├── main.cpp                  # Arduino entry point delegating to AppContainer
│   └── Secrets example.h         # Template for Wi-Fi/MQTT credentials
├── platformio.ini               # PlatformIO environments
└── README.md
```

## Getting Started

### Prerequisites

- [PlatformIO Core](https://platformio.org/install) or VS Code with PlatformIO
  extension
- A board based on Unexpected Maker Pro S3 (or compatible ESP32-S3 with matching
  pinout)
- VL53L0X sensors wired to the pins defined in `Config.h`
- Stepper motor driver compatible with FastAccelStepper
- Optional: Home Assistant / MQTT broker and OLED display

### Configuration

1. Copy `src/Secrets example.h` to `src/Secrets.h` and fill in Wi-Fi, OTA, and
   MQTT credentials.
2. Adjust hardware constants in `src/Config.h` if your wiring deviates from the
   defaults (pins, debounce timings, trigger thresholds, etc.).
3. (Optional) Toggle `Config.tof.enableDebugDisplay` to show sensor distances on
   the OLED during operation.

### Build & Upload

```bash
# Build
pio run

# Flash over USB
pio run -t upload

# Monitor serial output
pio device monitor
```

An `esp32_ota` environment exists for OTA uploads; update `platformio.ini` with
your device’s IP/OTA password before using `pio run -e esp32_ota -t upload`.

### MQTT & Home Assistant

- The firmware publishes retained topics under the `dogdoor/` namespace (door
  state, distances, limit switches, sensor health, etc.).
- Home Assistant discovery messages are automatically published when MQTT
  connects; entities appear under the `dogdoor` node.

Refer to `src/mqtt.cpp` if you want to adjust topics or payload structure.

### Diagnostics

Connect to the USB serial console at `115200` baud and type:

```
diag    # or d / status
help    # list commands
```

The diagnostic dump shows current door state, Wi-Fi status, last sensor trigger,
limit switch readings, and TOF ranges/health.

## Architecture Highlights

- **AppContainer** wires together the display service, TOF manager, door
  controller, connectivity manager, MQTT client, and diagnostics so that `main`
  simply calls `begin()`/`loop()`.
- **DisplayService** removes global state by wrapping OLED access in an
  injectable service; `DisplayHelpers` now simply forwards to this service.
- **TofSensorManager** encapsulates VL53L0X lifecycle management (sequential
  init, continuous reads, back-off retries) and exposes sensor health snapshots.
- **DoorStateMachine** provides an auditable transition callback so door state
  changes are centralised and easy to trace.

## Troubleshooting

| Symptom | Suggestion |
| ------- | ---------- |
| OLED shows `ERROR: VL53L0X` | Check wiring and power to the sensors; the firmware retries init and reports failures via MQTT/serial. |
| MQTT entities missing in Home Assistant | Confirm broker credentials in `Secrets.h`, verify the device is online (diagnostics), and clear retained discovery topics if needed. |
| Door not moving or seeking endlessly | Use the diagnostics dump to inspect limit switch states and TOF ranges; adjust thresholds or recalibrate stepper settings in `Config.h`. |

## Contributing & License

Contributions and feedback are always welcome. Submit pull requests or open issues with detailed descriptions of your
use case.

License information can be added here once defined.

---

Happy building, and may your dog enjoy seamless in/out privileges!

Martin
