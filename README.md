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
- **Controlled startup homing** – when position is unknown, moves slowly toward
  the bottom limit with bounded travel and timeout protection.
- **Early startup hold** – energizes the stepper before display, sensor, and
  network initialization to resist gravity while door position is established.
- **Fail-safe fault hold** – immediately stops commanded motion while retaining
  motor torque so an elevated door does not fall after a software fault.
- **Connectivity services** – Wi-Fi bring-up with OTA support, MQTT publishing,
  Home Assistant discovery payloads, and live display of connection state.
- **OLED UX** – Wi-Fi/MQTT icons, trigger indicators, status text, and live
  distance metrics when debug display mode is enabled.
- **Serial diagnostics** – lightweight REPL (`diag` / `d` / `status`) that dumps
  door state, sensor health, and the latest readings over USB serial.
- **Structured configuration** – all hardware timings/pins live in `Config.h`
  (`ConfigData` aggregate), simplifying reuse and variant boards.

## Repository Layout

```text
├── src/
│   ├── AppContainer.cpp/h        # AppContainer ownership graph and lifecycle hooks
│   ├── Config.cpp/h              # Configuration defaults and hardware pin maps
│   ├── ConnectivityManager.cpp/h # Wi-Fi, OTA, and status propagation interface
│   ├── DiagnosticsService.cpp/h  # Serial diagnostics command handling
│   ├── DisplayService.cpp/h      # Display service logic, fonts, and interface
│   ├── DoorController.cpp/h      # Door controller orchestration and collaborators
│   ├── DoorSafetyPolicy.h        # Pure, testable motion and reporting safety rules
│   ├── DoorStateMachine.cpp/h    # Door state transitions and event dispatch
│   ├── DoorTelemetryProvider.h   # Read-only MQTT telemetry interface
│   ├── IconBitmaps.h             # Bitmap assets for status icons
│   ├── mqtt.cpp/h                # MQTT topics, payload builders, and interface
│   ├── StatusPixel.cpp/h         # Onboard RGB status pixel control
│   ├── TofSensorManager.cpp/h    # VL53L0X manager coordination and sensor structs
│   └── main.cpp                  # Arduino entry point delegating to AppContainer
├── scripts/                     # Build-time version header generation
├── test/                        # Native state-machine tests
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

1. Create the ignored `src/Secrets.h` with the required Wi-Fi, OTA, and MQTT
   definitions used by `ConnectivityManager.cpp` and `mqtt.cpp`.
2. Adjust hardware constants in `src/Config.h` if your wiring deviates from the
   defaults (pins, debounce timings, trigger thresholds, etc.).
3. (Optional) Toggle `Config.tof.enableDebugDisplay` to show sensor distances on
   the OLED during operation.

The bottom limit switch must actuate before the door reaches its physical hard
stop. Leave enough mechanical margin for the motor to stop; firmware checks the
raw switch before other loop work but cannot compensate for a switch mounted at
or beyond the hard stop.

The early motor hold only applies after firmware begins executing. Preventing a
fall during power loss, reset, or flashing requires mechanical support such as a
normally engaged brake, self-locking drive, or counterbalance.

### Build & Upload

```bash
# Build
pio run

# Flash over USB
pio run -t upload

# Monitor serial output
pio device monitor
```

An `esp32_ota` environment exists for OTA uploads. Create an ignored
`private.ini` containing an `[env:esp32_ota]` section with `upload_port` set to
the device's `.local` mDNS hostname and `upload_flags` containing its OTA
password before running `pio run -e esp32_ota -t upload`.

### Versioning

- Each build runs `scripts/generate_version_file.py`, which reads the stable
  `custom_firmware_version` value from `platformio.ini`, stamps the current UTC
  date and Git metadata, and writes `src/Version.h` (kept out of Git).
- The latest version, build date, and git hash appear in the serial diagnostics
  output and the MQTT device metadata (`sw_version`) so Home Assistant and other
  clients see the same information.
- Update `custom_firmware_version` deliberately when preparing a release.

### MQTT & Home Assistant

- The firmware publishes retained topics under the `pet_door/` namespace (door
  state, distances, limit switches, sensor health, etc.).
- Persistent lifetime counters distinguish completed opening cycles from indoor
  and outdoor sensor-trigger events. Home Assistant can derive daily totals
  from any of these monotonic counters without losing counts across restarts.
- The binary door topic reports `CLOSED` only for a confirmed closed state;
  homing, movement, faults, and unknown states conservatively report `OPEN`.
- Home Assistant discovery messages are automatically published when MQTT
  connects; entities appear under the `pet_door` node.

Refer to `src/mqtt.cpp` if you want to adjust topics or payload structure.

### Diagnostics

Connect to the USB serial console at `115200` baud and type:

```text
diag    # or d / status
help    # list commands
```

The diagnostic dump shows current door state, Wi-Fi status, last sensor trigger,
limit switch readings, and TOF ranges/health.

## Architecture Highlights

- **AppContainer** wires together the display service, TOF manager, door
  controller, connectivity manager, MQTT client, and diagnostics so that `main`
  simply calls `begin()`/`loop()`.
- **DisplayService** wraps OLED access in a service passed directly to the
  components that render connectivity and door status.
- **TofSensorManager** encapsulates VL53L0X lifecycle management (sequential
  init, continuous reads, back-off retries) and exposes sensor health snapshots.
- **DoorStateMachine** provides an auditable transition callback so door state
  changes are centralised and easy to trace.
- **DoorSafetyPolicy** keeps endpoint, sensor-polling, and fail-safe MQTT rules
  independent of hardware so they can be covered by native tests.

## Troubleshooting

| Symptom | Suggestion |
| ------- | ---------- |
| OLED shows `ERROR: VL53L0X` | Check wiring and power to the sensors; the firmware retries init and reports failures via MQTT/serial. |
| MQTT entities missing in Home Assistant | Confirm broker credentials in `Secrets.h`, verify the device is online (diagnostics), and clear retained discovery topics if needed. |
| Door not moving or seeking endlessly | Use the diagnostics dump to inspect limit switch states and TOF ranges; adjust thresholds or recalibrate stepper settings in `Config.h`. |

## Contributing & License

Contributions and feedback are always welcome. Submit pull requests or open issues with detailed descriptions of your
use case.

DogDoor-ESP32 is released under the MIT License. See [`LICENSE`](LICENSE) for the full terms.

---

Happy building, and may your dog enjoy seamless in/out privileges!

Martin
