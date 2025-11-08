# Hill Climb Assist Platform

This project is a two-board control stack for an RC “hill-climb” prototype:

* **STM32 BlackPill (F411)** – Reads an MPU9250 IMU, renders a spirit-level UI, drives the TB6612FNG dual motor driver, and actuates two brake servos.
* **ESP32 Companion** – Bridges commands from a laptop/phone over Wi-Fi (and USB serial) to the STM board, hosts a control page, and mirrors telemetry back to the user.

Both boards are PlatformIO projects; build each from its own folder (`Hillclimbing_Project_blackpill` and `HillClimbingAssist_ESP32`).

---

## Hardware Summary

| Peripheral                     | MCU Pins                                   |
|--------------------------------|--------------------------------------------|
| IMU (MPU9250 I²C1)             | PB7 (SDA), PB8 (SCL)                       |
| OLED SSD1306 (I²C2)            | PB3 (SDA), PB10 (SCL)                      |
| TB6612FNG – Channel A          | PB13 (PWMA), PB15 (AIN1), PB14 (AIN2)      |
| TB6612FNG – Channel B          | PB0 (PWMB),  PA9 (BIN1), PA10 (BIN2)       |
| TB6612FNG Standby              | PA8                                        |
| Brake Servo Left               | PA5                                        |
| Brake Servo Right              | PA6                                        |
| Assist Toggle Button           | PC14 (other side to GND)                   |
| ESP32 UART Bridge (STM-side)   | PA3 (RX), PA2 (TX)                         |
| ESP32 UART Bridge (ESP-side)   | GPIO16 (RX2), GPIO17 (TX2)                 |

**Power**: Motors/servos draw from the TB6612’s VM supply (ensure common ground with the BlackPill). Brake servos will hold position continuously—size the supply accordingly.

---

## STM32 Firmware Overview

* **IMU & Level Display** – The MPU9250 is sampled (200 Hz base configuration), filtered, and used to compute roll/pitch. The SSD1306 shows orientation, temperature, command/output PWM, and the hill-assist status.
* **Motor Control** – User commands (left/right PWM) arrive via the ESP32 bridge. Hill-assist uses filtered pitch to add a *gradual* boost (see `kAssistPitchThresholdDeg`, `kAssistGainPerDeg`, `kAssistRampStep`). Outputs saturate at ±255.
* **Brake Servos** – Two servos on PA5/PA6 act as disc brakes. When assist is enabled and both motor commands are zero, brakes auto-engage; otherwise they release. Manual commands (`BRAKE`, `RELEASE`) are available.
* **Assist Toggle** – The PC14 latch switch directly controls `gAssistEnabled`. Toggling it updates the ESP32, OLED, and automatically reapplies releases.
* **Telemetry & Logging** – All status lines include button state, assist state, brake state, command PWM, and applied PWM. Telemetry streams to the ESP32 (`STM_STATUS`, `STM_IMU`) and USB serial.

Key tuneables in `src/main.cpp`:

```cpp
constexpr float  kAssistPitchThresholdDeg = 8.0f;
constexpr float  kAssistGainPerDeg        = 6.0f;
constexpr int16_t kAssistRampStep         = 5;
constexpr int16_t kAssistMaxBoost         = 120;

int gBrakeLeftRelease  = 90;
int gBrakeLeftApplied  = 120;
int gBrakeRightRelease = 120;
int gBrakeRightApplied = 90;
```

Adjust the servo angles to match the mechanical brake positions on your build (degrees, 0–180). Fine-tune the assist gain/ramp constants for your terrain.

---

## ESP32 Companion Firmware

* Starts a SoftAP (`HillAssist-ESP32` / password `hillassist`), runs an HTTP server, and serves a control dashboard at `http://192.168.4.1`.
* UI includes:
  * Speed slider + FWD/BACK/LEFT/RIGHT buttons
  * STOP / `/ping` / `/reset` helpers
  * Custom command box (send raw strings such as `MOTORS 120 -80`, `BRAKE`, `RELEASE`)
  * Live status panel with uptime, last command, last STM line, AP IP, and log buffer
* Also keeps a USB serial CLI at 115200 baud for redundancy (type the same commands as above).
* Forwards all commands to the STM32 via UART2 (pins 16/17) and mirrors STM responses/logs back to the UI & serial terminal.

---

## Command Reference

These commands are recognized from the ESP32 web UI, USB serial, or forwarded from other hosts (the STM32 also parses them when coming from ESP):

| Command          | Description                                           |
|------------------|-------------------------------------------------------|
| `FWD [speed]`    | Drive forward (0–255).                                |
| `BACK [speed]`   | Drive backward.                                       |
| `LEFT [speed]`   | Pivot left (left motor reverse, right forward).       |
| `RIGHT [speed]`  | Pivot right.                                          |
| `STOP` / `BRAKE` | Zero both motors; if assist is ON, brakes engage.     |
| `BRAKE OFF`/`RELEASE` | Release brakes manually.                         |
| `MOTORS L R`     | Direct PWM setpoints (-255..255).                     |
| `/ping`          | Send `ESP32_PING` to STM; used for link testing.      |
| `/reset`         | Send `ESP32_RESET` to STM (no MCU reboot, just signal).|
| `/help` or `/ ?` | Show the command list.                                |

The STM32 accepts the same raw strings when you connect via USB serial (115200 baud) on the BlackPill itself.

---

## Build & Flash (PlatformIO)

### STM32 BlackPill

```bash
cd Hillclimbing_Project_blackpill
pio run -t upload
pio device monitor  # optional: view serial logs @115200
```

The project uses the DFU bootloader (`upload_protocol = dfu`). Put the BlackPill into DFU mode when flashing.

### ESP32 Companion (UPesy WROOM)

```bash
cd HillClimbingAssist_ESP32
pio run -t upload
pio device monitor  # to issue USB commands if desired
```

After flashing, connect your laptop to the `HillAssist-ESP32` Wi-Fi network and browse to `http://192.168.4.1`.

---

## Bringing the System Up

1. Wire the hardware per the table above, ensuring grounds are tied together and high-current components (motors/servos) have adequate power.
2. Flash the STM32 and ESP32 firmware.
3. Power on both boards; observe the STM32 OLED for status and the ESP32 serial log for SoftAP info.
4. Toggle the PC14 switch to enable assist; ensure the brakes move to the “applied” angle (adjust servo angles in `src/main.cpp` if needed).
5. Open the ESP32 dashboard in a browser or use USB serial to issue drive commands.
6. Watch the OLED & telemetry: when the bot climbs, the assist boost will smoothly add PWM; when it stops, brakes engage automatically if assist remains on.

---

## Troubleshooting Tips

* **Motor channel B not moving** – Ensure `PWMB` is wired to PB0 (PA11 is USB DM and cannot output PWM).
* **Servos buzzing or moving the wrong way** – Adjust `gBrakeLeft/RightRelease/Applied` to the proper angles; they are plain degree values.
* **Assist feels too aggressive** – Reduce `kAssistGainPerDeg`, `kAssistMaxBoost`, or increase `kAssistRampStep` for slower changes.
* **ESP32 page unreachable** – Confirm you’re connected to the SoftAP; check serial logs for the AP IP and ensure no firewall blocks TCP port 80.
* **IMU missing/OLED blank** – Verify PB7/PB8 and PB3/PB10 wiring and pull the STM32’s reset if the IMU cube fails to probe.

---


