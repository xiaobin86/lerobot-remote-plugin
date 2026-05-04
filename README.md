# LeRobot Remote SO101 Plugin

Remote SO101 robot plugin for [LeRobot](https://github.com/huggingface/lerobot).  
Controls a physical SO101 robotic arm via an ESP32 TCP bridge, enabling `lerobot-replay` to run on a PC while the arm is driven wirelessly.

---

## Architecture

```text
┌──────────────┐      WiFi/TCP      ┌─────────────┐      UART (1 Mbps)     ┌─────────────┐
│   PC (LeRobot) │  ═══════════════>  │    ESP32    │  ═══════════════════>  │ Waveshare   │
│  lerobot-replay │   JSON over TCP   │   Bridge    │   Feetech Protocol 0   │ Controller  │
│                │  <═══════════════  │             │  <════════════════════  │  (STS3215)  │
└──────────────┘      Port 8889      └─────────────┘                        └─────────────┘
                           Port 8888
```

- **PC side**: A custom `Robot` subclass (`RemoteSO101`) is injected into LeRobot via the official third-party plugin mechanism — **no LeRobot source code is modified**.
- **ESP32 side**: A lightweight Arduino firmware receives JSON commands over TCP and forwards them to the Waveshare Feetech controller board via UART (GPIO17/18).

---

## Repository Structure (Git-Flow)

```text
lerobot-robot-remote-so101/
├── .gitignore
├── pyproject.toml                          # Python package manifest
├── README.md                               # This file
├── lerobot_robot_remote_so101/             # Python plugin source
│   ├── __init__.py
│   ├── config.py                           # RemoteSO101Config
│   └── remote_so101.py                     # RemoteSO101 Robot implementation
└── firmware/
    └── esp32_so101_bridge/
        └── esp32_so101_bridge.ino          # ESP32 Arduino sketch
```

**Branch model:**
- `main`    – stable releases
- `develop` – integration branch
- `feature/remote-so101-replay` – current development branch (where this feature lives)

To go back to the state **before** this extension existed, simply uninstall the pip package:
```bash
pip uninstall lerobot_robot_remote_so101
```
The LeRobot source repository remains completely untouched.

---

## Prerequisites

- LeRobot installed from source or pip (`lerobot>=0.5.0`)
- Python 3.10+
- ESP32 development board
- Arduino IDE or PlatformIO
- Waveshare Feetech servo controller board (connected to 6× STS3215 servos)
- WiFi network (or ESP32 AP mode)

---

## Quick Start

### 1. Install the PC-side plugin

```bash
cd D:\work\lerobot-workspace\lerobot-robot-remote-so101
pip install -e .
```

The package name `lerobot_robot_remote_so101` starts with `lerobot_robot_`, so LeRobot's `register_third_party_plugins()` will **auto-discover** it on startup.

### 2. Flash the ESP32

1. Open `firmware/esp32_so101_bridge/esp32_so101_bridge.ino` in **Arduino IDE**.
2. Update WiFi credentials at the top of the sketch:
   ```cpp
   const char* WIFI_SSID     = "YOUR_WIFI_SSID";
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   ```
   (Or switch to AP mode by uncommenting the `USE_AP_MODE` block.)
3. Select your ESP32 board and port.
4. Click **Upload**.
5. Open Serial Monitor (115200 baud) and note the ESP32 IP address.

### 3. Verify calibration files exist

`RemoteSO101` reuses the same calibration files as the local SO101 robot.  
Ensure the calibration JSON exists at the default location:

```text
%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\{robot_id}.json
```

If you calibrated your SO101 with a specific `--robot.id`, that ID must match the file name.  
Example: if you used `--robot.id=black`, the file must be named `black.json`.

### 4. Run `lerobot-replay`

```powershell
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.id=black `
  --dataset.repo_id=your_username/your_dataset `
  --dataset.episode=0
```

Replace:
- `192.168.x.x` – your ESP32 IP address
- `black` – your SO101 robot ID (must match the calibration file name)
- `your_username/your_dataset` – your Hugging Face dataset repo

---

## Command Reference

### PC → ESP32 (`port_cmd`, port 8888)

| Command | Payload | Description |
|---------|---------|-------------|
| `set_positions` | `{"positions":{"shoulder_pan":2048,...}}` | Move all joints to target steps (0–4095) |
| `get_obs` | `{}` | Request current joint positions |

### ESP32 → PC (`port_obs`, port 8889)

Response to `get_obs`:
```json
{"shoulder_pan":2048,"shoulder_lift":1024,"elbow_flex":3072,
 "wrist_flex":1500,"wrist_roll":2000,"gripper":100}
```

---

## Calibration & Denormalisation

LeRobot datasets store **normalised** actions (default `RANGE_M100_100`, i.e. floats in `[-100, 100]`).  
The `RemoteSO101` plugin performs the exact same denormalisation as the local `SOFollower` before sending native motor steps to the ESP32:

```python
native = ((norm_val + 100) / 200) * (range_max - range_min) + range_min
```

Therefore the calibration file (`range_min`, `range_max`, `drive_mode`) must be identical to the one used during dataset recording.

---

## Troubleshooting

### ESP32 Serial Monitor shows nothing after flashing

This is the most common issue. Follow this exact sequence:

1. **Check USB cable** — use a cable that is confirmed to transfer data (not charge-only).
2. **Check COM port** — Arduino IDE → Tools → Port → select the port that appears when you plug in the ESP32.
3. **Check Baud Rate** — Arduino IDE Serial Monitor → bottom-right dropdown → select **115200**.
4. **Press RST button** — ESP32 does **not** auto-reset after upload; you must press the physical `RST` button on the board.

If you still see nothing, burn the **minimum serial test** first:

```bash
# Open firmware/test_serial_only/test_serial_only.ino in Arduino IDE
# Upload → Open Serial Monitor (115200) → Press RST
```

You should see `Alive: 0`, `Alive: 1`, `Alive: 2` every second.  
- **If you do** → USB serial works; the issue is in the main sketch (probably WiFi blocking or a crash).
- **If you do NOT** → hardware/USB driver issue; try a different cable or USB port.

### Still no output from the main sketch?

Burn the **diagnostic version** which prints a log at every step:

```bash
# Open firmware/esp32_so101_bridge_diag/esp32_so101_bridge_diag.ino
# Update WiFi credentials → Upload → Serial Monitor (115200) → Press RST
```

Expected output:
```
[DIAG] ===== ESP32 SO101 Bridge (DIAG) =====
[DIAG] Serial USB initialized OK
[DIAG] Initializing UART2 (TX=17 RX=18)...
[DIAG] UART2 initialized OK
[DIAG] Enabling torque on all motors...
[DIAG] Torque enable packets sent
[DIAG] Probing motor ID 1...
[DIAG] Motor ID 1 present position: 2048
[DIAG] Connecting to WiFi: YOUR_WIFI_SSID
.....
[DIAG] WiFi Connected. IP: 192.168.1.105
[DIAG] TCP CMD server on port 8888
[DIAG] TCP OBS server on port 8889
[DIAG] Setup complete. Waiting for PC client...
```

If the output stops at a specific line, that is exactly where the problem is:

| Last visible line | Problem | Fix |
|---|---|---|
| `Serial USB initialized OK` | Code never reaches UART init | Very rare; likely a crash before that line |
| `Initializing UART2...` | `Serial2.begin()` crashes | Check GPIO pins; some ESP32 boards reserve 17/18 for PSRAM |
| `Torque enable packets sent` | Motors not responding | Check Waveshare power (5V) and UART wiring (TX↔RX) |
| `Probing motor ID 1...` | No response from servo bus | Waveshare not powered, or wrong baud-rate, or servo IDs ≠ 1-6 |
| `Connecting to WiFi...` (dots forever) | Wrong WiFi credentials | Double-check SSID/PASSWORD; try AP mode |
| `WiFi Connected` but no TCP | Firewall blocking | Ensure Windows Defender / antivirus allows ports 8888/8889 |

### Symptoms Table

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `No calibration file found` | Missing or misnamed `.json` | Copy the SO101 calibration file to `~/.cache/huggingface/lerobot/calibration/robots/so_follower/{id}.json` |
| `Cannot connect to ESP32` | Wrong IP / WiFi issue | Check Serial Monitor for the ESP32 IP; ensure PC and ESP32 are on the same network |
| `Servos do not move` | Torque not enabled / wiring | Verify GPIO17/18 wiring; ensure Waveshare board is powered; check ESP32 Serial output |
| `Jerk / overshoot` | PID / acceleration settings | Configure servo PID (P=16, I=0, D=32) and `Acceleration=254` via the Feetech config tool, or add setup code to ESP32 `setup()` |

---

## Advanced: Switching to AP Mode

If you do not have a router nearby, uncomment the following block in the Arduino sketch:

```cpp
#define USE_AP_MODE
const char* AP_SSID     = "SO101-ROBOT";
const char* AP_PASSWORD = "12345678";
```

The ESP32 will create a WiFi access point. Connect your PC to it, then use the ESP32's default AP IP (usually `192.168.4.1`) as `--robot.remote_ip`.

---

## Development

### Git-Flow Workflow

```bash
# Currently on feature/remote-so101-replay
git flow feature finish remote-so101-replay   # merges into develop
git checkout develop
git checkout -b release/v0.1.0
# ... version bump, final tests ...
git checkout main
git merge release/v0.1.0
git tag v0.1.0
```

### Running Tests

```bash
pytest tests/   # (add tests as needed)
```

---

## License

Apache-2.0 (same as LeRobot)
