# LeRobot Remote SO101 Plugin

Remote SO101 robot plugin for [LeRobot](https://github.com/huggingface/lerobot).  
Controls a physical SO101 robotic arm via an ESP32 TCP bridge, enabling `lerobot-replay` to run on a PC while the arm is driven wirelessly.

---

## Architecture

### Single Mode (Follower only)

```text
┌──────────────┐      WiFi/TCP      ┌─────────────┐      UART (1 Mbps)     ┌─────────────┐
│   PC (LeRobot) │  ═══════════════>  │    ESP32    │  ═══════════════════>  │ Waveshare   │
│  lerobot-replay │   JSON over TCP   │   Bridge    │   Feetech Protocol 0   │ Controller  │
│                │  <═══════════════  │             │  <════════════════════  │  (STS3215)  │
└──────────────┘      Port 8889      └─────────────┘                        └─────────────┘
                           Port 8888
```

### Dual Mode (Bimanual Leader + Follower)

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC (LeRobot)                                    │
│  ┌──────────────────┐              ┌──────────────────────────────────┐   │
│  │  Teleoperator    │              │            Robot                  │   │
│  │ RemoteSO101Leader│              │        RemoteSO101               │   │
│  │  (reads leader)  │              │      (writes follower)           │   │
│  └────────┬─────────┘              └──────────────┬───────────────────┘   │
│           │                                        │                        │
│           │ WiFi TCP (port 8889)                   │ WiFi TCP (port 8888)   │
│           │ ESP32 auto-streams positions           │ PC sends set_positions │
│           ▼                                        ▼                        │
│  ┌──────────────────┐              ┌──────────────────────────────────┐   │
│  │   ESP32 #1       │              │           ESP32 #2               │   │
│  │  (LEADER mode)   │              │        (FOLLOWER mode)           │   │
│  │  torque=OFF      │              │         torque=ON                │   │
│  │  reads positions │              │     receives positions           │   │
│  └────────┬─────────┘              └──────────────┬───────────────────┘   │
│           │ UART                                 │ UART                    │
│           ▼                                        ▼                        │
│  ┌──────────────────┐              ┌──────────────────────────────────┐   │
│  │   SO101 Leader   │              │        SO101 Follower            │   │
│  │  (human moves)   │              │      (executes commands)         │   │
│  └──────────────────┘              └──────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

- **PC side**: Two plugins — `RemoteSO101` (Robot, for follower) and `RemoteSO101Leader` (Teleoperator, for leader) — are injected into LeRobot via the official third-party plugin mechanism — **no LeRobot source code is modified**.
- **ESP32 side**: A single firmware codebase compiles to either **Leader** or **Follower** mode via compile-time `#define`. Leader auto-streams joint positions; Follower receives and executes commands.

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
    ├── esp32_so101_bridge_v2/              # ESP32 Arduino sketch (optimized)
    │   └── esp32_so101_bridge_v2.ino
    └── test_serial_only/                   # Minimal serial test
        └── test_serial_only.ino
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

1. Open `firmware/esp32_so101_bridge_v2/esp32_so101_bridge_v2.ino` in **Arduino IDE**.
2. Update WiFi credentials at the top of the sketch:
   ```cpp
   const char* WIFI_SSID     = "YOUR_WIFI_SSID";
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   ```
   (Or switch to AP mode by uncommenting the `USE_AP_MODE` block.)
3. Select your ESP32 board and port.
4. Click **Upload**.
5. Open Serial Monitor (115200 baud) and note the ESP32 IP address.

> **v2 Performance**: Uses `syncWrite` + `syncRead` to update/read all 6 servos in single bus packets, plus `TCP_NODELAY` to eliminate TCP buffering. Observation latency drops from ~30–50 ms to ~5 ms, enabling smooth 30 fps replay without skipping `get_observation`.

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

### 5. Run `lerobot-teleoperate`

The same plugin works as a **remote follower** for teleoperation. Use any LeRobot teleoperator (keyboard, gamepad, or a local leader arm) to control the remote SO101:

```powershell
# Keyboard teleoperation
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.id=black `
  --teleop.type=keyboard

# Gamepad teleoperation
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.id=black `
  --teleop.type=gamepad

# Local leader + remote follower (bimanual teleop)
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.id=black `
  --teleop.type=so100_leader `
  --teleop.port=/dev/ttyUSB0
```

> **Note**: For teleoperation, `skip_observation` defaults to `false` so the teleop loop receives real-time joint positions. The v2 firmware's `syncRead` provides ~5 ms observation latency, sufficient for 30–60 Hz teleop loops.

### Bimanual Teleoperation (Leader + Follower)

With **two ESP32 boards** (one for Leader, one for Follower) and the **dual-mode v3 firmware**, you can teleoperate the Follower arm by physically moving the Leader arm — both wirelessly:

#### 1. Flash the ESP32s

Open `firmware/esp32_so101_bridge_v2/esp32_so101_bridge_v2.ino` and set the mode at the top:

**ESP32 #1 (Leader):**
```cpp
#define LEADER_MODE
// #define FOLLOWER_MODE
```

**ESP32 #2 (Follower):**
```cpp
// #define LEADER_MODE
#define FOLLOWER_MODE
```

Update WiFi credentials for both (or use AP mode with different SSIDs), then flash each ESP32.

#### 2. Run bimanual teleoperation

```powershell
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.1.102 `
  --robot.id=black `
  --teleop.type=remote_so101_leader `
  --teleop.remote_ip=192.168.1.101 `
  --teleop.id=black
```

| Argument | Meaning |
|----------|---------|
| `--robot.remote_ip` | Follower ESP32 IP |
| `--teleop.remote_ip` | Leader ESP32 IP |
| `--robot.id` / `--teleop.id` | Calibration ID (must match existing SO101 calibration) |

**What happens:**
1. Leader ESP32 continuously reads servo positions and streams them to PC at 30 Hz
2. `RemoteSO101Leader` receives the stream and returns normalized actions
3. LeRobot's teleop loop feeds these actions to `RemoteSO101`
4. `RemoteSO101` denormalizes and sends them to the Follower ESP32
5. Follower ESP32 writes positions to its servos

> ⚠️ **Important**: Both Leader and Follower must use the **same calibration file** (same `id`) so that joint positions are interpreted identically. The Leader teleoperator reuses the `so_follower` calibration directory by default.

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

Burn the **minimum serial test** first to verify hardware:

```bash
# Open firmware/test_serial_only/test_serial_only.ino
# Upload → Serial Monitor (115200) → Press RST
```

If that works but the main sketch fails, check the v2 Serial Monitor output. Expected:
```
[V2] ===== ESP32 SO101 Bridge v2 (Optimized) =====
[V2] UART2 initialized
[V2] Scanning servos...
[V2]   ID=1 pos=2048
...
[V2] WiFi IP: 192.168.1.105
[V2] TCP CMD port 8888, OBS port 8889
[V2] Setup complete. Waiting for PC...
```



| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `No calibration file found` | Missing or misnamed `.json` | Copy the SO101 calibration file to `~/.cache/huggingface/lerobot/calibration/robots/so_follower/{id}.json` |
| `Cannot connect to ESP32` | Wrong IP / WiFi issue | Check Serial Monitor for the ESP32 IP; ensure PC and ESP32 are on the same network |
| `Servos do not move` | Torque not enabled / wiring | Verify GPIO17/18 wiring; ensure Waveshare board is powered; check ESP32 Serial output |
| `Motion feels slow / stuttery` | `get_observation()` RTT too high | v2 firmware uses `syncRead` (~5 ms) + `TCP_NODELAY`; if still slow, use `--robot.skip_observation=true` |
| `Jerk / overshoot` | PID / acceleration settings | v2 firmware sets `ACC=254` at startup; if needed, adjust PID via Feetech config tool |

---

## Performance Tuning

### Option A: Use v2 firmware (recommended)
The v2 firmware (`esp32_so101_bridge_v2`) uses `syncWrite` + `syncRead` + `TCP_NODELAY` to minimize latency. This is sufficient for most replay scenarios.

### Option B: Skip observation on PC side
If you are **only replaying** (not teleoperating) and still experience stutter, the PC-side plugin can skip `get_observation()` entirely:

```powershell
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.skip_observation=true `
  --dataset.repo_id=your_username/your_dataset `
  --dataset.episode=0
```

> ⚠️ Only safe for **replay** — do NOT use for teleoperation or closed-loop control.

---

## AP Mode (Bypass Chinese SSID / No Router)

If your WiFi router uses **Chinese characters in the SSID**, ESP32 may fail to connect due to encoding mismatch. The simplest fix is to use **AP mode** — the ESP32 creates its own WiFi hotspot.

### How to enable AP mode

In `esp32_so101_bridge_v2.ino`, find this section at the top:

```cpp
// Option B: Access Point mode
// Uncomment the line below to use AP mode:
// #define USE_AP_MODE
```

Uncomment the `#define`:

```cpp
#define USE_AP_MODE
const char* AP_SSID     = "SO101-ROBOT";
const char* AP_PASSWORD = "12345678";
```

Re-flash the ESP32. It will now broadcast a WiFi named `SO101-ROBOT`.

### PC connection

1. On your PC, connect to the `SO101-ROBOT` WiFi (password: `12345678`).
2. Check the Serial Monitor for the AP IP — usually `192.168.4.1`.
3. Use that IP when running `lerobot-replay`:

```powershell
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.4.1 `
  ...
```

> ⚠️ **Note**: In AP mode, your PC must be connected to the ESP32's WiFi. If your PC also needs internet access, you may need a second network interface (e.g., Ethernet or a USB WiFi dongle) or temporarily switch to mobile hotspot with an English SSID.

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
