# LeRobot 远程回放改造实录：让 lerobot-replay 通过 WiFi 控制机械臂

> **目标**：在不修改 LeRobot 源码的前提下，通过继承和插件机制，实现 `lerobot-replay` 命令通过 WiFi 远程发送数据给 ESP32，由 ESP32 控制 SO101 机械臂运动。
>
> **适用硬件**：LeRobot SO101 机械臂 + 微雪(Waveshare) Feetech 控制板 + ESP32 开发板（GPIO17/18 连接控制板 UART）
>
> **完成时间**：2026-05-05

---

## 目录

1. [项目概述与架构设计](#1-项目概述与架构设计)
2. [实现思路：LeRobot 插件机制深度分析](#2-实现思路lerobot-插件机制深度分析)
3. [PC 端实现：RemoteSO101 插件](#3-pc-端实现remoteso101-插件)
4. [ESP32 端实现：TCP → UART 桥接固件](#4-esp32-端实现tcp--uart-桥接固件)
5. [关键设计：反归一化（Denormalisation）](#5-关键设计反归一化denormalisation)
6. [完整安装与运行流程](#6-完整安装与运行流程)
7. [卸载方法：零残留回退](#7-卸载方法零残留回退)
8. [Git-Flow 分支管理](#8-git-flow-分支管理)
9. [踩坑记录与解决方案](#9-踩坑记录与解决方案)
10. [仓库文件清单](#10-仓库文件清单)

---

## 1. 项目概述与架构设计

### 1.1 为什么需要远程控制？

LeRobot 默认的 `lerobot-replay` 通过 USB 串口直连机械臂控制板。这带来几个限制：
- **线材束缚**：机械臂活动范围受限于 USB 线长度
- **PC 位置固定**：控制电脑必须放在机械臂旁边
- **多机协同困难**：一台 PC 难以同时控制多台机械臂

### 1.2 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              PC (Windows)                                │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  LeRobot (lerobot-smolvla) - 源码完全不修改                       │  │
│  │                                                                   │  │
│  │  lerobot-replay ──▶ make_robot_from_config(cfg.robot)            │  │
│  │                          │                                        │  │
│  │                          ▼                                        │  │
│  │  ┌─────────────────────────────────┐                              │  │
│  │  │  RemoteSO101 (第三方插件注入)    │                              │  │
│  │  │  ├─ _denormalize(action)        │                              │  │
│  │  │  ├─ send_action() ──▶ TCP 8888  │                              │  │
│  │  │  └─ get_observation() ◀── TCP 8889│                            │  │
│  │  └─────────────────────────────────┘                              │  │
│  └───────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
                                    │ WiFi / TCP
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                              ESP32 开发板                                │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  TCP Server (8888): JSON 命令接收                                 │  │
│  │  TCP Server (8889): JSON 观测回传                                 │  │
│  │                                                                   │  │
│  │  SYNC_WRITE(Goal_Position) ──▶ UART2 (GPIO17/18)                 │  │
│  └───────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
                                    │ 1 Mbps, Feetech Protocol 0
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         微雪 Feetech 控制板                              │
│                              6× STS3215 舵机                             │
│                    (shoulder_pan ~ gripper, IDs 1-6)                     │
└─────────────────────────────────────────────────────────────────────────┘
```

**核心设计原则**：
- **零源码修改**：LeRobot 源码仓库 `lerobot-smolvla` 完全不动
- **插件化注入**：通过 LeRobot 官方支持的第三方插件机制加载 `RemoteSO101`
- **校准复用**：直接使用 SO101 现有的校准文件，无需重新校准
- **反归一化在 PC 端完成**：ESP32 只接收原生步进值（0-4095），固件极简

---

## 2. 实现思路：LeRobot 插件机制深度分析

### 2.1 入口点分析

`lerobot-replay` 的入口在 `pyproject.toml`：

```toml
[project.scripts]
lerobot-replay="lerobot.scripts.lerobot_replay:main"
```

`main()` 函数做了两件事：

```python
def main():
    register_third_party_plugins()   # <-- 关键！自动导入插件包
    replay()
```

### 2.2 第三方插件自动发现机制

`register_third_party_plugins()`（位于 `lerobot/utils/import_utils.py`）会自动 import 所有以 `lerobot_robot_` 为前缀的 pip 包：

```python
def register_third_party_plugins() -> None:
    prefixes = ("lerobot_robot_", "lerobot_camera_", 
                "lerobot_teleoperator_", "lerobot_policy_")
    for dist in importlib.metadata.distributions():
        dist_name = dist.metadata.get("Name")
        if dist_name and dist_name.startswith(prefixes):
            importlib.import_module(dist_name)
```

**这意味着**：只要我们的包名是 `lerobot_robot_xxx` 并安装到环境中，LeRobot 启动时会**自动加载**，完全不需要修改 LeRobot 源码。

### 2.3 配置注册：draccus ChoiceRegistry

LeRobot 使用 `draccus` 库（类似 Hydra）处理配置。`RobotConfig` 继承自 `draccus.ChoiceRegistry`：

```python
# lerobot/robots/config.py
@dataclass(kw_only=True)
class RobotConfig(draccus.ChoiceRegistry, abc.ABC):
    ...
    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
```

子类通过装饰器注册：

```python
@RobotConfig.register_subclass("remote_so101")
@dataclass
class RemoteSO101Config(RobotConfig):
    ...
```

用户通过 `--robot.type=remote_so101` 在 CLI 中指定使用我们的机器人。

### 2.4 工厂加载链

```
lerobot-replay
    └── replay(cfg)
            └── make_robot_from_config(cfg.robot)
                    ├── 硬编码分支: so100_follower, so101_follower, ...
                    └── fallback: make_device_from_device_class(config)
```

对于未知的 `config.type`，`make_robot_from_config` 会 fallback 到 `make_device_from_device_class()`，它通过命名约定自动查找类：
- 配置类名 `MyRobotConfig` → 查找 `MyRobot`
- 搜索同模块或父模块

因此，只要我们注册的配置类名以 `Config` 结尾，并放置在同模块的 `Robot` 类，就能被自动实例化。

### 2.5 参考实现：LeKiwiClient

LeRobot 官方已经有一个远程机器人的实现：`LeKiwiClient`（`src/lerobot/robots/lekiwi/lekiwi_client.py`）。它通过 **ZMQ** 与远端 `LeKiwiHost` 通信。这证明"远程 Robot"这一模式在 LeRobot 架构中是完全可行的。

我们的实现参考了 `LeKiwiClient` 的结构，但改用更简单的 **TCP + JSON** 协议（ZMQ 在 ESP32 上需要额外库，TCP 更轻量）。

---

## 3. PC 端实现：RemoteSO101 插件

### 3.1 包结构

```
lerobot-robot-remote-so101/
├── pyproject.toml                          # pip 包配置
└── lerobot_robot_remote_so101/             # 插件源码
    ├── __init__.py
    ├── config.py                           # RemoteSO101Config
    └── remote_so101.py                     # RemoteSO101 Robot 实现
```

### 3.2 pyproject.toml

```toml
[project]
name = "lerobot_robot_remote_so101"
version = "0.1.0"
description = "Remote SO101 robot plugin for LeRobot via ESP32 TCP bridge"
requires-python = ">=3.10"
dependencies = [
    "lerobot>=0.5.0",
    "numpy>=2.0.0",
]

[build-system]
requires = ["setuptools>=61.0"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
where = ["."]
include = ["lerobot_robot_remote_so101*"]
```

**关键**：包名 `lerobot_robot_remote_so101` 前缀匹配 `lerobot_robot_`，满足自动发现条件。

### 3.3 配置类：config.py

```python
from dataclasses import dataclass, field
from pathlib import Path
from lerobot.robots.config import RobotConfig
from lerobot.cameras import CameraConfig


@RobotConfig.register_subclass("remote_so101")
@dataclass
class RemoteSO101Config(RobotConfig):
    remote_ip: str = "192.168.4.1"
    port_cmd: int = 8888          # PC → ESP32（发送 action）
    port_obs: int = 8889          # ESP32 → PC（回传 observation）
    timeout_s: float = 5.0
    use_degrees: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    calibration_dir: Path | None = field(default=None, repr=False)
```

`@RobotConfig.register_subclass("remote_so101")` 将配置注册到 LeRobot 的配置注册表中。用户在 CLI 中通过 `--robot.type=remote_so101` 即可选中。

### 3.4 Robot 实现：remote_so101.py

```python
class RemoteSO101(Robot):
    config_class = RemoteSO101Config
    name = "remote_so101"

    JOINTS = [
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper",
    ]

    def __init__(self, config: RemoteSO101Config):
        # 复用 SO101/SO100 的校准目录
        if config.calibration_dir is None:
            config.calibration_dir = HF_LEROBOT_CALIBRATION / ROBOTS / "so_follower"
        super().__init__(config)
        ...
```

**关键设计决策 1：校准目录复用**

SO101 和 SO100 在 LeRobot 中共享 `so_follower` 名称，校准文件默认存储在：

```
~/.cache/huggingface/lerobot/calibration/robots/so_follower/{robot_id}.json
```

`RemoteSO101` 直接指向这个目录，用户无需复制或重新生成校准文件。

**关键设计决策 2：反归一化在 PC 端完成**

LeRobot 数据集存储的是**归一化后的动作值**（默认 `RANGE_M100_100`，即 float ∈ [-100, 100]）。PC 端的 `_denormalize()` 方法将归一化值转换为原生步进值（0-4095），然后发送给 ESP32：

```python
def _denormalize(self, action: RobotAction) -> dict[str, int]:
    native: dict[str, int] = {}
    for joint in self.JOINTS:
        key = f"{joint}.pos"
        norm_val = float(action[key])
        cal = self.calibration[joint]
        min_, max_ = int(cal.range_min), int(cal.range_max)
        drive_mode = int(cal.drive_mode)

        # RANGE_M100_100 反归一化公式
        val = -norm_val if drive_mode else norm_val
        bounded = min(100.0, max(-100.0, val))
        native_val = int(((bounded + 100.0) / 200.0) * (max_ - min_) + min_)
        native[joint] = int(np.clip(native_val, min_, max_))
    return native
```

这个公式与 LeRobot 内部 `SerialMotorsBus._unnormalize()` 完全一致，确保行为与本地 SO101 完全相同。

**关键设计决策 3：双 TCP 连接**

- `port_cmd (8888)`：PC → ESP32，发送 `set_positions` 和 `get_obs` 命令
- `port_obs (8889)`：ESP32 → PC，回传当前关节位置观测值

分离命令和观测通道，避免读写冲突。

### 3.5 通信协议（PC ↔ ESP32）

```json
// PC → ESP32 (port 8888)
{"cmd":"set_positions","positions":{"shoulder_pan":2048,"shoulder_lift":1024,...}}
{"cmd":"get_obs"}

// ESP32 → PC (port 8889)
{"shoulder_pan":2048,"shoulder_lift":1024,"elbow_flex":3072,
 "wrist_flex":1500,"wrist_roll":2000,"gripper":100}
```

---

## 4. ESP32 端实现：TCP → UART 桥接固件

### 4.1 硬件连接

```
ESP32 GPIO17 (TX) ──▶ Waveshare RX
ESP32 GPIO18 (RX) ◀── Waveshare TX
ESP32 GND    ──────── Waveshare GND
Waveshare 5V ──────── 6× STS3215 舵机总线
```

### 4.2 Feetech Protocol 0 协议

STS3215 使用 Feetech Protocol 0，关键参数：
- **波特率**：1 Mbps
- **数据位**：8，停止位：1，校验：无
- **指令码**：`INST_WRITE = 0x03`, `INST_READ = 0x02`, `INST_SYNC_WRITE = 0x83`
- **关键寄存器地址**：
  - `ADDR_TORQUE_EN = 40`
  - `ADDR_GOAL_POS = 42`
  - `ADDR_PRESENT_POS = 56`

### 4.3 SYNC_WRITE：批量写目标位置

最高效的指令方式，一次性更新所有 6 个舵机的 `Goal_Position`：

```cpp
void syncWritePositions(const uint16_t positions[]) {
  uint8_t dataLen = 2;  // 每个舵机 2 字节
  uint8_t paramCnt = 2 + NUM_MOTORS * (1 + dataLen);
  uint8_t length = paramCnt + 2;

  uint8_t pkt[40];
  uint8_t idx = 0;
  pkt[idx++] = 0xFF;           // Header 1
  pkt[idx++] = 0xFF;           // Header 2
  pkt[idx++] = 0xFE;           // Broadcast ID
  pkt[idx++] = length;
  pkt[idx++] = INST_SYNC_WRITE; // 0x83
  pkt[idx++] = ADDR_GOAL_POS;   // 42
  pkt[idx++] = dataLen;         // 2

  for (int i = 0; i < NUM_MOTORS; i++) {
    pkt[idx++] = MOTOR_IDS[i];           // 舵机 ID
    pkt[idx++] = positions[i] & 0xFF;    // 低字节
    pkt[idx++] = positions[i] >> 8;      // 高字节
  }

  pkt[idx] = calcChecksum(&pkt[2], 3 + paramCnt);
  SERVO_SERIAL.write(pkt, idx + 1);
}
```

### 4.4 舵机初始化配置

这是最容易遗漏的部分。LeRobot 本地 SO101 在 `connect()` 后会自动调用 `configure()` 设置 PID、加速度和模式。ESP32 固件也必须做同样的配置：

```cpp
void configureMotors() {
  // 先关扭矩（才能写 EPROM 区域）
  for (int i = 0; i < NUM_MOTORS; i++) {
    writeByte(MOTOR_IDS[i], ADDR_TORQUE_EN, 0);
    writeByte(MOTOR_IDS[i], 55, 0);  // Lock = 0
  }
  delay(50);

  // 通用配置（所有关节）
  for (int i = 0; i < NUM_MOTORS; i++) {
    uint8_t id = MOTOR_IDS[i];
    writeByte(id, 7, 0);      // Return_Delay_Time = 0
    writeByte(id, 33, 0);     // Operating_Mode = POSITION
    writeByte(id, 21, 16);    // P_Coefficient = 16（降低抖动）
    writeByte(id, 23, 0);     // I_Coefficient = 0
    writeByte(id, 22, 32);    // D_Coefficient = 32
    writeByte(id, 41, 254);   // Acceleration = 254
    writeByte(id, 85, 254);   // Maximum_Acceleration = 254
  }

  // Gripper 特殊保护（防止夹持时烧电机）
  uint8_t gripperId = MOTOR_IDS[5];
  writeWord(gripperId, 16, 500);   // Max_Torque_Limit = 50%
  writeWord(gripperId, 28, 250);   // Protection_Current = 50%
  writeByte(gripperId, 36, 25);    // Overload_Torque = 25%

  delay(50);

  // 重新使能扭矩
  for (int i = 0; i < NUM_MOTORS; i++) {
    writeByte(MOTOR_IDS[i], ADDR_TORQUE_EN, 1);
  }
}
```

### 4.5 上电自检

固件在 `setup()` 完成后会自动执行 `testAllJoints()`，帮助用户快速验证硬件：

```
[TEST] Step 1/3: Moving to CENTER (2048)...
[TEST] Step 2/3: Moving to LOW (1024)...
[TEST] Step 3/3: Moving to HIGH (3072)...
[TEST] Returning to CENTER...
[TEST] Self-test complete.
```

如果 6 个关节都大幅度运动，说明硬件和固件都正常。如果只有抓夹动，说明**电源功率不足**。

---

## 5. 关键设计：反归一化（Denormalisation）

### 5.1 为什么需要反归一化？

LeRobot 数据集为了统一不同机械臂的动作空间，会将关节角度**归一化**后存储：

| 模式 | 存储范围 | 说明 |
|------|----------|------|
| `RANGE_M100_100` | [-100, 100] | 默认，线性映射到关节行程 |
| `DEGREES` | 角度值 | 使用实际角度 |

而 Feetech STS3215 舵机的原生值是 **0-4095** 的步进值。因此 PC 端必须将归一化值**反转换**为原生步进值，ESP32 才能直接写入舵机。

### 5.2 反归一化公式

对于 `RANGE_M100_100` 模式（SO101 默认）：

```
native = ((norm_val + 100) / 200) * (range_max - range_min) + range_min
```

其中 `range_min` 和 `range_max` 来自校准文件，每个关节可能不同。

### 5.3 校准文件的作用

校准文件（`{robot_id}.json`）记录了每个关节的：
- `homing_offset`：零位偏移
- `range_min` / `range_max`：关节行程边界
- `drive_mode`：是否反转方向

`RemoteSO101` 直接读取这些值，确保反归一化后的步进值与本地 SO101 完全相同。

---

## 6. 完整安装与运行流程

### 6.1 前置条件

- Python 3.10+
- LeRobot 已安装 (`pip install -e .` 在 `lerobot-smolvla` 目录)
- Arduino IDE + ESP32 开发板支持
- ESP32 开发板 + Waveshare Feetech 控制板 + 6× STS3215 舵机
- 5V/5A 或更大电流的电源（**关键！**）

### 6.2 安装 Python 插件

```bash
# 1. 克隆本仓库
cd D:\work\lerobot-workspace
git clone <repo-url> lerobot-robot-remote-so101
cd lerobot-robot-remote-so101

# 2. 安装到当前环境（开发模式）
pip install -e .
```

安装完成后，LeRobot 的 `register_third_party_plugins()` 会自动发现此包。

### 6.3 准备校准文件

确保 SO101 的校准文件已存在：

```
Windows: %USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\{robot_id}.json
Linux:   ~/.cache/huggingface/lerobot/calibration/robots/so_follower/{robot_id}.json
```

如果还没有，先用本地 SO101 运行一次校准：

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=COM4 --robot.id=black
```

### 6.4 烧录 ESP32

1. 打开 `firmware/esp32_so101_bridge/esp32_so101_bridge.ino`
2. 修改 WiFi 凭证：
   ```cpp
   const char* WIFI_SSID     = "你的WiFi名";
   const char* WIFI_PASSWORD = "你的WiFi密码";
   ```
3. 选择 ESP32 开发板和端口，点击 **上传**
4. 上传完成后**按 RST 复位键**
5. 打开串口监视器（115200），观察输出：
   ```
   [UART] Serial2 initialized at 1 Mbps
   [SERVO] Torque enabled on all motors
   [SERVO] Motor configuration done (P=16, I=0, D=32, Acc=254)
   [TEST] Step 1/3: Moving to CENTER (2048)...   ← 观察机械臂是否运动
   [WiFi] Connected
   [WiFi] IP: 192.168.x.xxx
   ```

### 6.5 运行 lerobot-replay

```powershell
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.xxx `
  --robot.id=black `
  --dataset.repo_id=your_username/your_dataset `
  --dataset.episode=0
```

参数说明：
- `--robot.type=remote_so101`：选中我们的远程机器人插件
- `--robot.remote_ip`：ESP32 的 IP（从串口输出中获取）
- `--robot.id`：必须与校准文件名一致（如 `black.json`）

---

## 7. 卸载方法：零残留回退

由于整个实现是**独立 pip 包**，卸载即可完全回退到原始状态，LeRobot 源码完全不受影响。

```bash
pip uninstall lerobot_robot_remote_so101
```

验证回退：

```bash
# 确认插件已移除
python -c "import lerobot_robot_remote_so101"
# 应该报错：ModuleNotFoundError

# 确认 lerobot-replay 仍能用内置机器人
lerobot-replay --robot.type=so101_follower --help
```

---

## 8. Git-Flow 分支管理

本项目采用 **Git-Flow** 工作流，便于随时回退和追踪变更：

```
main                 -- 稳定发布
  └── develop        -- 集成分支
        └── feature/remote-so101-replay   -- 当前功能开发分支
```

### Commit 历史

```bash
git log --oneline
# cd506d6 feat(firmware): add automatic joint self-test after boot
# 7f793a0 fix(firmware): prevent stack corruption - static TX buffers, flush RX
# 02f0604 fix(firmware): add motor configure() to match SO101 defaults
# 72712d3 fix(firmware): add AP mode support; document Chinese SSID issue
# 215c293 feat(firmware): add diagnostic sketches and troubleshooting guide
# 40b397d docs: add README with setup and usage instructions
# efb7a65 feat(firmware): add ESP32 SO101 bridge with Feetech Protocol 0
# 3964480 feat(remote-so101): add RemoteSO101 robot plugin with TCP bridge
```

### 切换分支

```bash
# 查看当前功能分支
git checkout feature/remote-so101-replay

# 完成功能后合并到 develop
git checkout develop
git merge feature/remote-so101-replay

# 创建发布分支
git checkout -b release/v0.1.0

# 发布到 main
git checkout main
git merge release/v0.1.0
git tag v0.1.0
```

---

## 9. 踩坑记录与解决方案

### 9.1 问题一：ESP32 串口无输出

**现象**：烧录后串口监视器完全空白，没有任何输出。

**排查过程**：
1. 确认波特率是否为 115200（Arduino IDE 默认 9600）
2. 确认是否按了 RST 复位键（ESP32 烧录后不会自动重启）
3. 确认 USB 线是否为数据线（而非仅充电线）

**根因**：波特率不匹配，或未按 RST。

**解决方案**：在仓库中添加了 `firmware/test_serial_only/test_serial_only.ino` 作为最小验证固件，只输出 `Alive: X` 每秒递增，用于确认硬件和串口是否正常。

---

### 9.2 问题二：WiFi 连接超时（中文 SSID）

**现象**：`[DIAG] Connecting to WiFi...` 无限循环，点号不断增加。

**根因**：WiFi SSID 包含中文字符，ESP32 Arduino 核心对非 ASCII SSID 的编码处理不完善，导致 `WiFi.begin()` 永远匹配失败。

**解决方案**：
1. **短期**：将路由器 SSID 改为英文名
2. **长期**：固件中添加了 AP 模式支持（`#define USE_AP_MODE`），ESP32 自己开热点，完全绕过中文 SSID 问题

---

### 9.3 问题三：机械臂"好像没完成"动作

**现象**：TCP 通信正常，`SYNC_WRITE` 已发送，但关节运动缓慢或不到位。

**根因**：ESP32 固件只启用了扭矩，**完全跳过了舵机初始化配置**。SO101 本地连接后会自动设置 PID（P=16, I=0, D=32）、加速度（254）、工作模式（POSITION）等，缺少这些配置时舵机用出厂默认值运行，可能非常慢或抖动。

**解决方案**：在 `setup()` 中添加 `configureMotors()`，完整复刻 SO101 的初始化逻辑：

```cpp
void configureMotors() {
  // P=16, I=0, D=32
  // Acceleration = 254
  // Operating_Mode = POSITION
  // Gripper 扭矩保护
}
```

---

### 9.4 问题四：ESP32 Guru Meditation Error（崩溃）

**现象**：`configureMotors()` 执行后，系统崩溃，输出 `Core 1 panic'ed (LoadProhibited)` 和 `Backtrace: ...CORRUPTED`。

**根因**：**栈溢出（Stack Corruption）**。`configureMotors()` 发送了 20+ 个单舵机写命令，每个舵机都回复一个 8 字节状态包。这些回复积压在 Serial2 的 RX 缓冲区里没有被读取。后续 `readPosition()` 调用时：

```cpp
uint8_t buf[16];
int n = SERVO_SERIAL.readBytes(buf, SERVO_SERIAL.available());
// available() 可能返回 80+，全部塞进 16 字节的 buf → 缓冲区溢出
```

80+ 字节的数据被强行写进 16 字节的栈数组，导致**栈损坏**，后续任意函数返回时触发崩溃。

**解决方案**（三处修复）：

1. **writeByte/writeWord 使用 static buffer**：
   ```cpp
   static uint8_t pkt[8];  // 全局静态，避免 DMA 读取已释放的栈内存
   SERVO_SERIAL.write(pkt, 8);
   SERVO_SERIAL.flush();   // 等待发送完成
   ```

2. **configureMotors 结束后清空 RX**：
   ```cpp
   delay(20);
   while (SERVO_SERIAL.available()) { SERVO_SERIAL.read(); }
   ```

3. **readPosition 防溢出读取**：
   ```cpp
   int toRead = min(SERVO_SERIAL.available(), 8);
   int n = SERVO_SERIAL.readBytes(buf, toRead);  // 最多读 8 字节
   ```

---

### 9.5 问题五：只有抓夹动，其他关节不动

**现象**：自检或 replay 时，只有 gripper 关节轻微运动，其他关节完全不动。

**排查过程**：
1. 观察自检输出，所有 `SYNC_WRITE` 都发送成功
2. 用手推 shoulder_lift 关节，感觉**很硬**（说明扭矩已使能）
3. 但关节不运动

**根因**：**电源功率不足**。6 个 STS3215 舵机同时启动时峰值电流可达 **3-6A**。如果电源只有 1-2A（如电脑 USB 口），电压会在 SYNC_WRITE 瞬间从 5V 跌落到 3V 以下，舵机根本无法启动。抓夹负载最轻、电流最小，所以能动。

**解决方案**：
- 换用 **5V/5A 或更大电流** 的开关电源（建议 5V/10A）
- 为每个关节单独供电（并联多个电源）

---

## 10. 仓库文件清单

```
lerobot-robot-remote-so101/
├── .gitignore
├── README.md                                    # 快速参考文档
├── pyproject.toml                               # Python 包配置
├── docs/
│   └── guide.md                                 # 本完整指南
├── lerobot_robot_remote_so101/                  # Python 插件源码
│   ├── __init__.py
│   ├── config.py                                # RemoteSO101Config
│   └── remote_so101.py                          # RemoteSO101 Robot
└── firmware/
    ├── esp32_so101_bridge/
    │   └── esp32_so101_bridge.ino               # 正式版 ESP32 固件
    ├── esp32_so101_bridge_diag/
    │   └── esp32_so101_bridge_diag.ino          # 诊断增强版固件
    └── test_serial_only/
        └── test_serial_only.ino                 # 最小串口测试固件
```

---

## 附录 A：PC 端完整代码

### `lerobot_robot_remote_so101/config.py`

```python
from dataclasses import dataclass, field
from pathlib import Path
from lerobot.robots.config import RobotConfig
from lerobot.cameras import CameraConfig


@RobotConfig.register_subclass("remote_so101")
@dataclass
class RemoteSO101Config(RobotConfig):
    remote_ip: str = "192.168.4.1"
    port_cmd: int = 8888
    port_obs: int = 8889
    timeout_s: float = 5.0
    use_degrees: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    calibration_dir: Path | None = field(default=None, repr=False)
```

### `lerobot_robot_remote_so101/remote_so101.py`

```python
import json
import logging
import socket

import numpy as np
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import RemoteSO101Config

logger = logging.getLogger(__name__)


class RemoteSO101(Robot):
    config_class = RemoteSO101Config
    name = "remote_so101"

    JOINTS = [
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper",
    ]

    def __init__(self, config: RemoteSO101Config):
        if config.calibration_dir is None:
            config.calibration_dir = HF_LEROBOT_CALIBRATION / ROBOTS / "so_follower"
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._cmd_sock = None
        self._obs_sock = None
        self._last_obs = {}

    @property
    def observation_features(self):
        return {f"{j}.pos": float for j in self.JOINTS}

    @property
    def action_features(self):
        return self.observation_features

    @property
    def is_connected(self):
        return self._is_connected

    @property
    def is_calibrated(self):
        return bool(self.calibration)

    def calibrate(self):
        raise NotImplementedError("RemoteSO101 does not support calibration.")

    def configure(self):
        pass

    @check_if_already_connected
    def connect(self, calibrate=True):
        if not self.is_calibrated and calibrate:
            raise RuntimeError(f"No calibration file found. Expected: {self.calibration_fpath}")

        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._cmd_sock.settimeout(self.config.timeout_s)
        self._cmd_sock.connect((self.config.remote_ip, self.config.port_cmd))

        self._obs_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._obs_sock.settimeout(self.config.timeout_s)
        self._obs_sock.connect((self.config.remote_ip, self.config.port_obs))

        self._is_connected = True
        logger.info(f"RemoteSO101 connected to {self.config.remote_ip}")

    def _denormalize(self, action: RobotAction) -> dict[str, int]:
        if not self.calibration:
            raise RuntimeError("Cannot denormalise without calibration data.")

        native = {}
        for joint in self.JOINTS:
            key = f"{joint}.pos"
            norm_val = float(action[key])
            cal = self.calibration[joint]
            min_ = int(cal.range_min)
            max_ = int(cal.range_max)
            drive_mode = int(cal.drive_mode)

            if self.config.use_degrees:
                mid = (min_ + max_) / 2.0
                max_res = 4095
                native_val = int((norm_val * max_res / 360.0) + mid)
            else:
                val = -norm_val if drive_mode else norm_val
                bounded = min(100.0, max(-100.0, val))
                native_val = int(((bounded + 100.0) / 200.0) * (max_ - min_) + min_)

            native[joint] = int(np.clip(native_val, min_, max_))
        return native

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        native_positions = self._denormalize(action)
        payload = {"cmd": "set_positions", "positions": native_positions}
        msg = json.dumps(payload, separators=(",", ":")) + "\n"
        self._cmd_sock.sendall(msg.encode())
        return action

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        try:
            request = json.dumps({"cmd": "get_obs"}, separators=(",", ":")) + "\n"
            self._cmd_sock.sendall(request.encode())
            buf = b""
            while b"\n" not in buf:
                chunk = self._obs_sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
            data = json.loads(buf.decode().strip())
            self._last_obs = {f"{k}.pos": float(v) for k, v in data.items()}
        except Exception as exc:
            logger.warning(f"Observation read failed: {exc}")
        return self._last_obs

    @check_if_not_connected
    def disconnect(self):
        if self._cmd_sock:
            self._cmd_sock.close()
        if self._obs_sock:
            self._obs_sock.close()
        self._is_connected = False
        logger.info("RemoteSO101 disconnected.")
```

---

## 附录 B：ESP32 固件核心代码（正式版）

由于完整固件较长（~400 行），此处只展示核心片段。完整代码见仓库 `firmware/esp32_so101_bridge/esp32_so101_bridge.ino`。

```cpp
// WiFi 配置（Station 模式）
const char* WIFI_SSID     = "你的WiFi名";
const char* WIFI_PASSWORD = "你的WiFi密码";

// 双 TCP 端口
const int PORT_CMD = 8888;
const int PORT_OBS = 8889;

// UART 配置（连接 Waveshare 控制板）
#define SERVO_SERIAL Serial2
#define SERVO_TX_PIN  17
#define SERVO_RX_PIN  18
#define SERVO_BAUD    1000000

// 6 个舵机
const uint8_t MOTOR_IDS[] = {1, 2, 3, 4, 5, 6};
const char*   MOTOR_NAMES[] = {
  "shoulder_pan", "shoulder_lift", "elbow_flex",
  "wrist_flex", "wrist_roll", "gripper"
};

// Feetech Protocol 0
#define INST_WRITE       0x03
#define INST_SYNC_WRITE  0x83
#define ADDR_TORQUE_EN   40
#define ADDR_GOAL_POS    42
#define ADDR_PRESENT_POS 56

// SYNC_WRITE 批量写位置
void syncWritePositions(const uint16_t positions[]) {
  // ... 实现见仓库 ...
}

// 舵机初始化（复刻 SO101 配置）
void configureMotors() {
  // P=16, I=0, D=32, Acc=254
  // Gripper 扭矩保护
  // ... 实现见仓库 ...
}

// setup() 中依次执行：
// 1. Serial.begin(115200)
// 2. SERVO_SERIAL.begin(1000000, SERIAL_8N1, 18, 17)
// 3. enableAllTorque()
// 4. configureMotors()
// 5. testAllJoints()  // 自检
// 6. WiFi.begin() / WiFi.softAP()
// 7. serverCmd.begin(8888); serverObs.begin(8889)
```

---

**文档结束**。如有疑问或需要补充其他章节，请提出。
