# LeRobot 远程控制完整指南

> **目标**：在不修改 LeRobot 源码的前提下，通过 WiFi 远程控制 SO101 机械臂，支持 replay、teleoperation 和双臂遥操作。
>
> **适用硬件**：LeRobot SO101 机械臂 + 微雪(Waveshare) Feetech 控制板 + ESP32 开发板 + ST7789 显示屏（可选）
>
> **完成时间**：2026-05-05

---

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构](#2-系统架构)
3. [PC 端插件](#3-pc-端插件)
4. [ESP32 固件](#4-esp32-固件)
5. [ST7789 显示屏](#5-st7789-显示屏)
6. [安装与使用](#6-安装与使用)
7. [性能优化](#7-性能优化)
8. [常见问题](#8-常见问题)
9. [仓库结构](#9-仓库结构)

---

## 1. 项目概述

### 1.1 功能特性

| 功能 | 说明 |
|------|------|
| **远程 Replay** | 通过 WiFi 回放数据集，控制远端机械臂 |
| **键盘/手柄遥操作** | 使用 keyboard/gamepad 控制远端机械臂 |
| **双臂遥操作** | 两块 ESP32 分别连接 Leader 和 Follower，实现无线主从控制 |
| **实时显示** | ST7789 显示屏显示 IP、连接状态、收发消息 |
| **零源码修改** | 通过 LeRobot 第三方插件机制注入，不修改 LeRobot 源码 |

### 1.2 版本演进

| 版本 | 特性 |
|------|------|
| v1 | 基础 TCP 桥接，单舵机读写 |
| v2 | syncWrite/syncRead 优化，TCP_NODELAY，~5ms 延迟 |
| v3 | 双模式固件（Leader/Follower），ST7789 显示支持 |

---

## 2. 系统架构

### 2.1 单臂模式（Follower）

```
PC (LeRobot)                    ESP32                      机械臂
├─ RemoteSO101 ──────WiFi/TCP──►├─ TCP Server (8888/8889)──►├─ Waveshare
│  ├─ send_action()    JSON      │  ├─ syncWrite()            │  └─ 6× STS3215
│  └─ get_obs() ◄───────────────┤  └─ syncRead()
```

### 2.2 双臂模式（Leader + Follower）

```
                    PC (LeRobot)
    ┌──────────────────┐  ┌──────────────────┐
    │ RemoteSO101Leader│  │   RemoteSO101    │
    │   (Teleoperator) │  │     (Robot)      │
    └────────┬─────────┘  └────────┬─────────┘
             │ WiFi (8889)          │ WiFi (8888)
             ▼                      ▼
    ┌──────────────────┐  ┌──────────────────┐
    │   ESP32 #1       │  │   ESP32 #2       │
    │   LEADER_MODE    │  │  FOLLOWER_MODE   │
    │   torque=OFF     │  │   torque=ON      │
    └────────┬─────────┘  └────────┬─────────┘
             │ UART                 │ UART
             ▼                      ▼
    ┌──────────────────┐  ┌──────────────────┐
    │   SO101 Leader   │  │  SO101 Follower  │
    └──────────────────┘  └──────────────────┘
```

---

## 3. PC 端插件

### 3.1 插件自动发现

LeRobot 启动时自动 import 所有 `lerobot_robot_*` 前缀的包：

```python
# register_third_party_plugins()
prefixes = ("lerobot_robot_", "lerobot_camera_", 
            "lerobot_teleoperator_", "lerobot_policy_")
```

### 3.2 文件结构

```
lerobot_robot_remote_so101/
├── __init__.py                    # 导出所有类
├── config.py                      # RemoteSO101Config + RemoteSO101LeaderConfig
├── remote_so101.py                # RemoteSO101 (Robot, Follower)
└── remote_so101_leader.py         # RemoteSO101Leader (Teleoperator, Leader)
```

### 3.3 RemoteSO101（Follower）

```python
@RobotConfig.register_subclass("remote_so101")
class RemoteSO101Config(RobotConfig):
    remote_ip: str = "192.168.4.1"
    port_cmd: int = 8888
    port_obs: int = 8889
    timeout_s: float = 5.0
    use_degrees: bool = False
    skip_observation: bool = False  # v2 新增：跳过观测读取
```

**关键特性**：
- 复用 SO101 校准文件（`so_follower/{id}.json`）
- PC 端完成反归一化（归一化值 → 原生步进值 0-4095）
- 双 TCP 连接：8888（命令）、8889（观测）
- `skip_observation=true`：replay 时跳过观测读取，消除 RTT 延迟

### 3.4 RemoteSO101Leader（Leader）

```python
@TeleoperatorConfig.register_subclass("remote_so101_leader")
class RemoteSO101LeaderConfig(TeleoperatorConfig):
    remote_ip: str = "192.168.4.2"
    port_obs: int = 8889
    timeout_s: float = 5.0
    use_degrees: bool = False
```

**关键特性**：
- 后台线程持续接收 Leader ESP32 的位置数据流
- `get_action()` 非阻塞，返回最新位置
- 使用 `so_follower` 校准文件（与 Follower 共用）

### 3.5 使用命令

```powershell
# 远程 replay
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.1.100 `
  --robot.id=black `
  --dataset.repo_id=your/dataset `
  --dataset.episode=0

# 键盘遥操作
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.1.100 `
  --robot.id=black `
  --teleop.type=keyboard

# 双臂遥操作（Leader + Follower）
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.1.102 `     # Follower
  --robot.id=black `
  --teleop.type=remote_so101_leader `
  --teleop.remote_ip=192.168.1.101      # Leader
```

---

## 4. ESP32 固件

### 4.1 双模式切换

通过编译开关选择模式：

```cpp
// 选择模式（二选一）
#define LEADER_MODE
// #define FOLLOWER_MODE
```

| 模式 | 扭矩 | 行为 |
|------|------|------|
| **Leader** | OFF | 读取位置，通过 8889 端口流式发送给 PC |
| **Follower** | ON | 接收 8888 端口的 set_positions，执行动作 |

### 4.2 硬件连接

**Servo UART**：
```
ESP32 GPIO17 (TX) ──► Waveshare RX
ESP32 GPIO18 (RX) ◀── Waveshare TX
ESP32 GND    ──────── Waveshare GND
```

**ST7789 显示屏**（J1 左侧）：
```
VCC  → 3.3V    GND  → GND
BLK  → GPIO5   DC   → GPIO6
RES  → GPIO7   CS2  → GPIO9
CS1  → GPIO10  SDA  → GPIO11
SCL  → GPIO12  FSO  → GPIO13
```

### 4.3 核心优化（v2/v3）

| 优化点 | v1 | v2/v3 |
|--------|-----|-------|
| 写位置 | 6× 单舵机写入 (~12ms) | **syncWrite** 单包广播 (~1ms) |
| 读位置 | 6× 单舵机读取 (~30ms) | **syncRead** 单包读取 (~5ms) |
| TCP 延迟 | Nagle 缓冲 | `TCP_NODELAY` 立即发送 |
| 加速度 | 默认值 | `ACC=254` 最大加速度 |

### 4.4 通信协议

```json
// PC → ESP32 (8888)
{"cmd":"set_positions","positions":{"shoulder_pan":2048,...}}
{"cmd":"get_obs"}
{"cmd":"release_torque"}

// ESP32 → PC (8889)
{"shoulder_pan":2048,"shoulder_lift":1024,"elbow_flex":3072,...}
```

---

## 5. ST7789 显示屏

### 5.1 显示布局

```
┌──────────────────────────────┐
│ SO101 [LEADER]               │  Y: 0-20  标题（固定）
│ IP: 192.168.1.105            │  Y: 20-40 IP（固定）
│ Status: PC Connected         │  Y: 40-60 状态（固定）
│ ─────────────────────────────│  Y: 60-65 分隔线
│ RX: set_positions            │  Y: 65+   消息日志（滚动）
│ TX: positions
│ CMD: 192.168.1.100
│ ...
└──────────────────────────────┘
```

### 5.2 显示内容

| 阶段 | 显示 |
|------|------|
| 启动 | "SO101 Bridge v2"、"UART2 OK"、"Scan servos..." |
| WiFi | "WiFi: oppowifi" → "IP: 192.168.x.x" |
| TCP 连接 | "CMD: 192.168.x.x"、"OBS: 192.168.x.x" |
| 运行时 | "RX: set_positions"、"TX: positions" |
| 断开 | "CMD disconnected"、"Disable torque..." |

### 5.3 技术实现

使用 **TFT_eSPI** 库：
- 支持 ESP32-S3 DMA，刷新速度提升 5-10 倍
- 项目级 `User_Setup.h` 配置
- SPI 事务支持，与字库芯片分时片选

```cpp
// User_Setup.h 关键配置
#define ST7789_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 240
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10
#define TFT_DC    6
#define TFT_RST   7
#define TFT_MISO 13
#define TFT_BL    5
#define SPI_FREQUENCY  40000000
#define SPI_READ_FREQ   8000000
#define SUPPORT_TRANSACTIONS
```

---

## 6. 安装与使用

### 6.1 安装 Python 插件

```bash
git clone https://github.com/xiaobin86/lerobot-remote-plugin.git
cd lerobot-remote-plugin
pip install -e .
```

### 6.2 烧录 ESP32

1. 打开 `firmware/esp32_so101_bridge_v2/esp32_so101_bridge_v2.ino`
2. 修改 WiFi 凭证
3. 选择模式（`#define LEADER_MODE` 或 `#define FOLLOWER_MODE`）
4. 上传固件
5. 按 RST 复位

### 6.3 准备校准文件

确保 SO101 校准文件存在：

```
Windows: %USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\{robot_id}.json
```

### 6.4 运行

```powershell
# Replay
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.1.100 `
  --robot.id=black `
  --dataset.repo_id=your/dataset `
  --dataset.episode=0

# 双臂遥操作
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.1.102 `
  --robot.id=black `
  --teleop.type=remote_so101_leader `
  --teleop.remote_ip=192.168.1.101
```

---

## 7. 性能优化

### 7.1 syncWrite + syncRead

v2 固件使用 `syncWrite`/`syncRead` 替代单舵机读写：
- **syncWrite**：1 个数据包更新 6 个舵机（~1ms）
- **syncRead**：1 个往返读取 6 个舵机（~5ms）
- 对比 v1：写入从 ~12ms 降至 ~1ms，读取从 ~30ms 降至 ~5ms

### 7.2 TCP_NODELAY

ESP32 和 PC 端均启用 `TCP_NODELAY`：
- 禁用 Nagle 算法，小包立即发送
- 消除 TCP 缓冲导致的延迟

### 7.3 skip_observation

replay 场景可跳过观测读取：

```powershell
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.1.100 `
  --robot.skip_observation=true `    # 跳过观测
  ...
```

- 消除 ~5ms 观测 RTT
- 仅适用于 replay（动作已预计算）
- 不适用于 teleoperation（需要实时反馈）

---

## 8. 常见问题

### 8.1 ESP32 串口无输出

- 检查波特率是否为 115200
- 烧录后按 RST 复位键
- 确认 USB 线为数据线（非充电线）

### 8.2 WiFi 连接失败

- 检查 WiFi 凭证是否正确
- 中文 SSID 可能不兼容，改用 AP 模式
- 确认 ESP32 与 PC 在同一网络

### 8.3 舵机无响应

- 检查电源功率（6 个舵机峰值 3-6A，建议 5V/5A+）
- 确认 GPIO17/18 接线正确
- 检查舵机 ID 是否为 1-6

### 8.4 运动卡顿

- 确认使用 v2/v3 固件（syncWrite/syncRead）
- replay 时启用 `skip_observation=true`
- 检查 WiFi 信号强度

### 8.5 显示屏无显示

- 检查背光引脚（GPIO5）是否接高电平
- 确认 `User_Setup.h` 引脚配置与实际接线一致
- 检查颜色顺序（`TFT_RGB_ORDER` 尝试 `TFT_BGR`）

---

## 9. 仓库结构

```
lerobot-robot-remote-so101/
├── README.md                              # 快速参考
├── pyproject.toml                         # Python 包配置
├── docs/
│   └── guide.md                           # 本完整指南
├── lerobot_robot_remote_so101/            # Python 插件
│   ├── __init__.py
│   ├── config.py                          # 配置类
│   ├── remote_so101.py                    # Follower Robot
│   └── remote_so101_leader.py             # Leader Teleoperator
└── firmware/
    └── esp32_so101_bridge_v2/             # ESP32 固件 (v3)
        ├── esp32_so101_bridge_v2.ino      # 主固件（双模式）
        └── User_Setup.h                     # TFT_eSPI 配置
```

---

**文档结束**
