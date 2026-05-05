# 概念 - SO101 远程 replay 与 ESP32 桥接方案

> 方案目标：在不修改 LeRobot 源码的前提下，通过 `lerobot-replay` 和 `lerobot-teleoperate` 远程控制 ESP32 驱动的 SO101 机械臂。
> 
> 硬件环境：SO101 双臂（Leader: L07252802 / Follower: R12552802）+ 微雪控制板 + ESP32
> 软件环境：LeRobot 0.5.2 + Windows + RTX 5070 Ti

---

## 1. 为什么需要远程控制？

标准 `lerobot-replay` 通过 USB 串口直连机械臂控制板（微雪 / Feetech 官方板）。当控制板需要独立供电、或 PC 与机械臂距离较远时，USB 连线不便。通过 WiFi + ESP32 桥接，可以：

- **摆脱 USB 线束缚**：PC 与机械臂通过局域网通信
- **独立供电**：ESP32 + 控制板使用独立 5V 电源
- **多机部署**：一台 PC 可同时控制多台远端机械臂

---

## 2. 核心设计：零源码修改的扩展机制

### 2.1 LeRobot 插件架构

LeRobot 的 `RobotConfig` 使用 `draccus.ChoiceRegistry`，通过装饰器自动注册子类：

```python
from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("remote_so101")
@dataclass
class RemoteSO101Config(RobotConfig):
    ...
```

`make_robot_from_config()` 在运行时通过类型名自动实例化对应的 `Robot` 子类。

### 2.2 第三方插件自动加载

LeRobot 启动时会调用 `register_third_party_plugins()`，**自动 import** 所有以 `lerobot_robot_` 为前缀的 pip 包。这意味着：

- 不需要修改 LeRobot 源码仓库的任何文件
- 安装独立 pip 包后，`lerobot-replay --robot.type=remote_so101` 即可识别
- 卸载 pip 包即可完全回退到原始状态

### 2.3 参考实现：LeKiwiClient

LeRobot 官方已验证远程机器人模式：`LeKiwiClient`（`src/lerobot/robots/lekiwi/lekiwi_client.py`）通过 **ZMQ** 与远端主机通信。本方案采用更简单的 **TCP + JSON** 协议，降低 ESP32 端实现复杂度。

---

## 3. 系统架构

```
┌─────────────────────────────────────┐
│  PC (Windows)                        │
│  ┌───────────────────────────────┐  │
│  │ lerobot-replay                │  │
│  │  └─ RemoteSO101 (Robot 子类)  │  │
│  │      ├─ _denormalize()        │  │
│  │      │   (归一化 → 原生步进值)  │  │
│  │      ├─ send_action() ────────┼──┼──► TCP:8888
│  │      └─ get_observation() ◄───┼──┼─── TCP:8889
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘
              WiFi
┌─────────────────────────────────────┐
│  ESP32                               │
│  ┌───────────────────────────────┐  │
│  │ WiFiServer (8888 / 8889)      │  │
│  │  ├─ parse JSON cmd            │  │
│  │  ├─ syncWritePositions()      │  │  ← v2: syncWrite (1 packet)
│  │  └─ syncReadPositions()       │  │  ← v2: syncRead  (1 round-trip)
│  └───────────────────────────────┘  │
│              │ UART (1Mbps)          │
│              ▼ GPIO17/18             │
│  ┌───────────────────────────────┐  │
│  │ 微雪控制板 (Waveshare)         │  │
│  │  └─ Feetech STS3215 总线       │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘
```

---

## 4. 通信协议（PC ↔ ESP32）

### 4.1 传输层

- **TCP**：可靠、有序，适合控制指令
- **双端口设计**：
  - `8888`：PC → ESP32（发送 action）
  - `8889`：ESP32 → PC（回传 observation）

### 4.2 应用层：JSON 行协议（`\n` 结尾）

**PC → ESP32（控制指令）**

```json
{"cmd":"set_positions","positions":{"shoulder_pan":2048,"shoulder_lift":2048,...}}
```

**PC → ESP32（查询状态）**

```json
{"cmd":"get_obs"}
```

**ESP32 → PC（状态回传）**

```json
{"shoulder_pan":2048,"shoulder_lift":2048,"elbow_flex":2048,...}
```

### 4.3 为什么反归一化在 PC 端完成？

| 方案 | 反归一化位置 | 优点 | 缺点 |
|------|------------|------|------|
| A | PC 端 | ESP32 固件极简（只收原生步进值）；校准文件复用现有 SO101 校准 | PC 端需加载校准文件 |
| B | ESP32 端 | PC 端逻辑简单 | ESP32 需存储校准参数；更新校准需重新烧录 |

**选择方案 A**：PC 端通过 `calibration_dir` 加载现有 SO101 校准文件，将数据集中的归一化值 `[-1, 1]` 反归一化为原生步进值 `[0, 4095]`，ESP32 直接写入舵机。

---

## 5. 项目结构

```text
lerobot-robot-remote-so101/          ← 独立仓库（git-flow 管理）
├── feature/remote-so101-replay      ← 功能分支
├── lerobot_robot_remote_so101/      ← Python 插件源码
│   ├── __init__.py
│   ├── config.py                    ← RemoteSO101Config
│   └── remote_so101.py              ← RemoteSO101 Robot 子类
├── firmware/
│   └── esp32_so101_bridge_v2/
│       └── esp32_so101_bridge_v2.ino   ← ESP32 Arduino 固件 (v2 优化版)
└── pyproject.toml
```

**LeRobot 源码仓库 `lerobot-smolvla` 完全未修改**，随时可以 pristine 回退。

---

## 6. 三步使用

```bash
# 1. 安装 PC 端插件（LeRobot 源码完全不动）
cd D:\work\lerobot-workspace\lerobot-robot-remote-so101
pip install -e .

# 2. 烧录 ESP32（Arduino IDE 打开 firmware/esp32_so101_bridge_v2/esp32_so101_bridge_v2.ino，修改 WiFi 后上传）

# 3. 运行 replay
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.id=<your_so101_id> `
  --dataset.repo_id=<user>/<dataset> `
  --dataset.episode=0
```

## 7. v2 性能优化说明

### 7.1 问题诊断

初版固件使用 `WritePosEx` 逐 servo 发送 + `ReadPos` 逐 servo 读取，导致：
- `set_positions`：6 次串行写入，总线饱和
- `get_obs`：6 次串行读取 + 2 次 TCP 往返，单次 ~30–50 ms
- **结果**：replay 帧率被拖慢到 ~15–20 fps，动作明显卡顿

### 7.2 v2 优化手段

| 优化点 | v1 (初版) | v2 |
|--------|-----------|-----|
| **写位置** | 6× `WritePosEx` 逐个发送 (~2ms×6) | **`syncWrite`** 单包广播 6 个 servo (~1ms) |
| **读位置** | 6× `ReadPos` 逐个读取 (~5ms×6=30ms) | **`syncRead`** 单包读取 6 个 servo (~5ms) |
| **TCP 延迟** | Nagle 缓冲，前快后慢 | `setNoDelay(true)` + `TCP_NODELAY` 立即发送 |
| **ACC 设置** | 未配置（使用 servo 默认） | 启动时写入 `ACC=254`（最大加速度，瞬时响应） |

**效果**：`get_obs` 延迟从 ~30–50 ms 降至 ~5 ms，恢复 30 fps 流畅 replay。

### 7.3 兜底方案：skip_observation

如果 v2 固件仍无法满足实时性要求（如 WiFi 环境差、信号干扰大），PC 端插件提供 `skip_observation` 开关：

```powershell
lerobot-replay `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.skip_observation=true `
  ...
```

- **原理**：跳过 `get_observation()` 的远程读取，直接返回缓存值
- **适用场景**：仅 replay（动作已预计算，不需要实时反馈）
- **不适用场景**：teleoperation、closed-loop 控制（需要实时关节位置反馈）

## 8. Teleoperation 支持

### 8.1 使用方式

同一个插件同时支持 `lerobot-teleoperate`，将远程 SO101 作为 **follower** 使用：

```powershell
# 键盘遥操作
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.id=<your_so101_id> `
  --teleop.type=keyboard

# 手柄遥操作
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.id=<your_so101_id> `
  --teleop.type=gamepad

# 本地主臂 + 远程从臂
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.x.x `
  --robot.id=<your_so101_id> `
  --teleop.type=so100_leader `
  --teleop.port=COM3
```

### 8.2 与 Replay 的区别

| 特性 | Replay | Teleoperation |
|------|--------|---------------|
| **Action 来源** | 数据集预录制 | Teleoperator 实时输入 |
| **Observation** | 可选跳过（`skip_observation=true`） | **必须实时读取**（`skip_observation=false`） |
| **帧率要求** | 30 fps | 30–60 fps |
| **延迟容忍** | 较高（动作已预计算） | 较低（需要实时反馈） |

### 8.3 Teleop 性能保障

- v2 固件的 `syncRead` 将 observation 延迟控制在 **~5 ms**
- Python 插件默认 `skip_observation=false`，确保 teleop 循环获取实时关节位置
- TCP 断开自动检测：如果发送/接收失败，插件标记为断开并记录警告

---

## 9. 双模式：Leader + Follower（v3 固件）

### 9.1 设计目标

使用 **两块 ESP32** 分别连接 Leader 臂和 Follower 臂，实现**无线双臂遥操作**：
- **Leader ESP32**：读取主臂位置，通过 WiFi 实时发送给 PC
- **Follower ESP32**：接收 PC 指令，驱动从臂执行动作
- **同一份固件**：通过编译开关 `#define LEADER_MODE` / `#define FOLLOWER_MODE` 切换模式

### 9.2 系统架构

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

### 9.3 Python 插件（PC 端）

新增 `RemoteSO101Leader` Teleoperator 插件：

```python
# lerobot_robot_remote_so101/remote_so101_leader.py
class RemoteSO101Leader(Teleoperator):
    """远程 SO101 Leader：通过 WiFi 读取主臂位置"""
    name = "remote_so101_leader"
    
    def connect(self):
        # 连接 Leader ESP32 的 8889 端口
        # 后台线程持续接收位置数据流
        pass
    
    def get_action(self):
        # 非阻塞，返回最新接收到的位置
        return self._latest_action
```

配置类：

```python
@TeleoperatorConfig.register_subclass("remote_so101_leader")
@dataclass
class RemoteSO101LeaderConfig(TeleoperatorConfig):
    remote_ip: str = "192.168.4.2"   # Leader ESP32 IP
    port_obs: int = 8889
    use_degrees: bool = False
```

### 9.4 使用方式

```powershell
# 1. 烧录两块 ESP32
#    ESP32 #1 (Leader): #define LEADER_MODE
#    ESP32 #2 (Follower): #define FOLLOWER_MODE

# 2. 运行双臂遥操作
lerobot-teleoperate `
  --robot.type=remote_so101 `
  --robot.remote_ip=192.168.1.102 `    # Follower
  --robot.id=black `
  --teleop.type=remote_so101_leader `
  --teleop.remote_ip=192.168.1.101     # Leader
```

**关键约束**：Leader 和 Follower 必须**共用同一份校准文件**（相同的 `--robot.id`），确保关节坐标系一致。

---

## 10. ST7789 显示屏支持

### 10.1 硬件接线

显示屏接 ESP32-S3 **J1 左侧**，与伺服 UART（GPIO17/18）**完全隔离**：

| 显示屏 | 功能 | GPIO | 说明 |
|--------|------|------|------|
| VCC | 电源 | 3.3V | J1-1 |
| GND | 地线 | GND | J1-22 |
| BLK | 背光 | GPIO5 | J1-5 |
| DC | 数据/命令 | GPIO6 | J1-6 |
| RES | 复位 | GPIO7 | J1-7 |
| CS2 | 字库片选 | GPIO9 | J1-15 |
| CS1 | 屏幕片选 | GPIO10 | J1-16 |
| SDA | SPI MOSI | GPIO11 | J1-17 |
| SCL | SPI CLK | GPIO12 | J1-18 |
| FSO | SPI MISO | GPIO13 | J1-19 |

### 10.2 显示布局（240×240）

```
┌──────────────────────────────┐
│ SO101 [LEADER]               │  ← 标题 (固定，青色)
│ IP: 192.168.1.105            │  ← IP 地址 (固定，黄色)
│ Status: PC Connected         │  ← 状态 (固定，绿色)
│ ─────────────────────────────│  ← 分隔线
│ RX: set_positions            │  ← 消息日志 (滚动，白色)
│ TX: positions
│ CMD: 192.168.1.100
│ ...
└──────────────────────────────┘
```

- **固定区域**（Y 0-60）：标题、IP、连接状态，刷新时局部重绘
- **滚动区域**（Y 65-240）：消息环形缓冲区（32 条），新消息追加，旧消息滚动

### 10.3 显示内容

| 阶段 | 显示内容 |
|------|---------|
| **启动** | "SO101 Bridge v2"、"UART2 OK"、"Scan servos..." |
| **WiFi 连接** | "WiFi: oppowifi" → "IP: 192.168.x.x" |
| **TCP 连接** | "CMD: 192.168.x.x"、"OBS: 192.168.x.x" |
| **运行时** | "RX: set_positions"、"TX: positions"、"Release torque..." |
| **断开** | "CMD disconnected"、"Disable torque..." |

### 10.4 技术实现

使用 **TFT_eSPI** 库（优于 Adafruit_ST7789）：
- 支持 ESP32-S3 DMA，刷新速度提升 5-10 倍
- 项目级 `User_Setup.h` 配置，不影响其他项目
- SPI 事务支持（`SUPPORT_TRANSACTIONS`），与字库芯片分时片选

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

## 11. 固件代码详解（`esp32_so101_bridge_v2.ino`）

以下逐行解释 ESP32 固件的核心代码，帮助理解其工作原理和修改方法。

### 11.1 编译时模式切换

```cpp
/**
 * ESP32 SO101 Remote Bridge v3 — Bimanual Leader/Follower Dual Mode + Display
 *
 * This firmware can operate in two modes by toggling the #define below:
 *
 *   FOLLOWER_MODE (default): Receives set_positions from PC and moves servos.
 *                            Used for the arm that EXECUTES actions.
 *
 *   LEADER_MODE:             Reads servo positions and streams them to PC.
 *                            Torque is DISABLED so human can move the arm freely.
 *                            Used for the arm that PROVIDES actions (teleoperator).
 *
 * To select mode, uncomment ONE of the following lines before flashing:
 */
//#define LEADER_MODE
#define FOLLOWER_MODE

#ifndef LEADER_MODE
#ifndef FOLLOWER_MODE
#define FOLLOWER_MODE
#endif
#endif
```

| 行 | 解释 |
|----|------|
| `//#define LEADER_MODE` | 注释掉的编译开关，取消注释则编译为 Leader 模式 |
| `#define FOLLOWER_MODE` | 当前激活的编译开关，Follower 模式（默认） |
| `#ifndef LEADER_MODE` | 双重保护：如果两个模式都未定义，自动选择 Follower |

**编译时分支**：

```cpp
#ifdef LEADER_MODE
const char* MODE_TAG = "[LEADER]";
const int   LEADER_STREAM_HZ = 30;  // Leader 向 PC 发送位置数据的频率
#endif

#ifdef FOLLOWER_MODE
const char* MODE_TAG = "[FOLLOWER]";
#endif
```

- `MODE_TAG`：用于串口/显示屏输出的模式标识字符串
- `LEADER_STREAM_HZ = 30`：Leader 模式每秒发送 30 次位置数据给 PC

### 11.2 库引入

```cpp
#include <WiFi.h>           // ESP32 WiFi 连接
#include <ArduinoJson.h>    // JSON 序列化/反序列化（PC ↔ ESP32 通信）
#include <SCServo.h>        // Feetech STS3215 舵机驱动库
#include <TFT_eSPI.h>       // ST7789 显示屏驱动（高性能 DMA）
```

### 11.3 显示屏配置

```cpp
TFT_eSPI tft = TFT_eSPI();  // 创建 TFT_eSPI 实例

// 布局常量（240×240 屏幕）
#define DISP_Y_TITLE    0   // 标题区域 Y 坐标
#define DISP_Y_IP      20   // IP 地址区域 Y 坐标
#define DISP_Y_STATUS  40   // 状态区域 Y 坐标
#define DISP_Y_SEP     60   // 分隔线 Y 坐标
#define DISP_Y_MSG     65   // 消息日志区域 Y 坐标
#define DISP_MSG_LINES 22   // 消息区域可显示行数（8px 字体）
#define DISP_MSG_CHARS 40   // 每行最大字符数

// 消息环形缓冲区
#define MSG_BUF_SIZE 32
struct {
  char lines[MSG_BUF_SIZE][DISP_MSG_CHARS + 1];  // 32 行 × 41 字符
  int head;    // 环形缓冲区头部指针
  int count;   // 当前缓冲的消息数量
} msgBuf;
```

**显示布局**：

```
Y:  0-20  标题（固定）     → "SO101 [LEADER]"
Y: 20-40  IP 地址（固定）   → "IP: 192.168.1.105"
Y: 40-60  状态（固定）      → "Status: PC Connected"
Y: 60-65  分隔线
Y: 65+    消息日志（滚动）   → "RX: set_positions"
```

### 11.4 WiFi 与 TCP 配置

```cpp
const char* WIFI_SSID     = "oppowifi";        // WiFi 名称
const char* WIFI_PASSWORD = "asdfghjkl";       // WiFi 密码

// AP 模式配置（备用）
//#define USE_AP_MODE                              // 取消注释启用 AP 模式
const char* AP_SSID     = "SO101-ROBOT";         // AP 热点名称
const char* AP_PASSWORD = "12345678";            // AP 密码
const int   AP_CHANNEL  = 6;                     // WiFi 信道

// TCP 服务器端口
const int PORT_CMD = 8888;   // 命令端口（PC → ESP32）
const int PORT_OBS = 8889;   // 观测端口（ESP32 → PC）

WiFiServer serverCmd(PORT_CMD);  // 命令服务器实例
WiFiServer serverObs(PORT_OBS);  // 观测服务器实例
WiFiClient clientCmd;            // 当前连接的命令客户端
WiFiClient clientObs;            // 当前连接的观测客户端
bool clientWasConnected = false; // 记录上一帧的连接状态（用于检测断开）
```

### 11.5 舵机配置

```cpp
#define SERVO_SERIAL Serial2   // 使用 UART2 连接舵机控制板
#define SERVO_TX_PIN  17       // GPIO17 → Waveshare RX
#define SERVO_RX_PIN  18       // GPIO18 ← Waveshare TX
#define SERVO_BAUD    1000000  // 波特率 1 Mbps（Feetech 默认）

SMS_STS st;  // Feetech 舵机控制对象

const uint8_t MOTOR_IDS[] = {1, 2, 3, 4, 5, 6};  // 6 个舵机 ID
const char*   MOTOR_NAMES[] = {                    // 舵机名称（JSON 中使用）
  "shoulder_pan", "shoulder_lift", "elbow_flex",
  "wrist_flex", "wrist_roll", "gripper"
};
const int NUM_MOTORS = 6;
```

### 11.6 显示辅助函数

```cpp
void initDisplay() {
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);  // GPIO5 拉高 → 背光开启

  tft.init();              // 初始化 TFT 控制器
  tft.setSwapBytes(true);  // ESP32-S3 小端 → ST7789 大端，必须交换字节
  tft.setRotation(0);      // 屏幕方向 0°（不旋转）
  tft.fillScreen(TFT_BLACK);  // 全屏黑色背景
  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // 白字黑底
  tft.setTextFont(1);      // 使用内置字体 1（8×8）
  tft.setTextSize(1);      // 字体缩放 1 倍

  // 初始化消息环形缓冲区
  msgBuf.head = 0;
  msgBuf.count = 0;
  for (int i = 0; i < MSG_BUF_SIZE; i++) {
    msgBuf.lines[i][0] = '\0';
  }

  // 绘制标题
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(0, DISP_Y_TITLE);
  tft.print("SO101 ");
  tft.print(MODE_TAG);  // 显示 [LEADER] 或 [FOLLOWER]

  // 绘制分隔线
  tft.drawLine(0, DISP_Y_SEP, 239, DISP_Y_SEP, TFT_DARKGREY);
}
```

**IP 和状态更新函数**：

```cpp
void updateDisplayIP(const char* ip) {
  tft.fillRect(0, DISP_Y_IP, 240, 18, TFT_BLACK);  // 清除旧 IP
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(0, DISP_Y_IP);
  tft.print("IP: ");
  tft.print(ip);
}

void updateDisplayStatus(const char* status, uint16_t color) {
  tft.fillRect(0, DISP_Y_STATUS, 240, 18, TFT_BLACK);  // 清除旧状态
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(0, DISP_Y_STATUS);
  tft.print(status);
}
```

**消息日志（滚动显示）**：

```cpp
void addDisplayMessage(const char* msg) {
  // 1. 计算写入位置（环形缓冲区）
  int idx = (msgBuf.head + msgBuf.count) % MSG_BUF_SIZE;
  strncpy(msgBuf.lines[idx], msg, DISP_MSG_CHARS);  // 复制消息
  msgBuf.lines[idx][DISP_MSG_CHARS] = '\0';          // 确保 null 结尾

  // 2. 更新缓冲区指针
  if (msgBuf.count < MSG_BUF_SIZE) {
    msgBuf.count++;           // 未满：增加计数
  } else {
    msgBuf.head = (msgBuf.head + 1) % MSG_BUF_SIZE;  // 已满：滚动头部
  }

  // 3. 重绘消息区域
  tft.fillRect(0, DISP_Y_MSG, 240, 240 - DISP_Y_MSG, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int visibleLines = min(msgBuf.count, DISP_MSG_LINES);
  for (int i = 0; i < visibleLines; i++) {
    int bufIdx = (msgBuf.head + i) % MSG_BUF_SIZE;  // 从头部开始显示
    tft.setCursor(0, DISP_Y_MSG + i * 8);
    tft.print(msgBuf.lines[bufIdx]);
  }
}
```

### 11.7 UART 辅助函数

```cpp
void flushRxBuffer() {
  while (SERVO_SERIAL.available()) {
    SERVO_SERIAL.read();  // 丢弃所有未读数据
  }
}
```

**用途**：发送命令前清空接收缓冲区，防止旧的状态包干扰新读取。

### 11.8 电机配置函数

```cpp
void configureMotors() {
  Serial.println("[V2] Configuring motors (ACC=254)...");
  addDisplayMessage("Config motors...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_ACC, 254);  // 写入 SRAM 寄存器
    delayMicroseconds(2000);  // 每个舵机间隔 2ms，避免总线冲突
  }
  delayMicroseconds(5000);   // 等待所有状态包到达
  flushRxBuffer();           // 清空状态包
  Serial.println("[V2] Motor config done");
  addDisplayMessage("Motor config done");
}
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `SMS_STS_ACC` | 41 | 加速度寄存器地址（SRAM） |
| `254` | 最大值 | 最大加速度，瞬时响应 |

```cpp
void enableAllTorque() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_TORQUE_ENABLE, 1);  // 扭矩使能
    delayMicroseconds(2000);
  }
}

void disableAllTorque() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_TORQUE_ENABLE, 0);  // 扭矩关闭
    delayMicroseconds(2000);
  }
}
```

### 11.9 syncWrite（批量写位置）

```cpp
void syncWritePositions(const uint16_t positions[]) {
  // 1. 防御：清空旧状态包
  flushRxBuffer();

  // 2. 构建数据数组：每个舵机 2 字节（低位、高位）
  uint8_t data[NUM_MOTORS * 2];
  for (int i = 0; i < NUM_MOTORS; i++) {
    data[i * 2]     = positions[i] & 0xFF;        // 低字节
    data[i * 2 + 1] = (positions[i] >> 8) & 0xFF; // 高字节
  }

  // 3. 发送 SYNC_WRITE 指令（单包广播所有舵机）
  st.syncWrite((u8*)MOTOR_IDS,      // 舵机 ID 数组
               NUM_MOTORS,           // 舵机数量
               SMS_STS_GOAL_POSITION_L,  // 起始寄存器地址（42）
               data,                 // 数据数组
               2);                   // 每个舵机数据长度（2 字节）

  SERVO_SERIAL.flush();  // 等待发送完成
}
```

**性能对比**：

| 方式 | 时间 | 说明 |
|------|------|------|
| 6× WritePosEx | ~12ms | 逐个发送，6 次往返 |
| **syncWrite** | **~1ms** | **单包广播，1 次往返** |

### 11.10 syncRead（批量读位置）

```cpp
bool syncReadPositions(int outPositions[]) {
  // 1. 初始化 syncRead 缓冲区：6 个舵机，每个 2 字节，50ms 超时
  st.syncReadBegin(NUM_MOTORS, 2, 50);

  // 2. 发送 syncRead 请求
  int rxLen = st.syncReadPacketTx((u8*)MOTOR_IDS, NUM_MOTORS,
                                   SMS_STS_PRESENT_POSITION_L, 2);
  if (rxLen <= 0) {
    st.syncReadEnd();
    return false;  // 发送失败
  }

  // 3. 从响应中提取每个舵机的位置
  bool ok = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    uint8_t buf[2];
    int n = st.syncReadPacketRx(MOTOR_IDS[i], buf);
    if (n == 2) {
      outPositions[i] = buf[0] | (buf[1] << 8);  // 低位 + 高位
    } else {
      outPositions[i] = -1;  // 读取失败标记
      ok = false;
    }
  }

  st.syncReadEnd();
  return ok;
}
```

**性能对比**：

| 方式 | 时间 | 说明 |
|------|------|------|
| 6× ReadPos | ~30ms | 逐个读取，6 次往返 |
| **syncRead** | **~5ms** | **单包读取，1 次往返** |

### 11.11 setup() — 初始化流程

```cpp
void setup() {
  Serial.begin(115200);   // 串口调试输出
  delay(1000);            // 等待串口稳定

  initDisplay();          // 初始化显示屏（最先执行，显示启动信息）

  // Step 1: UART 初始化
  SERVO_SERIAL.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &SERVO_SERIAL;  // 将 UART 绑定到舵机库
  delay(500);

  // Step 2: 扫描舵机
  int found = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    int pos = st.ReadPos(MOTOR_IDS[i]);  // 尝试读取位置
    if (pos >= 0) {
      found++;  // 响应正常
    }
  }

  // Step 3: 配置电机（两个模式都需要）
  disableAllTorque();     // 先关闭扭矩（才能写入 SRAM）
  configureMotors();      // 设置 ACC=254

#ifdef LEADER_MODE
  // Leader：保持扭矩关闭（人工可自由移动）
  Serial.println("[LEADER] Torque DISABLED");
#else
  // Follower：开启扭矩（执行命令）
  enableAllTorque();
#endif

  // Step 4: WiFi 连接
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500);
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    updateDisplayIP(WiFi.localIP().toString().c_str());
  } else {
    disableAllTorque();     // WiFi 失败 → 安全关闭扭矩
    while (true) { delay(1000); }  // 停止运行
  }

  // Step 5: 启动 TCP 服务器
  serverCmd.begin();
  serverObs.begin();
}
```

### 11.12 sendObservation() — 发送观测数据

```cpp
void sendObservation() {
  int poses[NUM_MOTORS];
  bool ok = syncReadPositions(poses);  // 读取所有舵机位置

  // 构建 JSON 响应
  StaticJsonDocument<512> resp;
  for (int i = 0; i < NUM_MOTORS; i++) {
    resp[MOTOR_NAMES[i]] = (poses[i] < 0) ? -1 : poses[i];
  }

  // 序列化为字符串并发送
  String out;
  serializeJson(resp, out);
  out += "\n";

  if (clientObs && clientObs.connected()) {
    clientObs.print(out);  // 通过 TCP 发送给 PC
  }
}
```

### 11.13 loop() — 主循环

```cpp
void loop() {
  // ====== 连接管理 ======
  if (!clientCmd || !clientCmd.connected()) {
    if (clientWasConnected) {
      // 检测到断开
      clientWasConnected = false;
      addDisplayMessage("CMD disconnected");
#ifdef FOLLOWER_MODE
      disableAllTorque();  // Follower 断开 → 安全释放扭矩
#endif
    }
    // 尝试接受新连接
    WiFiClient nc = serverCmd.available();
    if (nc) {
      clientCmd = nc;
      clientCmd.setNoDelay(true);  // 禁用 Nagle，降低延迟
      clientWasConnected = true;
    }
  }

  if (!clientObs || !clientObs.connected()) {
    WiFiClient nc = serverObs.available();
    if (nc) {
      clientObs = nc;
      clientObs.setNoDelay(true);
    }
  }

#ifdef LEADER_MODE
  // ====== Leader 模式 ======
  unsigned long now = millis();
  if (clientObs && clientObs.connected() &&
      (now - lastStreamMs >= streamIntervalMs)) {
    lastStreamMs = now;
    sendObservation();           // 定时发送位置给 PC
    addDisplayMessage("TX: positions");
  }

  // 响应显式的 get_obs 请求
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    // ... 解析 JSON，如果是 get_obs 则 sendObservation()
  }

#else
  // ====== Follower 模式 ======
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    line.trim();

    // 解析 JSON 命令
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, line);

    const char* cmd = doc["cmd"];

    if (strcmp(cmd, "set_positions") == 0) {
      // 提取位置并执行
      uint16_t goals[NUM_MOTORS];
      for (int i = 0; i < NUM_MOTORS; i++) {
        int val = doc["positions"][MOTOR_NAMES[i]].as<int>();
        goals[i] = (uint16_t)val;
      }
      syncWritePositions(goals);  // 批量写入舵机
    }
    else if (strcmp(cmd, "get_obs") == 0) {
      sendObservation();  // 回传当前位置
    }
    else if (strcmp(cmd, "release_torque") == 0) {
      disableAllTorque();  // 释放扭矩
    }
  }
#endif
}
```

### 11.14 关键设计要点总结

| 设计 | 说明 |
|------|------|
| **编译时双模式** | `#define LEADER_MODE` / `#define FOLLOWER_MODE` 切换，同一份代码编译出两种固件 |
| **syncWrite** | 单包广播 6 个舵机，写入延迟从 ~12ms 降至 ~1ms |
| **syncRead** | 单包读取 6 个舵机，读取延迟从 ~30ms 降至 ~5ms |
| **TCP_NODELAY** | ESP32 和 PC 端均禁用 Nagle 算法，消除 TCP 缓冲延迟 |
| **双 TCP 端口** | 8888（命令）和 8889（观测）分离，避免读写冲突 |
| **环形消息缓冲区** | 32 条消息的滚动日志，新消息覆盖最旧消息 |
| **安全机制** | TCP 断开自动释放扭矩，WiFi 失败停止运行 |
| **ACC=254** | 最大加速度设置，舵机瞬时响应 |

---

## 12. 回退方式

```bash
pip uninstall lerobot_robot_remote_so101
```

LeRobot 源码仓库从头到尾没有任何变更。

---

## 关联文档

- [LeRobot 官方文档 - Robot](https://huggingface.co/docs/lerobot)
- [Feetech STS3215 通信协议](https://www.feetech.cn/)
- [SO101_ESP32远程控制_烧录与使用指南](./SO101_ESP32远程控制_烧录与使用指南.md)
- [LeRobot_SO101数据集采集实战指南](./LeRobot_SO101数据集采集实战指南.md)

---

**版本迭代记录**

| 日期 | 操作 | 内容摘要 |
|------|------|---------|
| 2026-05-05 | 创建 | 初始版本，记录远程 replay 架构设计与实现方案 |
| 2026-05-05 | 更新 | 新增 v2 固件性能优化说明（syncWrite/syncRead/TCP_NODELAY）与 skip_observation 兜底方案 |
| 2026-05-05 | 更新 | 新增 teleoperation 支持章节，插件同时支持 replay 和 teleop 两种模式 |
| 2026-05-05 | 新增 | 新增双模式（Leader/Follower）章节，支持无线双臂遥操作 |
| 2026-05-05 | 新增 | 新增 ST7789 显示屏支持章节，包括硬件接线、显示布局、技术实现 |
| 2026-05-05 | 新增 | 新增固件代码详解章节（第 11 节），逐行解释 `esp32_so101_bridge_v2.ino` |

---

*最后更新: 2026-05-05*
