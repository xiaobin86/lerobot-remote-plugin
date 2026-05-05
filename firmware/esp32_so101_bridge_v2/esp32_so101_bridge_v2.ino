/**
 * ESP32 SO101 Remote Bridge v4 — Arm Visualization + Bimanual + Chinese SSID
 *
 * Features:
 *   - Real-time 2D arm stick-figure visualization on ST7789 (replaces message log)
 *   - Bimanual Leader/Follower dual mode via compile-time #define
 *   - Chinese WiFi SSID display via GT30L32S4W font chip
 *   - Unified 16px font height (ASCII 12x16 + Chinese 16x16)
 *
 * To select mode, uncomment ONE of the following lines before flashing:
 */
#define LEADER_MODE
//#define FOLLOWER_MODE

#ifndef LEADER_MODE
#ifndef FOLLOWER_MODE
#define FOLLOWER_MODE
#endif
#endif

#ifdef LEADER_MODE
const char* MODE_TAG = "LEADER";
const int   LEADER_STREAM_HZ = 30;
#endif
#ifdef FOLLOWER_MODE
const char* MODE_TAG = "FOLLOWER";
#endif

#include <WiFi.h>
#include <ArduinoJson.h>
#include <SCServo.h>
#include <TFT_eSPI.h>
#include <SPI.h>

// ===================== Display + Font Chip Configuration =====================
TFT_eSPI tft = TFT_eSPI();

// 字库片选（GT30L32S4W）
#define FONT_CS     9
#define FONT_SPI_FREQ  8000000UL

static uint8_t  fontBuf[32];
static uint16_t pixBuf[16 * 16];

// Layout constants (240x240 screen, unified 16px height font)
// ASCII: 12x16 (setTextSize(2)), Chinese: 16x16
#define LINE_HEIGHT    18   // 16px font + 2px spacing
#define CHAR_W_ASCII   12   // setTextSize(2) width
#define CHAR_W_CN      16   // Chinese width

#define DISP_Y_TITLE    0
#define DISP_Y_IP      18
#define DISP_Y_WIFI    36
#define DISP_Y_PC      54
#define DISP_Y_SEP     72
#define DISP_Y_MSG     74
#define DISP_MSG_LINES  9   // (240-74) / 18 = 9

// Message ring buffer
#define MSG_BUF_SIZE 32
struct {
  char lines[MSG_BUF_SIZE][41];  // 40 chars + null
  int head;
  int count;
} msgBuf;

// ===================== WiFi Configuration =====================
// UTF-8 SSID for WiFi connection
const char* WIFI_SSID     = "上网6元一小时";
const char* WIFI_PASSWORD = "asdfghjkl";

// GB2312 encoded SSID for screen display (confirmed encoding)
// 上:C9CF 网:CDF8 6:36 元:D4AA 一:D2BB 小:D0A1 时:CAB1
const char* WIFI_SSID_GB2312 = "\xC9\xCF\xCD\xF8\x36\xD4\xAA\xD2\xBB\xD0\xA1\xCA\xB1";

// Option B: Access Point mode
//#define USE_AP_MODE
const char* AP_SSID     = "SO101-ROBOT";
const char* AP_PASSWORD = "12345678";
const int   AP_CHANNEL  = 6;

// ===================== TCP Server Ports =====================
const int PORT_CMD = 8888;
const int PORT_OBS = 8889;

WiFiServer serverCmd(PORT_CMD);
WiFiServer serverObs(PORT_OBS);
WiFiClient clientCmd;
WiFiClient clientObs;
bool clientWasConnected = false;

// ===================== Feetech / SCServo Config =====================
#define SERVO_SERIAL Serial2
#define SERVO_TX_PIN  17
#define SERVO_RX_PIN  18
#define SERVO_BAUD    1000000

SMS_STS st;

const uint8_t MOTOR_IDS[] = {1, 2, 3, 4, 5, 6};
const char*   MOTOR_NAMES[] = {
  "shoulder_pan", "shoulder_lift", "elbow_flex",
  "wrist_flex", "wrist_roll", "gripper"
};
const int NUM_MOTORS = 6;

// ===================== Calibration Data (from L07252802.json / R12552802.json) =====================
// Mid-point and half-range for each servo, averaged from Leader & Follower calibration
// Used to normalize servo positions to [-1..1] for accurate visualization
// Order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
const int SERVO_MID[]       = {2112, 2014, 2008, 2052, 2048, 2689};  // mid = (min+max)/2
const int SERVO_RANGE_HALF[] = {1330, 1183, 1101, 1158, 2048,  656};  // half of (max-min)

// ===================== Arm Visualization Configuration =====================
// Comment out to disable arm visualization and restore message scrolling
#define ENABLE_ARM_VIZ

#ifdef ENABLE_ARM_VIZ

// Arm link lengths (mm) — approximate for SO101
#define ARM_L1_BASE    40   // Base pedestal to shoulder lift joint
#define ARM_L2_UPPER   120  // Upper arm (shoulder to elbow)
#define ARM_L3_FORE    100  // Forearm (elbow to wrist flex)
#define ARM_L4_WRIST   70   // Wrist flex to gripper center

// Visualization area (below status bar separator at Y=72)
#define ARM_AREA_Y0    74
#define ARM_AREA_Y1    240
#define ARM_AREA_H     (ARM_AREA_Y1 - ARM_AREA_Y0)  // 166 pixels

// Base position on screen (centered horizontally, near bottom)
#define ARM_BASE_X     100
#define ARM_BASE_Y     232

// Scale: pixels per millimeter
#define ARM_SCALE      0.40f

// Drawing constants
#define ARM_COLOR_BASE      TFT_DARKGREY
#define ARM_COLOR_LINK      TFT_CYAN
#define ARM_COLOR_JOINT     TFT_YELLOW
#define ARM_COLOR_GRIPPER   TFT_GREEN
#define ARM_COLOR_GRIPPER_CLOSED TFT_RED
#define JOINT_RADIUS        4
#define GRIPPER_LEN         14

// Current servo positions cached for visualization — init to mid (rest pose)
int vizPositions[NUM_MOTORS] = {2112, 2014, 2008, 2052, 2048, 2689};
bool vizPositionsValid = false;

// Computed joint screen coordinates
// [0]=base bottom, [1]=shoulder, [2]=elbow, [3]=wrist, [4]=gripper center
struct ArmPoint { int16_t x, y; };
ArmPoint armPts[5];

const float DEG2RAD = PI / 180.0f;

// Cached absolute wrist angle (for gripper orientation)
float wristAbsAngle = 0.0f;

#endif // ENABLE_ARM_VIZ

// ===================== GT30L32S4W Font Functions =====================

// GB2312 address calculation
uint32_t gb2312Addr(uint8_t msb, uint8_t lsb) {
  if (msb >= 0xA1 && msb <= 0xA9 && lsb >= 0xA1)
    return ((uint32_t)(msb - 0xA1) * 94 + (lsb - 0xA1)) * 32 + 0x2C9D0;
  if (msb >= 0xB0 && msb <= 0xF7 && lsb >= 0xA1)
    return ((uint32_t)(msb - 0xB0) * 94 + (lsb - 0xA1) + 846) * 32 + 0x2C9D0;
  return 0;
}

bool fontRead(uint32_t addr) {
  digitalWrite(TFT_CS, HIGH);
  SPI.beginTransaction(SPISettings(FONT_SPI_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(FONT_CS, LOW);
  SPI.transfer(0x03);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >>  8) & 0xFF);
  SPI.transfer( addr        & 0xFF);
  for (int i = 0; i < 32; i++) fontBuf[i] = SPI.transfer(0x00);
  digitalWrite(FONT_CS, HIGH);
  SPI.endTransaction();
  for (int i = 0; i < 32; i++) if (fontBuf[i]) return true;
  return false;
}

// Draw 16x16 Chinese character
void drawHanzi(int16_t x, int16_t y, uint8_t msb, uint8_t lsb,
               uint16_t fg, uint16_t bg) {
  uint32_t addr = gb2312Addr(msb, lsb);
  if (!addr || !fontRead(addr)) return;
  for (int row = 0; row < 16; row++) {
    uint8_t b0 = fontBuf[row * 2];
    uint8_t b1 = fontBuf[row * 2 + 1];
    for (int col = 0; col < 8; col++) {
      pixBuf[row * 16 + col]     = (b0 & (0x80 >> col)) ? fg : bg;
      pixBuf[row * 16 + 8 + col] = (b1 & (0x80 >> col)) ? fg : bg;
    }
  }
  tft.pushImage(x, y, 16, 16, pixBuf);
}

// Unified text drawing: ASCII 12x16 + Chinese 16x16, aligned at same baseline
void drawStr(int16_t x, int16_t y, const char* str,
             uint16_t fg, uint16_t bg, int16_t maxWidth = 240) {
  int16_t cx = x, cy = y;
  int16_t rightBound = (maxWidth > 0) ? (x + maxWidth) : 240;
  
  while (*str) {
    uint8_t c = (uint8_t)*str;
    if (c == '\n') { cx = x; cy += LINE_HEIGHT; str++; continue; }
    
    if (c >= 0x80) {
      // Chinese character: 16x16
      uint8_t lsb = (uint8_t)*(str + 1);
      if (!lsb) break;
      if (cx + CHAR_W_CN > rightBound) { cx = x; cy += LINE_HEIGHT; }
      if (cy + 16 > 240) break;
      drawHanzi(cx, cy, c, lsb, fg, bg);
      cx += CHAR_W_CN; str += 2;
    } else {
      // ASCII: 12x16 (setTextSize(2))
      if (cx + CHAR_W_ASCII > rightBound) { cx = x; cy += LINE_HEIGHT; }
      if (cy + 16 > 240) break;
      tft.setTextSize(2);
      tft.setTextColor(fg, bg);
      tft.setCursor(cx, cy);
      tft.print((char)c);
      cx += CHAR_W_ASCII; str++;
    }
  }
}

// Calculate pixel width of a mixed string
int strPixelWidth(const char* str) {
  int w = 0;
  while (*str) {
    uint8_t c = (uint8_t)*str;
    if (c >= 0x80) { w += CHAR_W_CN; str += 2; }
    else { w += CHAR_W_ASCII; str++; }
  }
  return w;
}

// ===================== Arm Visualization Functions =====================

#ifdef ENABLE_ARM_VIZ

// Normalize servo position to [-1..1] using calibrated mid-point and range
// idx: 0=shoulder_pan, 1=shoulder_lift, 2=elbow_flex, 3=wrist_flex, 4=wrist_roll, 5=gripper
inline float normPos(int idx, int pos) {
  if (pos < 0) return 0.0f;
  return (pos - SERVO_MID[idx]) / (float)SERVO_RANGE_HALF[idx];
}

// Compute screen coordinates of all arm joints from servo positions
// Coordinate system: 0° = horizontal right (+X), +90° = vertical up (-Y)
void updateArmGeometry(const int positions[]) {
  // Base center bottom
  armPts[0].x = ARM_BASE_X;
  armPts[0].y = ARM_BASE_Y;

  // Shoulder joint (top of base pedestal)
  armPts[1].x = ARM_BASE_X;
  armPts[1].y = ARM_BASE_Y - (int16_t)(ARM_L1_BASE * ARM_SCALE);

  // shoulder_lift: mid(2014) = horizontal (0°), range ±90°
  // min(831) = vertical down (-90°), max(3217) = vertical up (+90°)
  float absA1 = normPos(1, positions[1]) * 90.0f * DEG2RAD;

  // elbow_flex: relative to upper arm.
  // min(929) = 0° (folded back), mid(2008) = 90° (perpendicular down),
  // max(3128) = 180° (straight, continuing upper arm direction)
  float elbowRel = (1.0f + normPos(2, positions[2])) * 90.0f * DEG2RAD;
  float absA2 = absA1 + elbowRel;

  // wrist_flex: relative to forearm.
  // mid(2052) = aligned with forearm (0°), range ±90°
  float wristRel = normPos(3, positions[3]) * 90.0f * DEG2RAD;
  float absA3 = absA2 + wristRel;

  // Cache for gripper orientation
  wristAbsAngle = absA3;

  // Upper arm end = elbow
  armPts[2].x = armPts[1].x + (int16_t)(ARM_L2_UPPER * ARM_SCALE * cosf(absA1));
  armPts[2].y = armPts[1].y - (int16_t)(ARM_L2_UPPER * ARM_SCALE * sinf(absA1));

  // Forearm end = wrist flex joint
  armPts[3].x = armPts[2].x + (int16_t)(ARM_L3_FORE * ARM_SCALE * cosf(absA2));
  armPts[3].y = armPts[2].y - (int16_t)(ARM_L3_FORE * ARM_SCALE * sinf(absA2));

  // Gripper center
  armPts[4].x = armPts[3].x + (int16_t)(ARM_L4_WRIST * ARM_SCALE * cosf(absA3));
  armPts[4].y = armPts[3].y - (int16_t)(ARM_L4_WRIST * ARM_SCALE * sinf(absA3));
}

// Draw thick line by drawing the main line plus small offsets
void drawThickLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  tft.drawLine(x0, y0, x1, y1, color);
  tft.drawLine(x0 + 1, y0, x1 + 1, y1, color);
  tft.drawLine(x0, y0 + 1, x1, y1 + 1, color);
}

// Draw the complete arm stick figure
void drawArm() {
  // Clear only the visualization area
  tft.fillRect(0, ARM_AREA_Y0, 240, ARM_AREA_H, TFT_BLACK);

  if (!vizPositionsValid) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(50, ARM_AREA_Y0 + ARM_AREA_H / 2 - 8);
    tft.print("No arm data");
    return;
  }

  updateArmGeometry(vizPositions);

  // Draw base pedestal (grey rectangle)
  int16_t baseW = 24;
  int16_t baseH = (int16_t)(ARM_L1_BASE * ARM_SCALE);
  tft.fillRect(ARM_BASE_X - baseW / 2, ARM_BASE_Y - baseH, baseW, baseH, ARM_COLOR_BASE);

  // Draw links (thick cyan lines)
  drawThickLine(armPts[1].x, armPts[1].y, armPts[2].x, armPts[2].y, ARM_COLOR_LINK);
  drawThickLine(armPts[2].x, armPts[2].y, armPts[3].x, armPts[3].y, ARM_COLOR_LINK);
  drawThickLine(armPts[3].x, armPts[3].y, armPts[4].x, armPts[4].y, ARM_COLOR_LINK);

  // Draw joints (yellow circles with black outline)
  for (int i = 1; i <= 4; i++) {
    tft.fillCircle(armPts[i].x, armPts[i].y, JOINT_RADIUS, ARM_COLOR_JOINT);
    tft.drawCircle(armPts[i].x, armPts[i].y, JOINT_RADIUS, TFT_BLACK);
  }

  // Draw gripper
  // gripper min(2040)=closed, max(3213)=open (Leader calibration)
  // Map to [0..1] where 0=closed, 1=open
  float gripNorm = (vizPositions[5] - 2040.0f) / (3213.0f - 2040.0f);
  if (gripNorm < 0.0f) gripNorm = 0.0f;
  if (gripNorm > 1.0f) gripNorm = 1.0f;
  uint16_t gripColor = (gripNorm > 0.30f) ? ARM_COLOR_GRIPPER : ARM_COLOR_GRIPPER_CLOSED;

  // Gripper spread angle (0..20 degrees based on openness)
  float spread = gripNorm * 20.0f * DEG2RAD;

  // Use cached wrist absolute angle
  float wristAngle = wristAbsAngle;

  int16_t gx = armPts[4].x;
  int16_t gy = armPts[4].y;

  // Two gripper fingers
  float a1 = wristAngle + spread;
  float a2 = wristAngle - spread;
  int16_t g1x = gx + (int16_t)(GRIPPER_LEN * cosf(a1));
  int16_t g1y = gy - (int16_t)(GRIPPER_LEN * sinf(a1));
  int16_t g2x = gx + (int16_t)(GRIPPER_LEN * cosf(a2));
  int16_t g2y = gy - (int16_t)(GRIPPER_LEN * sinf(a2));

  tft.drawLine(gx, gy, g1x, g1y, gripColor);
  tft.drawLine(gx, gy, g2x, g2y, gripColor);

  // Small white dot at gripper center
  tft.fillCircle(gx, gy, 2, TFT_WHITE);
}

#endif // ENABLE_ARM_VIZ

// ===================== Display Helpers =====================
void initDisplay() {
  pinMode(5, OUTPUT); digitalWrite(5, HIGH);
  pinMode(FONT_CS, OUTPUT); digitalWrite(FONT_CS, HIGH);

  tft.init();
  tft.setSwapBytes(true);
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  msgBuf.head = 0; msgBuf.count = 0;
  for (int i = 0; i < MSG_BUF_SIZE; i++) msgBuf.lines[i][0] = '\0';

  // Title: "SO101 LEADER" or "SO101 FOLLOWER"
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(0, DISP_Y_TITLE);
  tft.print("SO101 ");
  tft.print(MODE_TAG);

  tft.drawLine(0, DISP_Y_SEP, 239, DISP_Y_SEP, TFT_DARKGREY);
}

void updateDisplayIP(const char* ip) {
  tft.fillRect(0, DISP_Y_IP, 240, LINE_HEIGHT, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(0, DISP_Y_IP);
  tft.print("IP:");
  tft.print(ip);
}

// WiFi status with colored dot + Chinese SSID
void updateDisplayWiFi(bool connected, bool connecting) {
  tft.fillRect(0, DISP_Y_WIFI, 240, LINE_HEIGHT, TFT_BLACK);
  
  // Colored status dot (radius 5, center at y+8)
  uint16_t dotColor = connected ? TFT_GREEN : (connecting ? TFT_YELLOW : TFT_RED);
  tft.fillCircle(8, DISP_Y_WIFI + 8, 5, dotColor);
  
  // Text after dot
  int x = 20;
  if (connected) {
    drawStr(x, DISP_Y_WIFI, "WiFi ", TFT_WHITE, TFT_BLACK);
    drawStr(x + 5*CHAR_W_ASCII, DISP_Y_WIFI, WIFI_SSID_GB2312, TFT_WHITE, TFT_BLACK);
  } else if (connecting) {
    drawStr(x, DISP_Y_WIFI, "WiFi Connecting...", TFT_YELLOW, TFT_BLACK);
  } else {
    drawStr(x, DISP_Y_WIFI, "WiFi FAILED", TFT_RED, TFT_BLACK);
  }
}

// PC connection status with colored dot
void updateDisplayPC(bool connected, const char* ip) {
  tft.fillRect(0, DISP_Y_PC, 240, LINE_HEIGHT, TFT_BLACK);
  
  uint16_t dotColor = connected ? TFT_GREEN : TFT_DARKGREY;
  tft.fillCircle(8, DISP_Y_PC + 8, 5, dotColor);
  
  tft.setTextSize(2);
  tft.setCursor(20, DISP_Y_PC);
  if (connected) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("PC ");
    tft.print(ip);
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("PC offline");
  }
}

void addDisplayMessage(const char* msg) {
  // Always log to serial for debugging
  Serial.println(msg);

#ifndef ENABLE_ARM_VIZ
  // Scroll message text on screen only when arm visualization is disabled
  int idx = (msgBuf.head + msgBuf.count) % MSG_BUF_SIZE;
  strncpy(msgBuf.lines[idx], msg, 40);
  msgBuf.lines[idx][40] = '\0';

  if (msgBuf.count < MSG_BUF_SIZE) msgBuf.count++;
  else msgBuf.head = (msgBuf.head + 1) % MSG_BUF_SIZE;

  // Redraw message area
  tft.fillRect(0, DISP_Y_MSG, 240, 240 - DISP_Y_MSG, TFT_BLACK);

  int visibleLines = min(msgBuf.count, DISP_MSG_LINES);
  for (int i = 0; i < visibleLines; i++) {
    int bufIdx = (msgBuf.head + i) % MSG_BUF_SIZE;
    drawStr(0, DISP_Y_MSG + i * LINE_HEIGHT, msgBuf.lines[bufIdx], TFT_WHITE, TFT_BLACK);
  }
#endif
}

void addDisplayMessage(String msg) {
  addDisplayMessage(msg.c_str());
}

// ===================== UART helpers =====================
void flushRxBuffer() {
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
}

// ===================== Motor Configuration =====================
void configureMotors() {
  Serial.println("[V2] Configuring motors (ACC=254)...");
  addDisplayMessage("Config motors...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_ACC, 254);
    delayMicroseconds(2000);
  }
  delayMicroseconds(5000);
  flushRxBuffer();
  Serial.println("[V2] Motor config done");
  addDisplayMessage("Motor config OK");
}

void enableAllTorque() {
  Serial.println("[V2] Enabling torque...");
  addDisplayMessage("Enable torque...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_TORQUE_ENABLE, 1);
    delayMicroseconds(2000);
  }
  delayMicroseconds(5000);
  flushRxBuffer();
  Serial.println("[V2] Torque enabled");
  addDisplayMessage("Torque ON");
}

void disableAllTorque() {
  addDisplayMessage("Release torque...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_TORQUE_ENABLE, 0);
    delayMicroseconds(2000);
  }
}

// ===================== syncWrite / syncRead =====================
void syncWritePositions(const uint16_t positions[]) {
  flushRxBuffer();
  uint8_t data[NUM_MOTORS * 2];
  for (int i = 0; i < NUM_MOTORS; i++) {
    data[i * 2]     = positions[i] & 0xFF;
    data[i * 2 + 1] = (positions[i] >> 8) & 0xFF;
  }
  st.syncWrite((u8*)MOTOR_IDS, NUM_MOTORS, SMS_STS_GOAL_POSITION_L, data, 2);
  SERVO_SERIAL.flush();
}

bool syncReadPositions(int outPositions[]) {
  st.syncReadBegin(NUM_MOTORS, 2, 50);
  int rxLen = st.syncReadPacketTx((u8*)MOTOR_IDS, NUM_MOTORS, SMS_STS_PRESENT_POSITION_L, 2);
  if (rxLen <= 0) { st.syncReadEnd(); return false; }
  bool ok = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    uint8_t buf[2];
    int n = st.syncReadPacketRx(MOTOR_IDS[i], buf);
    if (n == 2) outPositions[i] = buf[0] | (buf[1] << 8);
    else { outPositions[i] = -1; ok = false; }
  }
  st.syncReadEnd();
  return ok;
}

// ===================== Observation =====================
void sendObservation() {
  int poses[NUM_MOTORS];
  bool ok = syncReadPositions(poses);

#ifdef ENABLE_ARM_VIZ
  if (ok) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (poses[i] >= 0) vizPositions[i] = poses[i];
    }
    vizPositionsValid = true;
  }
#endif

  StaticJsonDocument<512> resp;
  for (int i = 0; i < NUM_MOTORS; i++) {
    resp[MOTOR_NAMES[i]] = (poses[i] < 0) ? -1 : poses[i];
  }
  String out;
  serializeJson(resp, out);
  out += "\n";
  if (clientObs && clientObs.connected()) clientObs.print(out);
  if (!ok) Serial.println("WARNING: syncRead failed");
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  initDisplay();
  Serial.println("\n[V2] ESP32 SO101 Bridge v2 (Chinese SSID + Unified Font)");
  addDisplayMessage("SO101 Bridge v2");

  // UART
  SERVO_SERIAL.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &SERVO_SERIAL;
  delay(500);
  Serial.println("[V2] UART2 OK");
  addDisplayMessage("UART2 OK");

  // Scan servos
  Serial.println("[V2] Scanning servos...");
  addDisplayMessage("Scan servos...");
  int found = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    int pos = st.ReadPos(MOTOR_IDS[i]);
    if (pos >= 0) { 
      Serial.printf("[V2] ID=%d pos=%d\n", MOTOR_IDS[i], pos); 
      found++; 
      vizPositions[i] = pos;  // save for arm visualization
    }
    else { 
      Serial.printf("[V2] ID=%d no response\n", MOTOR_IDS[i]); 
      vizPositions[i] = SERVO_MID[i];  // fallback to mid
    }
  }
  if (found > 0) vizPositionsValid = true;
  Serial.printf("[V2] Found %d/%d servos\n", found, NUM_MOTORS);
  addDisplayMessage((String("Servos ") + found + "/" + NUM_MOTORS).c_str());
  if (found == 0) addDisplayMessage("WARNING: No servos!");

  // Configure
  disableAllTorque();
  configureMotors();

#ifdef LEADER_MODE
  Serial.println("[LEADER] Torque OFF");
  addDisplayMessage("LEADER: torque OFF");
#else
  enableAllTorque();
#endif

  // WiFi
#ifdef USE_AP_MODE
  Serial.print("AP mode: "); Serial.println(AP_SSID);
  addDisplayMessage((String("AP: ") + AP_SSID).c_str());
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  updateDisplayIP(WiFi.softAPIP().toString().c_str());
  updateDisplayWiFi(true, false);
  updateDisplayPC(false, "");
#else
  Serial.print("WiFi: "); Serial.println(WIFI_SSID);
  addDisplayMessage("WiFi connecting...");
  updateDisplayWiFi(false, true);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500); Serial.print("."); retries++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
    updateDisplayIP(WiFi.localIP().toString().c_str());
    updateDisplayWiFi(true, false);
    addDisplayMessage((String("IP: ") + WiFi.localIP().toString()).c_str());
  } else {
    Serial.println("WiFi FAILED!");
    updateDisplayWiFi(false, false);
    addDisplayMessage("WiFi FAILED!");
    disableAllTorque();
    while (true) { delay(1000); }
  }
#endif

  // TCP
  serverCmd.begin();
  serverObs.begin();
  Serial.printf("TCP CMD:%d OBS:%d\n", PORT_CMD, PORT_OBS);
  addDisplayMessage((String("TCP ") + PORT_CMD + "/" + PORT_OBS).c_str());
  updateDisplayPC(false, "");
  
  Serial.println("Setup done. Waiting for PC...");
  addDisplayMessage("Wait for PC...");
}

// ===================== Main Loop =====================
#ifdef LEADER_MODE
unsigned long lastStreamMs = 0;
const unsigned long streamIntervalMs = 1000 / LEADER_STREAM_HZ;
#endif

void loop() {
  // CMD client (8888)
  if (!clientCmd || !clientCmd.connected()) {
    if (clientWasConnected) {
      clientWasConnected = false;
      Serial.println("CMD DISCONNECTED");
      addDisplayMessage("CMD disconnected");
#ifdef FOLLOWER_MODE
      updateDisplayPC(false, "");
      disableAllTorque();
#endif
    }
    WiFiClient nc = serverCmd.available();
    if (nc) {
      clientCmd = nc;
      clientCmd.setNoDelay(true);
      clientWasConnected = true;
      Serial.println("CMD connected: " + clientCmd.remoteIP().toString());
      addDisplayMessage((String("CMD: ") + clientCmd.remoteIP().toString()).c_str());
#ifdef FOLLOWER_MODE
      updateDisplayPC(true, clientCmd.remoteIP().toString().c_str());
#endif
    }
  }
  
  // OBS client (8889)
  if (!clientObs || !clientObs.connected()) {
    WiFiClient nc = serverObs.available();
    if (nc) {
      clientObs = nc;
      clientObs.setNoDelay(true);
      Serial.println("OBS connected: " + clientObs.remoteIP().toString());
      addDisplayMessage((String("OBS: ") + clientObs.remoteIP().toString()).c_str());
#ifdef LEADER_MODE
      updateDisplayPC(true, clientObs.remoteIP().toString().c_str());
#endif
    }
  }
#ifdef LEADER_MODE
  if (clientObs && !clientObs.connected()) {
    updateDisplayPC(false, "");
  }
#endif

#ifdef LEADER_MODE
  unsigned long now = millis();
  if (clientObs && clientObs.connected() && (now - lastStreamMs >= streamIntervalMs)) {
    lastStreamMs = now;
    sendObservation();
    addDisplayMessage("TX: positions");
  }
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, line)) return;
    if (doc["cmd"] && strcmp(doc["cmd"], "get_obs") == 0) {
      sendObservation();
      addDisplayMessage("RX: get_obs");
    }
  }
#else
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    
    Serial.print("RX: "); Serial.println(line);
    addDisplayMessage((String("RX: ") + line).c_str());
    
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
      addDisplayMessage((String("JSON err") ).c_str());
      return;
    }
    
    const char* cmd = doc["cmd"];
    if (!cmd) { addDisplayMessage("Missing cmd"); return; }
    
    if (strcmp(cmd, "set_positions") == 0) {
      uint16_t goals[NUM_MOTORS];
      bool ok = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        int val = doc["positions"][MOTOR_NAMES[i]].as<int>();
        if (val < 0 || val > 4095) ok = false;
        goals[i] = (uint16_t)val;
      }
      if (ok) {
        syncWritePositions(goals);
        addDisplayMessage("TX: positions");
      }
    }
    else if (strcmp(cmd, "get_obs") == 0) {
      sendObservation();
      addDisplayMessage("TX: obs");
    }
    else if (strcmp(cmd, "release_torque") == 0) {
      addDisplayMessage("Release torque...");
      disableAllTorque();
      StaticJsonDocument<128> resp;
      resp["success"] = true;
      resp["message"] = "torque_released";
      String out;
      serializeJson(resp, out);
      out += "\n";
      if (clientCmd && clientCmd.connected()) clientCmd.print(out);
    }
    else {
      addDisplayMessage((String("Unknown: ") + cmd).c_str());
    }
  }
#endif

#ifdef ENABLE_ARM_VIZ
  // Update arm visualization at ~10Hz (independent of command processing)
  static unsigned long lastArmDrawMs = 0;
  unsigned long armNow = millis();
  if (armNow - lastArmDrawMs >= 100) {
    lastArmDrawMs = armNow;
    drawArm();
  }
#endif
}
