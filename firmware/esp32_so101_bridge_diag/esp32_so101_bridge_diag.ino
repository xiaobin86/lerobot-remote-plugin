/**
 * ESP32 SO101 Remote Bridge - DIAGNOSTIC VERSION
 *
 * 这是诊断增强版。如果标准版烧录后无输出，请烧录此版本。
 * 它在 setup() 的每一步都打印日志，帮助定位卡在哪里。
 */

#include <WiFi.h>
#include <ArduinoJson.h>

// ===================== WiFi Configuration =====================
// Option A: Station mode (connect to existing router)
//
// For ASCII-only SSID (English letters, numbers):
const char* WIFI_SSID     = "oppowifi";
const char* WIFI_PASSWORD = "asdfghjkl";
//
// For Chinese SSID: fill in the HEX bytes of your SSID.
// Example: SSID "上网6元一小时" in GBK/GB2312 is:
//   C9 CF CD F8 36 D4 AA D2 BB D0 A1 CA B1
// Uncomment and fill WIFI_SSID_HEX below, then uncomment #define USE_HEX_SSID.
// The helper ssidHexToString() will convert it at runtime.
// #define USE_HEX_SSID
// const char* WIFI_SSID_HEX = "C9CFCD F836D4AAC9CFB5C8A7D2BB";
//
// How to get HEX bytes of your Chinese SSID:
//   Windows CMD:  chcp 936
//                 python -c "import sys; print(''.join('%02X'%b for b in sys.argv[1].encode('gb2312')))" "你的SSID"
//   Python:       '上网6元一小时'.encode('gb2312').hex()

// Option B: Access Point mode (ESP32 creates its own network)
// Uncomment the line below to use AP mode (bypasses Chinese SSID issue):
// #define USE_AP_MODE
const char* AP_SSID     = "SO101-ROBOT";
const char* AP_PASSWORD = "12345678";   // min 8 chars
const int   AP_CHANNEL  = 6;

// ===================== TCP Server Ports =====================
const int PORT_CMD = 8888;
const int PORT_OBS = 8889;

WiFiServer serverCmd(PORT_CMD);
WiFiServer serverObs(PORT_OBS);
WiFiClient clientCmd;
WiFiClient clientObs;

// ===================== Feetech / UART Config =====================
#define SERVO_SERIAL Serial2
#define SERVO_TX_PIN  17
#define SERVO_RX_PIN  18
#define SERVO_BAUD    1000000

const uint8_t MOTOR_IDS[] = {1, 2, 3, 4, 5, 6};
const char*   MOTOR_NAMES[] = {
  "shoulder_pan", "shoulder_lift", "elbow_flex",
  "wrist_flex", "wrist_roll", "gripper"
};
const int NUM_MOTORS = 6;

// Feetech Protocol 0
#define H1 0xFF
#define H2 0xFF
#define INST_WRITE       0x03
#define INST_READ        0x02
#define INST_SYNC_WRITE  0x83
#define ADDR_TORQUE_EN   40
#define ADDR_GOAL_POS    42
#define ADDR_PRESENT_POS 56

uint8_t calcChecksum(uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return ~sum;
}

void syncWritePositions(const uint16_t positions[]) {
  uint8_t dataLen   = 2;
  uint8_t paramCnt  = 2 + NUM_MOTORS * (1 + dataLen);
  uint8_t length    = paramCnt + 2;
  uint8_t pkt[40];
  uint8_t idx = 0;
  pkt[idx++] = H1; pkt[idx++] = H2;
  pkt[idx++] = 0xFE;
  pkt[idx++] = length;
  pkt[idx++] = INST_SYNC_WRITE;
  pkt[idx++] = ADDR_GOAL_POS;
  pkt[idx++] = dataLen;
  for (int i = 0; i < NUM_MOTORS; i++) {
    pkt[idx++] = MOTOR_IDS[i];
    pkt[idx++] = positions[i] & 0xFF;
    pkt[idx++] = positions[i] >> 8;
  }
  pkt[idx] = calcChecksum(&pkt[2], 3 + paramCnt);
  SERVO_SERIAL.write(pkt, idx + 1);
}

void enableTorque(uint8_t id) {
  uint8_t pkt[9] = {H1, H2, id, 4, INST_WRITE, ADDR_TORQUE_EN, 1, 1, 0};
  pkt[8] = calcChecksum(&pkt[2], 6);
  SERVO_SERIAL.write(pkt, 9);
}

uint16_t readPosition(uint8_t id) {
  uint8_t pkt[8] = {H1, H2, id, 4, INST_READ, ADDR_PRESENT_POS, 2, 0};
  pkt[7] = calcChecksum(&pkt[2], 5);
  SERVO_SERIAL.flush();
  SERVO_SERIAL.write(pkt, 8);
  unsigned long t0 = micros();
  while (SERVO_SERIAL.available() < 8) {
    if (micros() - t0 > 5000) return 0xFFFF;
  }
  uint8_t buf[16];
  int n = SERVO_SERIAL.readBytes(buf, SERVO_SERIAL.available());
  for (int i = 0; i <= n - 8; i++) {
    if (buf[i] == H1 && buf[i+1] == H2 && buf[i+2] == id) {
      if (buf[i+4] == 0 && buf[i+3] == 4) {
        return buf[i+5] | (buf[i+6] << 8);
      }
    }
  }
  return 0xFFFF;
}

void readAllPositions(uint16_t out[]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    out[i] = readPosition(MOTOR_IDS[i]);
  }
}

// ===================== Motor Configuration =====================
// Replicates SO101's configure() from lerobot/robots/so_follower/so_follower.py

void writeByte(uint8_t id, uint8_t addr, uint8_t value) {
  uint8_t pkt[8] = {H1, H2, id, 4, INST_WRITE, addr, value, 0};
  pkt[7] = calcChecksum(&pkt[2], 5);
  SERVO_SERIAL.write(pkt, 8);
  delayMicroseconds(300);  // brief delay between packets to avoid bus collision
}

void writeWord(uint8_t id, uint8_t addr, uint16_t value) {
  uint8_t pkt[9] = {H1, H2, id, 5, INST_WRITE, addr,
                    (uint8_t)(value & 0xFF), (uint8_t)(value >> 8), 0};
  pkt[8] = calcChecksum(&pkt[2], 6);
  SERVO_SERIAL.write(pkt, 9);
  delayMicroseconds(300);
}

void configureMotors() {
  // Disable torque before writing EPROM settings
  for (int i = 0; i < NUM_MOTORS; i++) {
    writeByte(MOTOR_IDS[i], ADDR_TORQUE_EN, 0);  // Torque off
    writeByte(MOTOR_IDS[i], 55, 0);              // Lock = 0
  }
  delay(50);

  // Common settings for all motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    uint8_t id = MOTOR_IDS[i];
    writeByte(id, 7, 0);     // Return_Delay_Time = 0  (2us response)
    writeByte(id, 33, 0);    // Operating_Mode = 0 (POSITION servo mode)
    writeByte(id, 21, 16);   // P_Coefficient = 16 (reduce shakiness)
    writeByte(id, 23, 0);    // I_Coefficient = 0
    writeByte(id, 22, 32);   // D_Coefficient = 32
    writeByte(id, 41, 254);  // Acceleration = 254 (max)
    writeByte(id, 85, 254);  // Maximum_Acceleration = 254
  }

  // Gripper-specific protection (motor index 5 = ID 6)
  uint8_t gripperId = MOTOR_IDS[5];
  writeWord(gripperId, 16, 500);  // Max_Torque_Limit = 500 (50%)
  writeWord(gripperId, 28, 250);  // Protection_Current = 250 (50%)
  writeByte(gripperId, 36, 25);   // Overload_Torque = 25 (25%)

  delay(50);

  // Re-enable torque
  for (int i = 0; i < NUM_MOTORS; i++) {
    writeByte(MOTOR_IDS[i], ADDR_TORQUE_EN, 1);
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(500);

  Serial.println("\n[DIAG] ===== ESP32 SO101 Bridge (DIAG) =====");
  Serial.println("[DIAG] Serial USB initialized OK");

  // Step 1: UART2
  Serial.print("[DIAG] Initializing UART2 (TX=");
  Serial.print(SERVO_TX_PIN);
  Serial.print(" RX=");
  Serial.print(SERVO_RX_PIN);
  Serial.println(")...");
  SERVO_SERIAL.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  delay(200);
  Serial.println("[DIAG] UART2 initialized OK");

  // Step 2: Configure motors (PID, acceleration, operating mode)
  Serial.println("[DIAG] Configuring all motors (PID, acceleration, mode)...");
  configureMotors();
  Serial.println("[DIAG] Motor configuration done");

  // Step 3: Read back one motor to verify bus is alive
  Serial.println("[DIAG] Probing motor ID 1...");
  uint16_t testPos = readPosition(1);
  if (testPos == 0xFFFF) {
    Serial.println("[DIAG] WARNING: Motor ID 1 did not respond. Check wiring/power.");
  } else {
    Serial.print("[DIAG] Motor ID 1 present position: ");
    Serial.println(testPos);
  }

  // Step 4: WiFi
#ifdef USE_AP_MODE
  Serial.print("[DIAG] Starting AP mode: ");
  Serial.println(AP_SSID);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.print("[DIAG] AP started. IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("[DIAG] Connect your PC to this WiFi, then use the IP above.");
#else
  Serial.print("[DIAG] Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int wifiRetries = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetries < 40) {
    delay(500);
    Serial.print(".");
    wifiRetries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[DIAG] WiFi Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[DIAG] ERROR: WiFi connection failed!");
    Serial.println("[DIAG] Possible causes:");
    Serial.println("  - Wrong SSID or PASSWORD");
    Serial.println("  - Chinese SSID (encoding mismatch)");
    Serial.println("  - Router too far / signal too weak");
    Serial.println("[DIAG] Fix: Uncomment '#define USE_AP_MODE' and re-flash.");
    Serial.println("[DIAG] Halting. Please fix WiFi config and reset.");
    while (true) { delay(1000); }
  }
#endif

  // Step 5: TCP servers
  serverCmd.begin();
  serverObs.begin();
  Serial.print("[DIAG] TCP CMD server on port ");
  Serial.println(PORT_CMD);
  Serial.print("[DIAG] TCP OBS server on port ");
  Serial.println(PORT_OBS);

  Serial.println("\n[DIAG] Setup complete. Waiting for PC client...\n");
}

// ===================== Main Loop =====================
void loop() {
  // Accept connections
  if (!clientCmd || !clientCmd.connected()) {
    WiFiClient newClient = serverCmd.available();
    if (newClient) {
      clientCmd = newClient;
      Serial.print("[DIAG] CMD client: ");
      Serial.println(clientCmd.remoteIP());
    }
  }
  if (!clientObs || !clientObs.connected()) {
    WiFiClient newClient = serverObs.available();
    if (newClient) {
      clientObs = newClient;
      Serial.print("[DIAG] OBS client: ");
      Serial.println(clientObs.remoteIP());
    }
  }

  // Process commands
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    Serial.print("[DIAG] RX: ");
    Serial.println(line);

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
      Serial.println("[DIAG] JSON parse error");
      return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
      Serial.println("[DIAG] Missing 'cmd'");
      return;
    }

    if (strcmp(cmd, "set_positions") == 0) {
      uint16_t goals[NUM_MOTORS];
      bool ok = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        int val = doc["positions"][MOTOR_NAMES[i]].as<int>();
        if (val < 0 || val > 4095) {
          Serial.print("[DIAG] Invalid pos for ");
          Serial.print(MOTOR_NAMES[i]);
          Serial.print(": ");
          Serial.println(val);
          ok = false;
        }
        goals[i] = (uint16_t)val;
      }
      if (ok) {
        syncWritePositions(goals);
        Serial.println("[DIAG] SYNC_WRITE sent");
      }
    }
    else if (strcmp(cmd, "get_obs") == 0) {
      uint16_t poses[NUM_MOTORS];
      readAllPositions(poses);
      StaticJsonDocument<512> resp;
      for (int i = 0; i < NUM_MOTORS; i++) {
        resp[MOTOR_NAMES[i]] = (poses[i] == 0xFFFF) ? -1 : (int)poses[i];
      }
      String out;
      serializeJson(resp, out);
      out += "\n";
      if (clientObs && clientObs.connected()) {
        clientObs.print(out);
      }
    }
    else {
      Serial.print("[DIAG] Unknown cmd: ");
      Serial.println(cmd);
    }
  }
}
