/**
 * ESP32 SO101 Remote Bridge
 *
 * Receives joint position commands from PC (LeRobot replay) over TCP/WiFi
 * and forwards them to the Waveshare Feetech controller board via UART.
 *
 * Hardware:
 *   - ESP32 GPIO17 (TX) -> Waveshare RX
 *   - ESP32 GPIO18 (RX) -> Waveshare TX
 *   - 6x Feetech STS3215 servos on the bus (IDs 1-6)
 *
 * Protocol:
 *   - TCP port 8888: PC -> ESP32 (JSON commands)
 *   - TCP port 8889: ESP32 -> PC (JSON observations)
 *   - Feetech Protocol 0 (1 Mbps, 8N1)
 */

#include <WiFi.h>
#include <ArduinoJson.h>

// ===================== WiFi Configuration =====================
// Option A: Station mode (connect to existing router)
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Option B: Access Point mode (ESP32 creates its own network)
// Uncomment the following lines and comment out Option A to use AP mode:
// #define USE_AP_MODE
// const char* AP_SSID     = "SO101-ROBOT";
// const char* AP_PASSWORD = "12345678";   // min 8 chars, or empty for open
// const int   AP_CHANNEL  = 6;

// ===================== TCP Server Ports =====================
const int PORT_CMD = 8888;   // PC sends commands here
const int PORT_OBS = 8889;   // ESP32 sends observations here

WiFiServer serverCmd(PORT_CMD);
WiFiServer serverObs(PORT_OBS);
WiFiClient clientCmd;
WiFiClient clientObs;

// ===================== Feetech / UART Config =====================
#define SERVO_SERIAL Serial2
#define SERVO_TX_PIN  17
#define SERVO_RX_PIN  18
#define SERVO_BAUD    1000000   // 1 Mbps (default for STS3215)

const uint8_t MOTOR_IDS[] = {1, 2, 3, 4, 5, 6};
const char*   MOTOR_NAMES[] = {
  "shoulder_pan", "shoulder_lift", "elbow_flex",
  "wrist_flex", "wrist_roll", "gripper"
};
const int NUM_MOTORS = 6;

// Feetech Protocol 0 packet structure
#define H1 0xFF
#define H2 0xFF

#define INST_WRITE       0x03
#define INST_READ        0x02
#define INST_SYNC_WRITE  0x83

#define ADDR_TORQUE_EN   40
#define ADDR_GOAL_POS    42
#define ADDR_PRESENT_POS 56

// ===================== Protocol Helpers =====================
uint8_t calcChecksum(uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return ~sum;
}

/**
 * Send SYNC_WRITE to update Goal_Position for all motors in one packet.
 * This is the most efficient way to command multiple Feetech servos.
 */
void syncWritePositions(const uint16_t positions[]) {
  // Each motor: 1 byte ID + 2 bytes data
  uint8_t dataLen   = 2;
  uint8_t paramCnt  = 2 + NUM_MOTORS * (1 + dataLen); // addr(1) + len(1) + N*(id+data)
  uint8_t length    = paramCnt + 2;                   // + INST + CS

  uint8_t pkt[40];
  uint8_t idx = 0;
  pkt[idx++] = H1;
  pkt[idx++] = H2;
  pkt[idx++] = 0xFE;          // Broadcast ID
  pkt[idx++] = length;
  pkt[idx++] = INST_SYNC_WRITE;
  pkt[idx++] = ADDR_GOAL_POS; // Start address
  pkt[idx++] = dataLen;       // Bytes per motor

  for (int i = 0; i < NUM_MOTORS; i++) {
    pkt[idx++] = MOTOR_IDS[i];
    pkt[idx++] = positions[i] & 0xFF;
    pkt[idx++] = positions[i] >> 8;
  }

  pkt[idx] = calcChecksum(&pkt[2], 3 + paramCnt);
  SERVO_SERIAL.write(pkt, idx + 1);
}

/**
 * Enable torque for a single motor.
 */
void enableTorque(uint8_t id) {
  uint8_t pkt[9] = {H1, H2, id, 4, INST_WRITE, ADDR_TORQUE_EN, 1, 1, 0};
  pkt[8] = calcChecksum(&pkt[2], 6);
  SERVO_SERIAL.write(pkt, 9);
}

/**
 * Read Present_Position from a single motor.
 * Returns 0xFFFF on failure/timeout.
 */
uint16_t readPosition(uint8_t id) {
  // Build READ packet
  uint8_t pkt[8] = {H1, H2, id, 4, INST_READ, ADDR_PRESENT_POS, 2, 0};
  pkt[7] = calcChecksum(&pkt[2], 5);

  SERVO_SERIAL.flush();
  SERVO_SERIAL.write(pkt, 8);

  // Wait for status packet (8 bytes expected for 2-byte read)
  unsigned long t0 = micros();
  while (SERVO_SERIAL.available() < 8) {
    if (micros() - t0 > 5000) { // 5ms timeout
      return 0xFFFF;
    }
  }

  uint8_t buf[16];
  int n = SERVO_SERIAL.readBytes(buf, SERVO_SERIAL.available());

  // Parse status packet: FF FF ID LEN ERR PARAM_LO PARAM_HI CS
  for (int i = 0; i <= n - 8; i++) {
    if (buf[i] == H1 && buf[i+1] == H2 && buf[i+2] == id) {
      uint8_t len   = buf[i+3];
      uint8_t err   = buf[i+4];
      if (err == 0 && len == 4) {
        uint16_t pos = buf[i+5] | (buf[i+6] << 8);
        return pos;
      }
    }
  }
  return 0xFFFF;
}

/**
 * Read all motor positions sequentially.
 * Stores results in the provided array.
 */
void readAllPositions(uint16_t out[]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    out[i] = readPosition(MOTOR_IDS[i]);
  }
}

/**
 * Enable torque for all motors.
 */
void enableAllTorque() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    enableTorque(MOTOR_IDS[i]);
    delayMicroseconds(200);
  }
}

// ===================== WiFi Helpers =====================
void setupWiFi() {
#ifdef USE_AP_MODE
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.println("\n[WiFi] AP Mode Started");
  Serial.print("[WiFi] SSID: ");
  Serial.println(AP_SSID);
  Serial.print("[WiFi] IP:   ");
  Serial.println(WiFi.softAPIP());
#else
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("\n[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected");
  Serial.print("[WiFi] IP:   ");
  Serial.println(WiFi.localIP());
#endif
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(1000);
  Serial.println("\n===== ESP32 SO101 Remote Bridge =====");

  // Initialize UART2 for Feetech bus
  SERVO_SERIAL.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  delay(200);
  Serial.println("[UART] Serial2 initialized at 1 Mbps");

  // Enable torque on all servos so they hold position
  enableAllTorque();
  Serial.println("[SERVO] Torque enabled on all motors");

  // Start WiFi
  setupWiFi();

  // Start TCP servers
  serverCmd.begin();
  serverObs.begin();
  Serial.println("[TCP] Command server listening on port " + String(PORT_CMD));
  Serial.println("[TCP] Observation server listening on port " + String(PORT_OBS));
  Serial.println("\nWaiting for PC client connections...\n");
}

// ===================== Main Loop =====================
void loop() {
  // Accept incoming connections (non-blocking)
  if (!clientCmd || !clientCmd.connected()) {
    WiFiClient newClient = serverCmd.available();
    if (newClient) {
      clientCmd = newClient;
      Serial.println("[TCP] CMD client connected from " + clientCmd.remoteIP().toString());
    }
  }
  if (!clientObs || !clientObs.connected()) {
    WiFiClient newClient = serverObs.available();
    if (newClient) {
      clientObs = newClient;
      Serial.println("[TCP] OBS client connected from " + clientObs.remoteIP().toString());
    }
  }

  // Process commands from PC
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    Serial.println("[RX] " + line);

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
      Serial.println("[ERR] JSON parse failed");
      return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
      Serial.println("[ERR] Missing 'cmd' field");
      return;
    }

    if (strcmp(cmd, "set_positions") == 0) {
      uint16_t goals[NUM_MOTORS];
      bool ok = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        int val = doc["positions"][MOTOR_NAMES[i]].as<int>();
        if (val < 0 || val > 4095) {
          Serial.println("[WARN] Invalid position for " + String(MOTOR_NAMES[i]) + ": " + String(val));
          ok = false;
        }
        goals[i] = (uint16_t)val;
      }
      if (ok) {
        syncWritePositions(goals);
        Serial.println("[SERVO] Positions updated");
      }
    }
    else if (strcmp(cmd, "get_obs") == 0) {
      uint16_t poses[NUM_MOTORS];
      readAllPositions(poses);

      StaticJsonDocument<512> resp;
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (poses[i] == 0xFFFF) {
          resp[MOTOR_NAMES[i]] = -1; // indicate read failure
        } else {
          resp[MOTOR_NAMES[i]] = poses[i];
        }
      }

      String out;
      serializeJson(resp, out);
      out += "\n";

      if (clientObs && clientObs.connected()) {
        clientObs.print(out);
      }
    }
    else {
      Serial.println("[ERR] Unknown command: " + String(cmd));
    }
  }
}
