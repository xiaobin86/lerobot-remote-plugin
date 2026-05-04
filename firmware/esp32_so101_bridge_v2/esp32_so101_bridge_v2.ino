/**
 * ESP32 SO101 Remote Bridge v2 — Performance Optimized
 *
 * Key improvements over v1 (diag):
 *   1. syncWrite():  Updates Goal Position for all 6 servos in ONE bus packet
 *      (v1 sent 6 individual WritePosEx packets).
 *   2. syncRead():   Reads Present Position from all 6 servos in ONE bus round-trip
 *      (v1 called ReadPos 6 times sequentially, ~5ms each = ~30ms total).
 *      This restores get_obs latency from ~30-50ms to ~5ms, enabling smooth 30fps replay
 *      without having to skip observation on the PC side.
 *   3. TCP_NODELAY:  Disabled Nagle's algorithm on both TCP sockets so every
 *      JSON packet is flushed immediately (eliminates the "first few frames fast,
 *      then slow" TCP buffering artifact).
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
#include <SCServo.h>

// ===================== WiFi Configuration =====================
const char* WIFI_SSID     = "oppowifi";
const char* WIFI_PASSWORD = "asdfghjkl";

// Option B: Access Point mode
// Uncomment the line below to use AP mode:
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

// ===================== UART helpers =====================
void flushRxBuffer() {
  while (SERVO_SERIAL.available()) {
    SERVO_SERIAL.read();
  }
}

// ===================== Motor Configuration =====================
// Set acceleration once at startup (SRAM register, safe to write).
// ACC=254 matches original SOFollower configure() and gives instant response.
void configureMotors() {
  Serial.println("[V2] Configuring motors (ACC=254)...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_ACC, 254);
    delayMicroseconds(2000);
  }
  // Drain any status packets from the writeByte calls
  delayMicroseconds(5000);
  flushRxBuffer();
  Serial.println("[V2] Motor config done");
}

void enableAllTorque() {
  Serial.println("[V2] Enabling torque...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_TORQUE_ENABLE, 1);
    delayMicroseconds(2000);
  }
  delayMicroseconds(5000);
  flushRxBuffer();
  Serial.println("[V2] Torque enabled");
}

void disableAllTorque() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_TORQUE_ENABLE, 0);
    delayMicroseconds(2000);
  }
}

// ===================== Position Write (syncWrite) =====================
// Use syncWrite to update Goal Position (addr 42, 2 bytes) for ALL motors
// in a single bus packet. Much faster than 6 individual WritePosEx calls.

void syncWritePositions(const uint16_t positions[]) {
  // Defensive: clear any stale Status Packets before writing
  flushRxBuffer();

  // Build data array: 2 bytes per servo (low, high)
  uint8_t data[NUM_MOTORS * 2];
  for (int i = 0; i < NUM_MOTORS; i++) {
    data[i * 2]     = positions[i] & 0xFF;
    data[i * 2 + 1] = (positions[i] >> 8) & 0xFF;
  }

  // Single SYNC_WRITE packet to all motors at addr 42 (Goal Position)
  st.syncWrite((u8*)MOTOR_IDS, NUM_MOTORS, SMS_STS_GOAL_POSITION_L, data, 2);

  SERVO_SERIAL.flush();
}

// ===================== Position Read (syncRead) =====================
// Use syncRead to read Present Position (addr 56, 2 bytes) from ALL motors
// in a single bus round-trip. Restores ~5ms observation latency vs ~30ms
// for 6 sequential ReadPos calls.

bool syncReadPositions(int outPositions[]) {
  // Initialize syncRead buffer for 6 servos, 2 bytes each, 50ms timeout
  // TimeOut is in milliseconds (syncReadPacketTx calls readSCS with millis()-based timeout)
  st.syncReadBegin(NUM_MOTORS, 2, 50);

  // Send sync read request for Present Position (addr 56, 2 bytes)
  int rxLen = st.syncReadPacketTx((u8*)MOTOR_IDS, NUM_MOTORS, SMS_STS_PRESENT_POSITION_L, 2);
  if (rxLen <= 0) {
    st.syncReadEnd();
    return false;
  }

  // Extract each motor's position from the response buffer
  bool ok = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    uint8_t buf[2];
    int n = st.syncReadPacketRx(MOTOR_IDS[i], buf);
    if (n == 2) {
      outPositions[i] = buf[0] | (buf[1] << 8);
    } else {
      outPositions[i] = -1;  // read failure
      ok = false;
    }
  }

  st.syncReadEnd();
  return ok;
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n[V2] ===== ESP32 SO101 Bridge v2 (Optimized) =====");

  // Step 1: UART
  SERVO_SERIAL.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &SERVO_SERIAL;
  delay(500);
  Serial.println("[V2] UART2 initialized");

  // Step 2: Scan
  Serial.println("[V2] Scanning servos...");
  int found = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    int pos = st.ReadPos(MOTOR_IDS[i]);
    if (pos >= 0) {
      Serial.printf("[V2]   ID=%d pos=%d\n", MOTOR_IDS[i], pos);
      found++;
    } else {
      Serial.printf("[V2]   ID=%d no response\n", MOTOR_IDS[i]);
    }
  }
  Serial.printf("[V2] Found %d/%d servos\n", found, NUM_MOTORS);

  if (found == 0) {
    Serial.println("[V2] WARNING: No servos found! Check wiring/power.");
  }

  // Step 3: Configure (disable torque first, then set ACC, then re-enable)
  disableAllTorque();
  configureMotors();
  enableAllTorque();

  // Step 4: WiFi
#ifdef USE_AP_MODE
  Serial.print("[V2] AP mode: ");
  Serial.println(AP_SSID);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.print("[V2] AP IP: ");
  Serial.println(WiFi.softAPIP());
#else
  Serial.print("[V2] WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[V2] WiFi IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[V2] WiFi FAILED. Releasing torque and halting.");
    disableAllTorque();
    while (true) { delay(1000); }
  }
#endif

  // Step 5: TCP servers
  serverCmd.begin();
  serverObs.begin();
  Serial.printf("[V2] TCP CMD port %d, OBS port %d\n", PORT_CMD, PORT_OBS);

  Serial.println("[V2] Setup complete. Waiting for PC...\n");
}

// ===================== Main Loop =====================
void loop() {
  // Accept connections
  if (!clientCmd || !clientCmd.connected()) {
    if (clientWasConnected) {
      clientWasConnected = false;
      Serial.println("[V2] CMD client DISCONNECTED. Releasing torque.");
      disableAllTorque();
    }
    WiFiClient nc = serverCmd.available();
    if (nc) {
      clientCmd = nc;
      // CRITICAL: Disable Nagle to eliminate TCP buffering delay
      clientCmd.setNoDelay(true);
      clientWasConnected = true;
      Serial.println("[V2] CMD client connected: " + clientCmd.remoteIP().toString());
    }
  }
  if (!clientObs || !clientObs.connected()) {
    WiFiClient nc = serverObs.available();
    if (nc) {
      clientObs = nc;
      clientObs.setNoDelay(true);
      Serial.println("[V2] OBS client connected: " + clientObs.remoteIP().toString());
    }
  }

  // Process commands
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    Serial.print("[V2] RX: ");
    Serial.println(line);

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
      Serial.print("[V2] JSON error: ");
      Serial.println(err.c_str());
      return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
      Serial.println("[V2] Missing 'cmd'");
      return;
    }

    if (strcmp(cmd, "set_positions") == 0) {
      uint16_t goals[NUM_MOTORS];
      bool ok = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        int val = doc["positions"][MOTOR_NAMES[i]].as<int>();
        if (val < 0 || val > 4095) {
          Serial.printf("[V2] Invalid pos %s=%d\n", MOTOR_NAMES[i], val);
          ok = false;
        }
        goals[i] = (uint16_t)val;
      }
      if (ok) {
        syncWritePositions(goals);
        Serial.println("[V2] Positions sent via syncWrite");
      }
    }
    else if (strcmp(cmd, "get_obs") == 0) {
      int poses[NUM_MOTORS];
      bool ok = syncReadPositions(poses);

      StaticJsonDocument<512> resp;
      for (int i = 0; i < NUM_MOTORS; i++) {
        resp[MOTOR_NAMES[i]] = (poses[i] < 0) ? -1 : poses[i];
      }
      String out;
      serializeJson(resp, out);
      out += "\n";

      if (clientObs && clientObs.connected()) {
        clientObs.print(out);
      }

      if (!ok) {
        Serial.println("[V2] WARNING: Some positions failed syncRead");
      }
    }
    else if (strcmp(cmd, "release_torque") == 0) {
      Serial.println("[V2] Releasing torque...");
      disableAllTorque();
      StaticJsonDocument<128> resp;
      resp["success"] = true;
      resp["message"] = "torque_released";
      String out;
      serializeJson(resp, out);
      out += "\n";
      if (clientCmd && clientCmd.connected()) {
        clientCmd.print(out);
      }
    }
    else {
      Serial.print("[V2] Unknown cmd: ");
      Serial.println(cmd);
    }
  }
}
