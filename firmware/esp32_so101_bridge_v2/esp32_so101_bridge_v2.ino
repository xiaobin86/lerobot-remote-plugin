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

// Leader mode config
#ifdef LEADER_MODE
const char* MODE_TAG = "[LEADER]";
const int   LEADER_STREAM_HZ = 30;  // How often to stream positions to PC
#endif

#ifdef FOLLOWER_MODE
const char* MODE_TAG = "[FOLLOWER]";
#endif

#include <WiFi.h>
#include <ArduinoJson.h>
#include <SCServo.h>
#include <TFT_eSPI.h>

// ===================== Display Configuration =====================
TFT_eSPI tft = TFT_eSPI();

// Layout constants (240x240 screen)
#define DISP_Y_TITLE    0
#define DISP_Y_IP      20
#define DISP_Y_STATUS  40
#define DISP_Y_SEP     60
#define DISP_Y_MSG     65
#define DISP_MSG_LINES 22   // (240-65) / 8 = ~21.8, use 8px font height
#define DISP_MSG_CHARS 40   // 240 / 6 = 40 chars per line with font 1

// Message ring buffer for scrolling log
#define MSG_BUF_SIZE 32
struct {
  char lines[MSG_BUF_SIZE][DISP_MSG_CHARS + 1];
  int head;
  int count;
} msgBuf;

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

// ===================== Display Helpers =====================
void initDisplay() {
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);  // Backlight on

  tft.init();
  tft.setSwapBytes(true);
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextSize(1);

  // Init message buffer
  msgBuf.head = 0;
  msgBuf.count = 0;
  for (int i = 0; i < MSG_BUF_SIZE; i++) {
    msgBuf.lines[i][0] = '\0';
  }

  // Title
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(0, DISP_Y_TITLE);
  tft.print("SO101 ");
  tft.print(MODE_TAG);

  // Separator line
  tft.drawLine(0, DISP_Y_SEP, 239, DISP_Y_SEP, TFT_DARKGREY);
}

void updateDisplayIP(const char* ip) {
  tft.fillRect(0, DISP_Y_IP, 240, 18, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(0, DISP_Y_IP);
  tft.print("IP: ");
  tft.print(ip);
}

void updateDisplayStatus(const char* status) {
  tft.fillRect(0, DISP_Y_STATUS, 240, 18, TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(0, DISP_Y_STATUS);
  tft.print(status);
}

void updateDisplayStatus(const char* label, const char* value, uint16_t color) {
  tft.fillRect(0, DISP_Y_STATUS, 240, 18, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(0, DISP_Y_STATUS);
  tft.print(label);
  tft.print(value);
}

void updateDisplayStatus(const char* status, uint16_t color) {
  tft.fillRect(0, DISP_Y_STATUS, 240, 18, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(0, DISP_Y_STATUS);
  tft.print(status);
}

void addDisplayMessage(const char* msg) {
  // Add to ring buffer
  int idx = (msgBuf.head + msgBuf.count) % MSG_BUF_SIZE;
  strncpy(msgBuf.lines[idx], msg, DISP_MSG_CHARS);
  msgBuf.lines[idx][DISP_MSG_CHARS] = '\0';

  if (msgBuf.count < MSG_BUF_SIZE) {
    msgBuf.count++;
  } else {
    msgBuf.head = (msgBuf.head + 1) % MSG_BUF_SIZE;
  }

  // Redraw message area
  tft.fillRect(0, DISP_Y_MSG, 240, 240 - DISP_Y_MSG, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int visibleLines = min(msgBuf.count, DISP_MSG_LINES);
  for (int i = 0; i < visibleLines; i++) {
    int bufIdx = (msgBuf.head + i) % MSG_BUF_SIZE;
    tft.setCursor(0, DISP_Y_MSG + i * 8);
    tft.print(msgBuf.lines[bufIdx]);
  }
}

void addDisplayMessage(String msg) {
  addDisplayMessage(msg.c_str());
}

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
  addDisplayMessage("Config motors...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    st.writeByte(MOTOR_IDS[i], SMS_STS_ACC, 254);
    delayMicroseconds(2000);
  }
  // Drain any status packets from the writeByte calls
  delayMicroseconds(5000);
  flushRxBuffer();
  Serial.println("[V2] Motor config done");
  addDisplayMessage("Motor config done");
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
  addDisplayMessage("Torque enabled");
}

void disableAllTorque() {
  addDisplayMessage("Disable torque...");
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

  // Init display first so we can show boot messages
  initDisplay();

  Serial.println("\n[V2] ===== ESP32 SO101 Bridge v2 (Optimized) =====");
  addDisplayMessage("SO101 Bridge v2");

  // Step 1: UART
  SERVO_SERIAL.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &SERVO_SERIAL;
  delay(500);
  Serial.println("[V2] UART2 initialized");
  addDisplayMessage("UART2 OK");

  // Step 2: Scan
  Serial.println("[V2] Scanning servos...");
  addDisplayMessage("Scan servos...");
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
  addDisplayMessage((String("Found ") + found + "/" + NUM_MOTORS + " servos").c_str());

  if (found == 0) {
    Serial.println("[V2] WARNING: No servos found! Check wiring/power.");
    addDisplayMessage("WARN: No servos!");
  }

  // Step 3: Configure motors (always needed for both modes)
  disableAllTorque();
  configureMotors();

#ifdef LEADER_MODE
  // Leader: torque stays OFF so human can move the arm freely
  Serial.println("[LEADER] Torque DISABLED -- arm is free to move");
  addDisplayMessage("LEADER: torque OFF");
#else
  // Follower: torque ON so arm holds position and executes commands
  enableAllTorque();
#endif

  // Step 4: WiFi
#ifdef USE_AP_MODE
  Serial.print(MODE_TAG);
  Serial.print(" AP mode: ");
  Serial.println(AP_SSID);
  addDisplayMessage((String("AP: ") + AP_SSID).c_str());
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.print(MODE_TAG);
  Serial.print(" AP IP: ");
  Serial.println(WiFi.softAPIP());
  updateDisplayIP(WiFi.softAPIP().toString().c_str());
  updateDisplayStatus("WiFi: AP Mode", TFT_YELLOW);
#else
  Serial.print(MODE_TAG);
  Serial.print(" WiFi: ");
  Serial.println(WIFI_SSID);
  addDisplayMessage((String("WiFi: ") + WIFI_SSID).c_str());
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(MODE_TAG);
    Serial.print(" WiFi IP: ");
    Serial.println(WiFi.localIP());
    updateDisplayIP(WiFi.localIP().toString().c_str());
    updateDisplayStatus("WiFi: Connected", TFT_GREEN);
    addDisplayMessage((String("IP: ") + WiFi.localIP().toString()).c_str());
  } else {
    Serial.println("[FOLLOWER] WiFi FAILED. Releasing torque and halting.");
    addDisplayMessage("WiFi FAILED!");
    updateDisplayStatus("WiFi: FAILED", TFT_RED);
    disableAllTorque();
    while (true) { delay(1000); }
  }
#endif

  // Step 5: TCP servers
  serverCmd.begin();
  serverObs.begin();
  Serial.printf("%s TCP CMD port %d, OBS port %d\n", MODE_TAG, PORT_CMD, PORT_OBS);
  addDisplayMessage((String("TCP ") + PORT_CMD + "/" + PORT_OBS).c_str());

  Serial.print(MODE_TAG);
  Serial.println(" Setup complete. Waiting for PC...\n");
  addDisplayMessage("Setup done. Wait PC...");
}

// ===================== Send Observation Helper =====================
void sendObservation() {
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
    Serial.print(MODE_TAG);
    Serial.println(" WARNING: Some positions failed syncRead");
  }
}

// ===================== Main Loop =====================
#ifdef LEADER_MODE
unsigned long lastStreamMs = 0;
const unsigned long streamIntervalMs = 1000 / LEADER_STREAM_HZ;  // ~33ms for 30Hz
#endif

void loop() {
  // Accept connections
  if (!clientCmd || !clientCmd.connected()) {
    if (clientWasConnected) {
      clientWasConnected = false;
      Serial.print(MODE_TAG);
      Serial.println(" CMD client DISCONNECTED.");
      addDisplayMessage("CMD disconnected");
      updateDisplayStatus("Status: No PC", TFT_ORANGE);
#ifdef FOLLOWER_MODE
      disableAllTorque();
#endif
    }
    WiFiClient nc = serverCmd.available();
    if (nc) {
      clientCmd = nc;
      clientCmd.setNoDelay(true);
      clientWasConnected = true;
      Serial.print(MODE_TAG);
      Serial.println(" CMD client connected: " + clientCmd.remoteIP().toString());
      addDisplayMessage((String("CMD: ") + clientCmd.remoteIP().toString()).c_str());
      updateDisplayStatus("Status: PC Connected", TFT_GREEN);
    }
  }
  if (!clientObs || !clientObs.connected()) {
    WiFiClient nc = serverObs.available();
    if (nc) {
      clientObs = nc;
      clientObs.setNoDelay(true);
      Serial.print(MODE_TAG);
      Serial.println(" OBS client connected: " + clientObs.remoteIP().toString());
      addDisplayMessage((String("OBS: ") + clientObs.remoteIP().toString()).c_str());
    }
  }

#ifdef LEADER_MODE
  // ====== LEADER MODE ======
  // Stream positions to PC at fixed rate (LEADER_STREAM_HZ)
  unsigned long now = millis();
  if (clientObs && clientObs.connected() && (now - lastStreamMs >= streamIntervalMs)) {
    lastStreamMs = now;
    sendObservation();
    addDisplayMessage("TX: positions");
  }

  // Also respond to explicit get_obs requests
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) return;

    const char* cmd = doc["cmd"];
    if (cmd && strcmp(cmd, "get_obs") == 0) {
      sendObservation();
      addDisplayMessage("RX: get_obs");
    }
    // Leader ignores set_positions and release_torque
  }
#else
  // ====== FOLLOWER MODE ======
  // Process commands
  if (clientCmd && clientCmd.connected() && clientCmd.available()) {
    String line = clientCmd.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    Serial.print(MODE_TAG);
    Serial.print(" RX: ");
    Serial.println(line);
    addDisplayMessage((String("RX: ") + line).c_str());

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
      Serial.print(MODE_TAG);
      Serial.print(" JSON error: ");
      Serial.println(err.c_str());
      addDisplayMessage((String("JSON err: ") + err.c_str()).c_str());
      return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
      Serial.print(MODE_TAG);
      Serial.println(" Missing 'cmd'");
      addDisplayMessage("Missing cmd");
      return;
    }

    if (strcmp(cmd, "set_positions") == 0) {
      uint16_t goals[NUM_MOTORS];
      bool ok = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        int val = doc["positions"][MOTOR_NAMES[i]].as<int>();
        if (val < 0 || val > 4095) {
          Serial.printf("%s Invalid pos %s=%d\n", MODE_TAG, MOTOR_NAMES[i], val);
          ok = false;
        }
        goals[i] = (uint16_t)val;
      }
      if (ok) {
        syncWritePositions(goals);
        Serial.print(MODE_TAG);
        Serial.println(" Positions sent via syncWrite");
        addDisplayMessage("TX: positions");
      }
    }
    else if (strcmp(cmd, "get_obs") == 0) {
      sendObservation();
      addDisplayMessage("TX: obs");
    }
    else if (strcmp(cmd, "release_torque") == 0) {
      Serial.print(MODE_TAG);
      Serial.println(" Releasing torque...");
      addDisplayMessage("Release torque...");
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
      Serial.print(MODE_TAG);
      Serial.print(" Unknown cmd: ");
      Serial.println(cmd);
      addDisplayMessage((String("Unknown: ") + cmd).c_str());
    }
  }
#endif
}
