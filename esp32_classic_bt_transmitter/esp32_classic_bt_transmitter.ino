#include "BluetoothSerial.h"
#include <ArduinoJson.h>
#include <string.h>

// --- Pins ---
const int STATUS_LED = 2;
const int BOOT_BUTTON = 0;

// --- Mode & Timing State ---
bool isPropellerMode = false;
unsigned long lastBlinkMillis = 0;
bool ledState = false;

const unsigned long INTERVAL_BS2  = 125;
const unsigned long INTERVAL_PROP = 1000;

// Button Debounce
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 250;
bool lastButtonState = HIGH;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Classic SPP is not enabled for this ESP32 build.
#endif

BluetoothSerial SerialBT;

static const char*    DEV_NAME   = "ESP32_MASTER";
static const uint16_t DEFAULT_ID = 2;
static uint8_t        HC05_ADDR[6] = {0x20, 0x17, 0x07, 0x25, 0x00, 0x71};
static const char*    HC05_PIN   = "1234";

// --- Reconnect state ---
static unsigned long lastConnectTry  = 0;
static const unsigned long RECONNECT_INTERVAL = 5000; // ms between attempts
static const int MAX_RETRIES = 10;        // after this many failures, do a full BT restart
static int reconnectAttempts = 0;

// --- Fixed serial input buffer (no heap alloc) ---
static char   inputBuf[256];
static uint8_t inputLen = 0;

// --- BS2 repeat pacing (non-blocking) ---
static uint8_t  pendingPkt[7];
static int      pendingRepeats   = 0;
static unsigned long lastPktSent = 0;
static const unsigned long PKT_SPACING_MS = 5;

// --- LED Logic ---
void updateStatusLED() {
  if (!SerialBT.connected()) {
    digitalWrite(STATUS_LED, LOW);
    return;
  }
  unsigned long now = millis();
  unsigned long interval = isPropellerMode ? INTERVAL_PROP : INTERVAL_BS2;
  if (now - lastBlinkMillis >= interval) {
    lastBlinkMillis = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState);
  }
}

// --- Button Logic ---
void checkModeButton() {
  bool cur = digitalRead(BOOT_BUTTON);
  if (cur == LOW && lastButtonState == HIGH) {
    if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
      isPropellerMode = !isPropellerMode;
      lastDebounceTime = millis();
      Serial.print("Mode Switched to: ");
      Serial.println(isPropellerMode ? "PROPELLER (0.5Hz)" : "BS2 (4Hz)");
      lastBlinkMillis = millis();
    }
  }
  lastButtonState = cur;
}

// --- Parsers (unchanged) ---
static bool parseGripper(const JsonVariantConst& v, int& outAngle, bool& provided) {
  provided = false; outAngle = -1;
  if (v.isNull()) return true;
  provided = true;
  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    if (strcasecmp(s, "open")  == 0) { outAngle = 0;   return true; }
    if (strcasecmp(s, "close") == 0) { outAngle = 180; return true; }
    outAngle = atoi(s); return true;
  }
  if (v.is<int>() || v.is<long>()) { outAngle = v.as<int>(); return true; }
  return false;
}

static bool parseAngleIfPresent(const JsonDocument& doc, const char* key, int& outVal, bool& provided) {
  provided = false; outVal = -1;
  if (!doc.containsKey(key)) return true;
  provided = true;
  if (doc[key].is<int>() || doc[key].is<long>()) { outVal = (int)doc[key]; return true; }
  if (doc[key].is<const char*>()) {
    const char* s = doc[key].as<const char*>();
    if (!s) return false;
    char* endp = nullptr;
    long val = strtol(s, &endp, 10);
    if (endp && *endp == '\0') { outVal = (int)val; return true; }
  }
  return false;
}

// --- Non-blocking packet send pump ---
// Call every loop(). Drains pending BS2 repeats without blocking.
void pumpPendingPacket() {
  if (pendingRepeats <= 0) return;
  if (!SerialBT.connected()) { pendingRepeats = 0; return; }
  if (millis() - lastPktSent >= PKT_SPACING_MS) {
    SerialBT.write(pendingPkt, sizeof(pendingPkt));
    lastPktSent = millis();
    pendingRepeats--;
  }
}

static void sendPacket(uint16_t id, uint8_t left, uint8_t right, uint8_t gripper) {
  uint8_t pkt[7];
  pkt[0] = '!'; pkt[1] = '!';
  pkt[2] = (uint8_t)(id & 0xFF);
  pkt[3] = (uint8_t)((id >> 8) & 0xFF);
  pkt[4] = left; pkt[5] = right; pkt[6] = gripper;

  if (!SerialBT.connected()) return;

  if (!isPropellerMode) {
    // Queue 3 sends; first one fires immediately, rest handled by pumpPendingPacket()
    memcpy(pendingPkt, pkt, sizeof(pkt));
    SerialBT.write(pendingPkt, sizeof(pendingPkt));
    lastPktSent = millis();
    pendingRepeats = 2; // 2 more remaining
  } else {
    SerialBT.write(pkt, sizeof(pkt));
  }

  Serial.printf("[%s] Sent ID:%u L:%u R:%u G:%u\n",
                isPropellerMode ? "PROP" : "BS2", id, pkt[4], pkt[5], pkt[6]);
}

// --- Reconnect with full BT restart after MAX_RETRIES ---
static void ensureConnected() {
  if (SerialBT.connected()) {
    reconnectAttempts = 0; // reset counter on successful connection
    return;
  }

  unsigned long now = millis();
  if (now - lastConnectTry < RECONNECT_INTERVAL) return;
  lastConnectTry = now;

  if (reconnectAttempts >= MAX_RETRIES) {
    // Full BT stack restart to reclaim memory
    Serial.println("Max retries reached. Restarting BT stack...");
    SerialBT.end();
    delay(500);
    if (!SerialBT.begin(DEV_NAME, true)) {
      Serial.println("BT restart failed! Will retry...");
    } else {
      SerialBT.setPin(HC05_PIN, (uint8_t)strlen(HC05_PIN));
      Serial.println("BT stack restarted.");
    }
    reconnectAttempts = 0;
    return; // skip connect this cycle, let next cycle try
  }

  // Always disconnect cleanly before connecting to free internal BT resources
  SerialBT.disconnect();
  delay(100); // give stack time to clean up

  Serial.printf("Reconnecting to HC-05 (attempt %d/%d)...\n",
                reconnectAttempts + 1, MAX_RETRIES);
  SerialBT.connect(HC05_ADDR);
  reconnectAttempts++;
}

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  digitalWrite(STATUS_LED, LOW);

  Serial.begin(115200);
  delay(1000);

  if (!SerialBT.begin(DEV_NAME, true)) {
    Serial.println("Bluetooth init failed!");
    while (1) delay(100);
  }

  SerialBT.setPin(HC05_PIN, (uint8_t)strlen(HC05_PIN));
  Serial.println("System Ready. Initial Mode: BS2 (4Hz)");
}

void loop() {
  updateStatusLED();
  checkModeButton();
  ensureConnected();
  pumpPendingPacket(); // non-blocking BS2 repeat pump

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      // Trim trailing spaces in-place
      while (inputLen > 0 && inputBuf[inputLen - 1] == ' ') inputLen--;
      inputBuf[inputLen] = '\0';

      if (inputLen > 0) {
        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, inputBuf, inputLen);

        if (!err) {
          uint16_t id = doc["id"] | DEFAULT_ID;
          int lVal, rVal; bool lProv, rProv;
          parseAngleIfPresent(doc, "left",  lVal, lProv);
          parseAngleIfPresent(doc, "right", rVal, rProv);

          uint8_t left  = (lProv && lVal >= 0 && lVal <= 180) ? (uint8_t)lVal : 0xFF;
          uint8_t right = (rProv && rVal >= 0 && rVal <= 180) ? (uint8_t)rVal : 0xFF;

          uint8_t grip = 0xFF;
          if (doc.containsKey("gripper")) {
            int gVal; bool gProv;
            if (parseGripper(doc["gripper"], gVal, gProv) && gProv) grip = (uint8_t)gVal;
          }

          if (SerialBT.connected()) sendPacket(id, left, right, grip);
        } else {
          Serial.printf("JSON parse error: %s\n", err.c_str());
        }
      }
      inputLen = 0; // reset buffer
    } else {
      // Prevent buffer overflow
      if (inputLen < sizeof(inputBuf) - 1) {
        inputBuf[inputLen++] = c;
      }
      // silently drop characters if buffer full (malformed/too-long input)
    }
  }
}
