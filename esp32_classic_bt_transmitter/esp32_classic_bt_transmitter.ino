#include "BluetoothSerial.h"
#include <ArduinoJson.h>
#include <string.h>

// --- Pins ---
const int STATUS_LED = 2;
const int BOOT_BUTTON = 0; // Standard ESP32 Boot Button

// --- Mode & Timing State ---
bool isPropellerMode = false; // Starts in BS2 mode (false)
unsigned long lastBlinkMillis = 0;
bool ledState = false;

// Mode frequencies (in ms for interval)
// BS2: 4Hz = 125ms toggle
// Propeller: 0.5Hz = 1000ms toggle
const unsigned long INTERVAL_BS2 = 125;
const unsigned long INTERVAL_PROP = 1000;

// Button Debounce
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 250; // ms to wait between toggles
bool lastButtonState = HIGH;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Classic SPP is not enabled for this ESP32 build.
#endif

BluetoothSerial SerialBT;

static const char* DEV_NAME = "ESP32_MASTER";
static const uint16_t DEFAULT_ID = 2;
static uint8_t HC05_ADDR[6] = {0x20, 0x17, 0x07, 0x25, 0x00, 0x71};
static const char* HC05_PIN = "1234";

// --- LED Logic ---
void updateStatusLED() {
  if (!SerialBT.connected()) {
    digitalWrite(STATUS_LED, LOW);
    return;
  }

  unsigned long currentMillis = millis();
  unsigned long interval = isPropellerMode ? INTERVAL_PROP : INTERVAL_BS2;

  if (currentMillis - lastBlinkMillis >= interval) {
    lastBlinkMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState);
  }
}

// --- Button Logic ---
void checkModeButton() {
  bool currentButtonState = digitalRead(BOOT_BUTTON);

  // If button is pressed (LOW) and it's been long enough since the last toggle
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
      isPropellerMode = !isPropellerMode; // Toggle mode
      lastDebounceTime = millis();
      
      Serial.print("Mode Switched to: ");
      Serial.println(isPropellerMode ? "PROPELLER (0.5Hz)" : "BS2 (4Hz)");
      
      // Reset LED timing so the new frequency starts immediately
      lastBlinkMillis = millis();
    }
  }
  lastButtonState = currentButtonState;
}

static bool parseGripper(const JsonVariantConst& v, int& outAngle, bool& provided) {
  provided = false; outAngle = -1;
  if (v.isNull()) return true;
  provided = true;
  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    if (strcasecmp(s, "open") == 0) { outAngle = 0; return true; }
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

static void sendPacket(uint16_t id, uint8_t left, uint8_t right, uint8_t gripper) {
  uint8_t pkt[7];
  pkt[0] = '!'; pkt[1] = '!';
  pkt[2] = (uint8_t)(id & 0xFF);
  pkt[3] = (uint8_t)((id >> 8) & 0xFF);
  pkt[4] = left; pkt[5] = right; pkt[6] = gripper;

  if (SerialBT.connected()) {
    if (!isPropellerMode) {
      // BS2 Mode - Send 3 times
      for(int i = 0; i < 3; i++) {
        SerialBT.write(pkt, sizeof(pkt));
        delay(5); 
      }
    } else {
      // Propeller Mode - Send 1 time
      SerialBT.write(pkt, sizeof(pkt));
    }
  }

  Serial.printf("[%s] Sent ID:%u L:%u R:%u G:%u\n", 
                isPropellerMode ? "PROP" : "BS2", id, pkt[4], pkt[5], pkt[6]);
}

static void ensureConnected() {
  if (SerialBT.connected()) return;
  static unsigned long lastConnectTry = 0;
  if (millis() - lastConnectTry > 5000) {
    Serial.println("Reconnecting to HC-05...");
    SerialBT.connect(HC05_ADDR);
    lastConnectTry = millis();
  }
}

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BOOT_BUTTON, INPUT_PULLUP); // Boot button usually has pullup, but safety first
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
  static String input;

  // Non-blocking tasks
  updateStatusLED();
  checkModeButton();
  ensureConnected();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      input.trim();
      if (!input.isEmpty()) {
        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, input);

        if (!err) {
          uint16_t id = doc["id"] | DEFAULT_ID;
          int lVal, rVal; bool lProv, rProv;
          parseAngleIfPresent(doc, "left", lVal, lProv);
          parseAngleIfPresent(doc, "right", rVal, rProv);

          uint8_t left = (lProv && lVal >= 0 && lVal <= 180) ? (uint8_t)lVal : 0xFF;
          uint8_t right = (rProv && rVal >= 0 && rVal <= 180) ? (uint8_t)rVal : 0xFF;

          uint8_t grip = 0xFF;
          if (doc.containsKey("gripper")) {
            int gVal; bool gProv;
            if (parseGripper(doc["gripper"], gVal, gProv) && gProv) grip = (uint8_t)gVal;
          }

          if (SerialBT.connected()) sendPacket(id, left, right, grip);
        }
      }
      input = "";
    } else {
      input += c;
    }
  }
}