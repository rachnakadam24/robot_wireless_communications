#include "BluetoothSerial.h"
#include <ArduinoJson.h>
#include <string.h>

/*
  ESP32 Classic BT SPP MASTER -> HC-05 SLAVE
  JSON (Serial Monitor) -> 5-byte packet (id, left, right, gripper)
  Optional fields send 0xFF.

  Input examples (Serial Monitor @115200, Newline):
    {"id":12,"left":90,"right":95}                   // gripper -> 0xFF
    {"left":120}                                     // id -> DEFAULT_ID, right/gripper -> 0xFF
    {"id":7,"left":100,"right":100,"gripper":"open"} // gripper -> 180
    {"gripper":"close"}                              // gripper -> 0, left/right -> 0xFF

  Packet (little-endian):
    [0] id_lo
    [1] id_hi
    [2] left      (0..180 or 0xFF)
    [3] right     (0..180 or 0xFF)
    [4] gripper   (0..180 or 0xFF) (convention: 180=open, 0=close)
*/

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Classic SPP is not enabled for this ESP32 build.
#endif

BluetoothSerial SerialBT;

static const char*   DEV_NAME    = "ESP32_MASTER";
static const uint16_t DEFAULT_ID = 2;

// Your HC-05: +ADDR:2017:7:250071 -> 20:17:07:25:00:71
static uint8_t HC05_ADDR[6] = {0x20, 0x17, 0x07, 0x25, 0x00, 0x71};

// If your HC-05 PIN is default 1234 (or 0000), set it here
static const char* HC05_PIN = "1234";

static bool parseGripper(const JsonVariantConst& v, int& outAngle, bool& provided) {
  provided = false;
  outAngle = -1;

  if (v.isNull()) return true; // Not provided

  provided = true;

  // Handle Strings ("open", "close", "90")
  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    if (strcasecmp(s, "open") == 0)  { outAngle = 0; return true; }
    if (strcasecmp(s, "close") == 0) { outAngle = 180;   return true; }
    
    // Numeric string fallback
    outAngle = atoi(s); 
    return true;
  }

  // Handle Raw Numbers (180, 0, 90)
  if (v.is<int>() || v.is<long>()) {
    outAngle = v.as<int>();
    return true;
  }

  return false;
}

static bool parseAngleIfPresent(const JsonDocument& doc, const char* key, int& outVal, bool& provided) {
  provided = false;
  outVal = -1;

  if (!doc.containsKey(key)) return true; // not provided

  provided = true;

  if (doc[key].is<int>())  { outVal = (int)doc[key];  return true; }
  if (doc[key].is<long>()) { outVal = (int)doc[key];  return true; }

  // numeric string fallback
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
  // Packet size is now 7 bytes: [H1] [H2] [idLo] [idHi] [left] [right] [gripper]
  uint8_t pkt[7];
  pkt[0] = '!'; pkt[1] = '!';
  pkt[2] = (uint8_t)(id & 0xFF);
  pkt[3] = (uint8_t)((id >> 8) & 0xFF);
  pkt[4] = left; pkt[5] = right; pkt[6] = gripper;

  // Send all 7 bytes over SPP
  if (SerialBT.connected()) {
    // ---- BS2 - Send data 3 times ----
    // for(int i = 0; i < 3; i++) {
    //   SerialBT.write(pkt, sizeof(pkt));
    //   delay(5); // Give the BS2 a moment to process if it missed the first one
    // }
    
    // ---- Propeller - Send data 1 time ----
    SerialBT.write(pkt, sizeof(pkt));
  }

  auto lab = [](uint8_t v)->const char* { return (v == 0xFF) ? " (unset)" : ""; };

  // Updated Printf to show the 7-byte structure
  Serial.printf("Sent SPP -> ID:%u  Left:%u%s  Right:%u%s  Gripper:%u%s  [bytes: %02X %02X %02X %02X %02X %02X %02X]\n",
                (unsigned)id,
                (unsigned)pkt[4], lab(pkt[4]),
                (unsigned)pkt[5], lab(pkt[5]),
                (unsigned)pkt[6], lab(pkt[6]),
                pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5], pkt[6]);
}

static void ensureConnected() {
  if (SerialBT.connected()) return;

  Serial.println("Not connected to HC-05. Connecting...");
  bool ok = SerialBT.connect(HC05_ADDR);

  if (ok) Serial.println("Connected to HC-05!");
  else    Serial.println("Connect failed. Will retry...");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n[ESP32] Classic BT SPP MASTER (JSON -> 5-byte packet, 0xFF sentinel)");
  Serial.println(R"(Example: {"id":12,"left":90,"right":95})");

  // Start BT in master mode
  if (!SerialBT.begin(DEV_NAME, true)) {
    Serial.println("Bluetooth init failed!");
    while (1) delay(100);
  }

  // Newer ESP32 core requires PIN + length
  SerialBT.setPin(HC05_PIN, (uint8_t)strlen(HC05_PIN));

  ensureConnected();
}

void loop() {
  static String input;

  // Keep connection alive / auto-reconnect
  if (!SerialBT.connected()) {
    ensureConnected();
    delay(1000);
  }

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      input.trim();
      if (!input.isEmpty()) {
        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, input);

        if (err) {
          Serial.print("JSON parse error: ");
          Serial.println(err.c_str());
        } else {
          // id default
          uint16_t id = DEFAULT_ID;
          if (doc.containsKey("id")) {
            long idl = doc["id"].as<long>();
            if (idl >= 0 && idl <= 999) id = (uint16_t)idl;
            else Serial.println("Warning: id out of range (0–999). Using DEFAULT_ID.");
          }

          // left/right optional
          int lVal, rVal;
          bool lProv, rProv;
          bool lOk = parseAngleIfPresent(doc, "left",  lVal, lProv);
          bool rOk = parseAngleIfPresent(doc, "right", rVal, rProv);

          uint8_t left  = 0xFF;
          uint8_t right = 0xFF;

          if (lProv && lOk && lVal >= 0 && lVal <= 180) left  = (uint8_t)lVal;
          else if (lProv) Serial.println("Warning: left invalid -> 0xFF");

          if (rProv && rOk && rVal >= 0 && rVal <= 180) right = (uint8_t)rVal;
          else if (rProv) Serial.println("Warning: right invalid -> 0xFF");

          // gripper optional
          uint8_t grip = 0xFF;
          if (doc.containsKey("gripper")) {
            int gVal; bool gProv;
            bool gOk = parseGripper(doc["gripper"], gVal, gProv);
            if (gProv && gOk && gVal >= 0 && gVal <= 180) grip = (uint8_t)gVal;
            else Serial.println("Warning: gripper invalid -> 0xFF");
          }

          if (!SerialBT.connected()) {
            Serial.println("No SPP link; cannot send.");
          } else {
            sendPacket(id, left, right, grip);
          }
        }
      }
      input = "";
    } else {
      input += c;
      if (input.length() > 256) {
        Serial.println("Input too long; clearing.");
        input = "";
      }
    }
  }

  delay(5);
}