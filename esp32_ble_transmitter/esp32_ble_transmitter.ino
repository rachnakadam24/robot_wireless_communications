/*
  ESP32 BLE NUS Server — JSON -> 5-byte packet (id, left, right, gripper)
  Optional fields send 0xFF.

  Input examples (Serial Monitor @115200, "Newline"):
    {"id":12,"left":90,"right":95}                  // gripper -> 0xFF
    {"left":120}                                    // id -> DEFAULT_ID, right/gripper -> 0xFF
    {"id":7,"left":100,"right":100,"gripper":"open"}// gripper -> 180
    {"gripper":"close"}                              // gripper -> 0, left/right -> 0xFF

  Packet (little-endian):
    [0] id_lo
    [1] id_hi
    [2] left      (0..180 or 0xFF)
    [3] right     (0..180 or 0xFF)
    [4] gripper   (0..180 or 0xFF)  (convention: 180=open, 0=close)
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

static const char* DEV_NAME     = "ESP-UART";
static const uint16_t DEFAULT_ID = 2;

static const char* NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* NUS_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // optional
static const char* NUS_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

BLEServer*         pServer  = nullptr;
BLECharacteristic* pRXChar  = nullptr;
BLECharacteristic* pTXChar  = nullptr;
volatile bool      deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* p) override {
    deviceConnected = true;
    Serial.println("[ESP32] Central connected");
  }
  void onDisconnect(BLEServer* p) override {
    deviceConnected = false;
    Serial.println("[ESP32] Central disconnected, restarting advertising...");
    p->getAdvertising()->start();
  }
};

class RXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    std::string v = std::string(ch->getValue().c_str());
    if (v.empty()) return;
    Serial.print("[ESP32] From Pi: ");
    Serial.write((const uint8_t*)v.data(), v.size());
    Serial.println();
  }
};

static void notifyRaw(const uint8_t* data, size_t len) {
  if (!deviceConnected || !pTXChar) return;
  const size_t maxChunk = 20;
  size_t off = 0;
  while (off < len) {
    size_t n = (len - off > maxChunk) ? maxChunk : (len - off);
    pTXChar->setValue((uint8_t*)data + off, n);
    pTXChar->notify();
    off += n;
    delay(2);
  }
}

static void sendPacket(uint16_t id, uint8_t left, uint8_t right, uint8_t gripper) {
  uint8_t pkt[5];
  pkt[0] = (uint8_t)(id & 0xFF);
  pkt[1] = (uint8_t)((id >> 8) & 0xFF);
  pkt[2] = left;
  pkt[3] = right;
  pkt[4] = gripper;
  notifyRaw(pkt, sizeof(pkt));
  auto lab = [](uint8_t v)->const char* { return v == 0xFF ? "unset" : ""; };
  Serial.printf("Sent BLE -> ID:%u  Left:%u%s  Right:%u%s  Gripper:%u%s\n",
                (unsigned)id,
                (unsigned)pkt[2], lab(pkt[2]),
                (unsigned)pkt[3], lab(pkt[3]),
                (unsigned)pkt[4], lab(pkt[4]));
}

static bool parseGripper(const JsonVariantConst& v, int& outAngle, bool& provided) {
  provided = false;
  outAngle = -1;
  if (v.isNull()) return true; // not provided is fine
  provided = true;

  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    if (!s) { outAngle = -1; return false; }
    if (strcasecmp(s, "open") == 0)  { outAngle = 180; return true; }
    if (strcasecmp(s, "close") == 0) { outAngle = 0;   return true; }
    // numeric string?
    char* endp = nullptr;
    long val = strtol(s, &endp, 10);
    if (endp && *endp == '\0') { outAngle = (int)val; return true; }
    return false;
  }
  if (v.is<int>())  { outAngle = v.as<int>();  return true; }
  if (v.is<long>()) { outAngle = (int)v.as<long>(); return true; }
  return false;
}

static bool parseAngleIfPresent(const JsonDocument& doc, const char* key, int& outVal, bool& provided) {
  provided = false;
  outVal = -1;
  if (!doc.containsKey(key)) return true;  // not provided -> fine
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

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("\n[ESP32] BLE NUS server (JSON -> 5-byte packet with 0xFF sentinel)");
  Serial.println(F("Example: {\"id\":12,\"left\":90,\"right\":95}   // gripper -> 0xFF"));

  BLEDevice::init(DEV_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);

  pTXChar = pService->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTXChar->addDescriptor(new BLE2902());

  pRXChar = pService->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRXChar->setCallbacks(new RXCallbacks());

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  Serial.println("[ESP32] Advertising as 'ESP-UART' with NUS...");
}

void loop() {
  static String input;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      input.trim();
      if (!input.isEmpty()) {
        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, input);
        if (err) {
          Serial.print("JSON parse error: ");
          Serial.println(err.c_str());
        } else {
          // id (defaults to DEFAULT_ID if missing/invalid)
          uint16_t id = DEFAULT_ID;
          if (doc.containsKey("id")) {
            long idl = doc["id"].as<long>();
            if (idl >= 0 && idl <= 999) id = (uint16_t)idl;
            else Serial.println("Warning: id out of range (0–999). Using DEFAULT_ID.");
          }

          // left / right: optional → 0xFF if missing or invalid; clamp to 0..180 if present & valid
          int lVal, rVal;
          bool lProv, rProv;
          bool lOk = parseAngleIfPresent(doc, "left",  lVal, lProv);
          bool rOk = parseAngleIfPresent(doc, "right", rVal, rProv);

          uint8_t left  = 0xFF;
          uint8_t right = 0xFF;
          if (lProv && lOk && lVal >= 0 && lVal <= 180) left  = (uint8_t)lVal; else if (lProv) Serial.println("Warning: left invalid -> 0xFF");
          if (rProv && rOk && rVal >= 0 && rVal <= 180) right = (uint8_t)rVal; else if (rProv) Serial.println("Warning: right invalid -> 0xFF");

          // gripper: optional with mapping; -> 0xFF if missing/invalid
          int gVal; bool gProv;
          bool gOk = parseGripper(doc["gripper"], gVal, gProv);
          uint8_t grip = 0xFF;
          if (gProv && gOk && gVal >= 0 && gVal <= 180) grip = (uint8_t)gVal;
          else if (gProv) Serial.println("Warning: gripper invalid -> 0xFF");

          if (!deviceConnected) {
            Serial.println("No BLE central connected; cannot send.");
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
