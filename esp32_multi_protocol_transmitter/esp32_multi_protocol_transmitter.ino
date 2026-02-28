/*
  ESP32 Unified Transmitter: Wi-Fi AP (TCP) → ESP-NOW → BLE (NUS)
  ---------------------------------------------------------------
  - On boot: self-test (R→G→B→off), then Wi-Fi (RED) until long-press BOOT (~3s)
  - Long-press cycles modes: Wi-Fi (RED) → ESP-NOW (GREEN) → BLE (BLUE) → repeat

  JSON over Serial (115200):
    Wi-Fi  : any line is broadcast as-is (string)
    ESP-NOW: {"id":0..999,"left":0..180,"right":0..180}
    BLE    : {"id":0..999,"left":0..180,"right":0..180,"gripper":0..180|"open"|"close"}
             -> 5-byte packet [id_lo,id_hi,left|0xFF,right|0xFF,gripper|0xFF]

  Requires: ESP32 Arduino core, ArduinoJson, Adafruit NeoPixel
*/

#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

// BLE (from ESP32 core)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Onboard RGB: WS2812 / NeoPixel on GPIO 48 ----------
#include <Adafruit_NeoPixel.h>
#define RGB_PIN 48
#define RGB_NUM 1
Adafruit_NeoPixel rgb(RGB_NUM, RGB_PIN, NEO_GRB + NEO_KHZ800);
static inline void rgb_init() { rgb.begin(); rgb.clear(); rgb.show(); }
static inline void rgb_set(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}
static inline void rgb_self_test() {   // quick visible check
  rgb_set(64, 0, 0);   delay(200);
  rgb_set(0, 64, 0);   delay(200);
  rgb_set(0, 0, 64);   delay(200);
  rgb_set(0, 0, 0);
}

// ---------- Pins ----------
#ifndef BOOT_PIN
#define BOOT_PIN 0       // BOOT button: IO0 → GND when pressed (active-LOW)
#endif

// ---------- Mode state ----------
static const uint8_t MODE_WIFI   = 0;  // RED
static const uint8_t MODE_ESPNOW = 1;  // GREEN
static const uint8_t MODE_BLE    = 2;  // BLUE
volatile uint8_t currentMode = MODE_WIFI;

// Long-press detection (~3s)
const unsigned long LONG_PRESS_MS = 3000;
bool prevPressed = false;              // previous "pressed" state
unsigned long pressedSince = 0;
bool longPressArmed = false;           // set on press edge, cleared after 3s hold or on release

// Show mode color
static inline void setModeRgb(uint8_t m) {
  switch (m) {
    case MODE_WIFI:   rgb_set(255, 0,   0  ); break; // RED
    case MODE_ESPNOW: rgb_set(0,   255, 0  ); break; // GREEN
    case MODE_BLE:    rgb_set(0,   0,   255); break; // BLUE
  }
}

// ---------- Wi-Fi AP ----------
static const char* AP_SSID     = "ESP32S3_HOTSPOT";
static const char* AP_PASSWORD = "esp32pass123";
static const int   AP_CHANNEL  = 6;
static const uint16_t SERVER_PORT = 9000;
static const int      MAX_CLIENTS = 10;

WiFiServer server(SERVER_PORT);
WiFiClient clients[MAX_CLIENTS];

IPAddress apIP(192,168,4,1);
IPAddress apGW(192,168,4,1);
IPAddress apMASK(255,255,255,0);

String lineBuf;
unsigned long lastSerialRx = 0;
const unsigned long SERIAL_IDLE_FLUSH_MS = 20;
const size_t MAX_LINE_LEN = 4096;

void wifi_logClientsCount() {
  int n = 0;
  for (int i = 0; i < MAX_CLIENTS; i++) if (clients[i] && clients[i].connected()) n++;
  Serial.printf("[WiFi] Connected clients: %d\n", n);
}
void wifi_addClient(WiFiClient& c) {
  if (!c) return;
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!clients[i] || !clients[i].connected()) {
      clients[i].stop();
      clients[i] = c;
      clients[i].setNoDelay(true);
      Serial.printf("[WiFi] Client in slot %d  %s:%u\n",
                    i,
                    clients[i].remoteIP().toString().c_str(),
                    clients[i].remotePort());
      wifi_logClientsCount();
      return;
    }
  }
  Serial.println("[WiFi] Too many clients; rejecting.");
  c.stop();
}
void wifi_pruneClients() {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && !clients[i].connected()) {
      clients[i].stop();
      clients[i] = WiFiClient();
    }
  }
}
void wifi_broadcastLine(const String& s) {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) {
      clients[i].println(s);
    }
  }
}
void startWiFi() {
  Serial.println("\n[MODE] Wi-Fi AP");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGW, apMASK);
  WiFi.setSleep(false);
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, MAX_CLIENTS);
  if (!ok) {
    Serial.println("[WiFi] softAP start failed");
  } else {
    Serial.printf("[WiFi] AP SSID: %s  PASS: %s\n", AP_SSID, AP_PASSWORD);
    Serial.printf("[WiFi] AP IP:   %s  Port: %u\n",
                  WiFi.softAPIP().toString().c_str(), SERVER_PORT);
  }
  server.begin(SERVER_PORT);
  server.setNoDelay(true);
  for (int i = 0; i < MAX_CLIENTS; i++) { clients[i].stop(); clients[i] = WiFiClient(); }
}
void stopWiFi() {
  server.stop();
  for (int i = 0; i < MAX_CLIENTS; i++) { clients[i].stop(); clients[i] = WiFiClient(); }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("[WiFi] Stopped");
}

// ---------- ESP-NOW ----------
uint8_t receiverMac[] = { 0xF0, 0xF5, 0xBD, 0x79, 0x36, 0xA4 };  // peer MAC
#define DEFAULT_ID  2

typedef struct __attribute__((packed)) {
  uint16_t id;        // 0..999
  uint8_t  leftAngle; // 0..180
  uint8_t  rightAngle;// 0..180
} AngleMessage;

extern "C" {
  #include "esp_wifi_types.h"  // for wifi_tx_info_t (ESP32 core IDF v5.x)
}

void espnow_onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.printf("[ESPNOW] Delivery: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void espnow_sendAngles(uint16_t id, uint8_t left, uint8_t right) {
  AngleMessage msg{ id, left, right };
  esp_err_t res = esp_now_send(receiverMac, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
  if (res == ESP_OK) {
    Serial.printf("[ESPNOW] Sent -> ID:%u  Left:%u  Right:%u\n",
                  (unsigned)id, (unsigned)left, (unsigned)right);
  } else {
    Serial.printf("[ESPNOW] Send failed: %d\n", (int)res);
  }
}
bool startEspNow() {
  Serial.println("\n[MODE] ESP-NOW");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return false;
  }
  esp_now_register_send_cb(espnow_onDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESPNOW] add_peer failed");
    return false;
  }
  return true;
}
void stopEspNow() {
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  Serial.println("[ESPNOW] Stopped");
}

// ---------- BLE (Nordic UART Service style) ----------
static const char* DEV_NAME        = "ESP-UART";
static const char* NUS_SERVICE_UUID= "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* NUS_RX_UUID     = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
static const char* NUS_TX_UUID     = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

BLEServer*         pServer  = nullptr;
BLECharacteristic* pRXChar  = nullptr;
BLECharacteristic* pTXChar  = nullptr;
volatile bool      bleConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* p) override {
    bleConnected = true;
    Serial.println("[BLE] Central connected");
  }
  void onDisconnect(BLEServer* p) override {
    bleConnected = false;
    Serial.println("[BLE] Central disconnected, advertising…");
    p->getAdvertising()->start();
  }
};

static inline void ble_notifyRaw(const uint8_t* data, size_t len) {
  if (!bleConnected || !pTXChar) return;
  pTXChar->setValue((uint8_t*)data, len);
  pTXChar->notify();
}

bool startBLE() {
  Serial.println("\n[MODE] BLE (NUS)");
  WiFi.mode(WIFI_OFF);

  BLEDevice::init(DEV_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);
  pTXChar = pService->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTXChar->addDescriptor(new BLE2902());

  // RX not used by this sketch, but created for completeness
  pRXChar = pService->createCharacteristic(NUS_RX_UUID,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR);

  pService->start();
  pServer->getAdvertising()->addServiceUUID(NUS_SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("[BLE] Advertising ESP-UART");
  return true;
}
void stopBLE() {
  if (pServer) pServer->getAdvertising()->stop();
  BLEDevice::deinit(true);
  Serial.println("[BLE] Stopped");
}

// ---------- Helper ----------
static inline uint8_t clamp0_180(int v) {
  if (v < 0) return 0;
  if (v > 180) return 180;
  return (uint8_t)v;
}

// ---------- Mode switch ----------
void switchTo(uint8_t m) {
  // Tear down current
  switch (currentMode) {
    case MODE_WIFI:   stopWiFi();   break;
    case MODE_ESPNOW: stopEspNow(); break;
    case MODE_BLE:    stopBLE();    break;
  }
  currentMode = m;

  // Bring up new
  bool ok = true;
  switch (currentMode) {
    case MODE_WIFI:   startWiFi();          break;
    case MODE_ESPNOW: ok = startEspNow();   break;
    case MODE_BLE:    ok = startBLE();      break;
  }

  setModeRgb(currentMode);
  if (!ok) Serial.println("[MODE] Start failed for selected mode.");
}

// ---------- Arduino ----------
void setup() {
  pinMode(BOOT_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  delay(150);

  rgb_init();
  rgb_self_test();          // R→G→B→off
  setModeRgb(MODE_WIFI);    // show Wi-Fi color immediately
  switchTo(MODE_WIFI);      // start in Wi-Fi mode (red)
}

// CORRECTED long-press logic (proper press/release edges)
void handleButton() {
  bool pressed = (digitalRead(BOOT_PIN) == LOW); // active-LOW
  unsigned long now = millis();

  // Press edge: (now pressed) && (was not pressed)
  if (pressed && !prevPressed) {
    pressedSince = now;
    longPressArmed = true;
    // Serial.println("[UI] Press");
  }

  // While held: if armed and held long enough, mark as long-press
  if (pressed && longPressArmed && (now - pressedSince >= LONG_PRESS_MS)) {
    longPressArmed = false; // recognized; we'll switch on release edge
    // Serial.println("[UI] Long-press recognized");
  }

  // Release edge: (now not pressed) && (was pressed)
  if (!pressed && prevPressed) {
    if (!longPressArmed) {
      uint8_t next = (currentMode == MODE_WIFI) ? MODE_ESPNOW :
                     (currentMode == MODE_ESPNOW) ? MODE_BLE : MODE_WIFI;
      Serial.printf("[UI] Switching mode: %u -> %u\n", currentMode, next);
      switchTo(next);
    }
    // else it was a short press: ignore
  }

  prevPressed = pressed;
}

void loop() {
  handleButton();

  switch (currentMode) {
    case MODE_WIFI: {
      // Accept new TCP clients
      WiFiClient c = server.available();
      if (c) wifi_addClient(c);

      // Read Serial and broadcast by line (with short idle flush)
      while (Serial.available()) {
        char ch = (char)Serial.read();
        lastSerialRx = millis();
        if (ch == '\r' || ch == '\n') {
          if (lineBuf.length() > 0) {
            wifi_broadcastLine(lineBuf);
            lineBuf = "";
          }
        } else {
          if (lineBuf.length() < MAX_LINE_LEN) lineBuf += ch;
        }
      }
      if (lineBuf.length() && (millis() - lastSerialRx) >= SERIAL_IDLE_FLUSH_MS) {
        wifi_broadcastLine(lineBuf);
        lineBuf = "";
      }

      wifi_pruneClients();
      delay(1);
    } break;

    case MODE_ESPNOW: {
      // Parse JSON from Serial and send angles
      static String input;
      while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
          input.trim();
          if (!input.isEmpty()) {
            StaticJsonDocument<128> doc;
            DeserializationError err = deserializeJson(doc, input);
            if (err) {
              Serial.print("[ESPNOW] JSON error: "); Serial.println(err.c_str());
            } else {
              int id_i    = doc.containsKey("id")    ? (int)doc["id"]    : (int)DEFAULT_ID;
              int left_i  = doc.containsKey("left")  ? (int)doc["left"]  : -1;
              int right_i = doc.containsKey("right") ? (int)doc["right"] : -1;

              bool ok = true;
              if (id_i   < 0 || id_i   > 999) { Serial.println("[ESPNOW] id 0..999"); ok = false; }
              if (left_i < 0 || left_i > 180) { Serial.println("[ESPNOW] left 0..180"); ok = false; }
              if (right_i< 0 || right_i> 180) { Serial.println("[ESPNOW] right 0..180"); ok = false; }

              if (ok) espnow_sendAngles((uint16_t)id_i, clamp0_180(left_i), clamp0_180(right_i));
              else    Serial.println(F("[ESPNOW] Usage: {\"id\":0..999, \"left\":0..180, \"right\":0..180}"));
            }
          }
          input = "";
        } else {
          input += c;
          if (input.length() > 256) input = ""; // safety
        }
      }
      delay(2);
    } break;

    case MODE_BLE: {
      // Parse JSON → 5-byte packet and notify
      static String input;
      while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
          input.trim();
          if (!input.isEmpty()) {
            StaticJsonDocument<256> doc;
            DeserializationError err = deserializeJson(doc, input);
            if (err) {
              Serial.print("[BLE] JSON error: "); Serial.println(err.c_str());
            } else {
              uint16_t id = DEFAULT_ID;
              if (doc.containsKey("id")) {
                long idl = doc["id"].as<long>();
                if (idl >= 0 && idl <= 999) id = (uint16_t)idl;
                else Serial.println("[BLE] id out of range (0–999) → DEFAULT_ID");
              }

              uint8_t left  = 0xFF, right = 0xFF, grip = 0xFF;
              if (doc.containsKey("left")) {
                long lv = doc["left"].as<long>();
                if (lv >= 0 && lv <= 180) left = (uint8_t)lv; else Serial.println("[BLE] left invalid → 0xFF");
              }
              if (doc.containsKey("right")) {
                long rv = doc["right"].as<long>();
                if (rv >= 0 && rv <= 180) right = (uint8_t)rv; else Serial.println("[BLE] right invalid → 0xFF");
              }
              if (doc.containsKey("gripper")) {
                if (doc["gripper"].is<const char*>()) {
                  const char* s = doc["gripper"];
                  if      (!strcasecmp(s, "open"))  grip = 0;
                  else if (!strcasecmp(s, "close")) grip = 180;
                  else {
                    char* endp = nullptr;
                    long val = strtol(s, &endp, 10);
                    if (endp && *endp == '\0' && val >= 0 && val <= 180) grip = (uint8_t)val;
                    else Serial.println("[BLE] gripper invalid → 0xFF");
                  }
                } else {
                  long gv = doc["gripper"].as<long>();
                  if (gv >= 0 && gv <= 180) grip = (uint8_t)gv; else Serial.println("[BLE] gripper invalid → 0xFF");
                }
              }

              uint8_t pkt[5] = {
                (uint8_t)(id & 0xFF),
                (uint8_t)((id >> 8) & 0xFF),
                left, right, grip
              };
              ble_notifyRaw(pkt, sizeof(pkt));
              auto lab = [](uint8_t v){ return v==0xFF ? " (unset)" : ""; };
              Serial.printf("[BLE] Sent -> ID:%u  Left:%u%s  Right:%u%s  Grip:%u%s\n",
                            (unsigned)id, (unsigned)pkt[2], lab(pkt[2]),
                            (unsigned)pkt[3], lab(pkt[3]),
                            (unsigned)pkt[4], lab(pkt[4]));
            }
          }
          input = "";
        } else {
          input += c;
          if (input.length() > 256) input = ""; // safety
        }
      }
      delay(2);
    } break;
  }
}
