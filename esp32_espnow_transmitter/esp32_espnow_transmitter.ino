#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

// =================== Configure ===================
uint8_t receiverMac[] = { 0xF0, 0xF5, 0xBD, 0x79, 0x36, 0xA4 }; // <-- set receiver's MAC
#define DEFAULT_ID  2     // used if "id" is not provided in JSON (0..999)

// =================== Message Format ==============
typedef struct __attribute__((packed)) {
  uint16_t id;        // 0..999
  uint8_t  leftAngle; // 0..180
  uint8_t  rightAngle;// 0..180
} AngleMessage;

// =================== Send Helper =================
void sendAngles(uint16_t id, uint8_t left, uint8_t right) {
  AngleMessage msg{ id, left, right };
  esp_err_t res = esp_now_send(receiverMac, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
  if (res == ESP_OK) {
    Serial.printf("Sent -> ID:%u  Left:%u  Right:%u\n", (unsigned)id, (unsigned)left, (unsigned)right);
  } else {
    Serial.printf("ESP-NOW send failed: %d\n", (int)res);
  }
}

// (optional) on-send callback for extra diagnostics
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Delivery status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register send callback (optional)
  esp_now_register_send_cb(onDataSent);

  // Add peer if needed
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(receiverMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
    }
  }

  Serial.println(F("Enter JSON like: {\"id\":42, \"left\":88, \"right\":91}"));
  Serial.println(F("Or omit id to use DEFAULT_ID"));
}

void loop() {
  static String input;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      input.trim();
      if (!input.isEmpty()) {
        // Enough for {"id":999,"left":180,"right":180}
        StaticJsonDocument<128> doc;
        DeserializationError err = deserializeJson(doc, input);
        if (err) {
          Serial.print("JSON parse error: ");
          Serial.println(err.c_str());
        } else {
          // Extract with defaults
          int id_i    = doc.containsKey("id") ? (int)doc["id"] : (int)DEFAULT_ID;
          int left_i  = doc.containsKey("left")  ? (int)doc["left"]  : -1;
          int right_i = doc.containsKey("right") ? (int)doc["right"] : -1;

          bool ok = true;
          if (id_i   < 0 || id_i   > 999) { Serial.println("id out of range (0–999)."); ok = false; }
          if (left_i < 0 || left_i > 180) { Serial.println("left out of range (0–180)."); ok = false; }
          if (right_i< 0 || right_i> 180) { Serial.println("right out of range (0–180)."); ok = false; }

          if (ok) {
            uint16_t id   = static_cast<uint16_t>(id_i);
            uint8_t  left = static_cast<uint8_t>(left_i);
            uint8_t  right= static_cast<uint8_t>(right_i);
            sendAngles(id, left, right);
          } else {
            Serial.println(F("Usage: {\"id\":0..999, \"left\":0..180, \"right\":0..180}"));
          }
        }
      }
      input = "";
    } else {
      input += c;
    }
  }
}
