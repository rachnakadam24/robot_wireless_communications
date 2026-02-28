#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// =================== Robot selector ===================
#define ROBOT_ID 782        // <-- set this robot's ID: 0..999

// =================== LCD (UART1) ======================
// LCD on UART1 (TX = GPIO 5, RX = GPIO 8)
HardwareSerial lcdSerial(1);

// =================== Servo Pins =======================
const int leftPin  = 10;
const int rightPin = 9;

Servo leftServo;
Servo rightServo;

// =================== Message Format ===================
// id up to 999 => use 16-bit; angles 0..180 (stored in uint8_t)
typedef struct __attribute__((packed)) {
  uint16_t id;        // 0..999 (fits in 16 bits)
  uint8_t  leftAngle; // 0..180
  uint8_t  rightAngle;// 0..180
} AngleMessage;

// =================== Display State ====================
volatile int currentLeft   = -1;
volatile int currentRight  = -1;
volatile int lastMsgId     = -1;   // last received ID (for debug on LCD)

// =================== Timing ===========================
unsigned long lastDisplayTime = 0;
// Idle refresh rate: ONLY updates on this cadence
const unsigned long displayInterval = 2000;  // 2 seconds

// -------------------- LCD utils --------------------
static inline void lcdClear() {
  lcdSerial.write(0x0C);
  delay(5);
}

static inline void printToLCD(const String &text, uint8_t row, uint8_t col) {
  if (row < 1 || row > 2 || col > 15) return;
  // 16x2 serial LCDs often map row2 base at 0x14 (20 decimal)
  uint8_t pos = 128 + ((row == 1) ? 0 : 20) + col;
  lcdSerial.write(pos);
  delay(5);
  if (text.length() > 16) {
    lcdSerial.print(text.substring(0, 16));
  } else {
    lcdSerial.print(text);
  }
}

String getMacAddress() {
  uint8_t baseMac[6];
  if (esp_wifi_get_mac(WIFI_IF_STA, baseMac) != ESP_OK) return "MAC ERR";
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          baseMac[0], baseMac[1], baseMac[2],
          baseMac[3], baseMac[4], baseMac[5]);
  return String(macStr);
}

// -------------------- ESP-NOW RX --------------------
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(AngleMessage)) {
    return; // Unexpected payload size
  }

  AngleMessage msg;
  memcpy(&msg, incomingData, sizeof(msg));
  lastMsgId = (int)msg.id;

  // Validate ranges
  if (msg.leftAngle > 180 || msg.rightAngle > 180) {
    return;
  }

  // Only act if the ID matches this robot
  if (msg.id == (uint16_t)ROBOT_ID) {
    leftServo.write(msg.leftAngle);
    rightServo.write(msg.rightAngle);

    currentLeft  = msg.leftAngle;
    currentRight = msg.rightAngle;

    Serial.printf("[ID %u] Applied  L:%u  R:%u\n",
                  (unsigned)msg.id, (unsigned)msg.leftAngle, (unsigned)msg.rightAngle);
  } else {
    Serial.printf("[ID %u] Ignored (this robot is ID %u)\n",
                  (unsigned)msg.id, (unsigned)ROBOT_ID);
  }
}

// -------------------- Display Refresh --------------------
void refreshLCD() {
  lcdClear();

  // Row 1: show "Me:<ID> Rx:<lastId>"
  String row1 = "Me:" + String(ROBOT_ID);
  if (lastMsgId >= 0) {
    row1 += " Rx:" + String(lastMsgId);
  }
  printToLCD(row1, 1, 0);

  // Row 2: angles or waiting
  if (currentLeft >= 0 && currentRight >= 0) {
    String row2 = "L:" + String(currentLeft) + " R:" + String(currentRight);
    printToLCD(row2, 2, 0);
  } else {
    printToLCD("Waiting...", 2, 0);
  }
}

void setup() {
  Serial.begin(115200);

  // LCD Setup
  lcdSerial.begin(19200, SERIAL_8N1, -1, 8); // TX = GPIO 5, RX = GPIO 8
  delay(100);
  lcdSerial.write(0x11); delay(5);  // Brightness (device-specific)
  lcdSerial.write(0x16); delay(5);  // Contrast   (device-specific)
  lcdClear();

  // Servo Setup
  leftServo.setPeriodHertz(50);
  rightServo.setPeriodHertz(50);
  leftServo.attach(leftPin, 500, 2500);
  rightServo.attach(rightPin, 500, 2500);

  // ESP-NOW Setup
  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  delay(100);

  printToLCD("ESP-NOW Ready", 1, 0);
  printToLCD(getMacAddress(), 2, 0);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    lcdClear();
    printToLCD("ESP-NOW FAIL", 1, 0);
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.printf("ESP-NOW Receiver Ready (ROBOT_ID=%u)\n", (unsigned)ROBOT_ID);

  delay(600);
  refreshLCD(); // initial screen
  lastDisplayTime = millis();
}

void loop() {
  // Periodic-only LCD updates
  unsigned long now = millis();
  if (now - lastDisplayTime >= displayInterval) {
    lastDisplayTime = now;
    refreshLCD();
  }
}
