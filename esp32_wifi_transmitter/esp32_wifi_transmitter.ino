#include <WiFi.h>

// ===== Hotspot (AP) config =====
static const char* AP_SSID     = "ESP32S3_HOTSPOT";
static const char* AP_PASSWORD = "esp32pass123";   // >= 8 chars
static const int   AP_CHANNEL  = 6;

// ===== TCP server config =====
static const uint16_t SERVER_PORT = 9000;
static const int      MAX_CLIENTS = 10;

WiFiServer server(SERVER_PORT);
WiFiClient clients[MAX_CLIENTS];  // connected Pis

// Optional: static AP IP (default is 192.168.4.1)
IPAddress apIP(192,168,4,1);
IPAddress apGW(192,168,4,1);
IPAddress apMASK(255,255,255,0);

// Heartbeat every 5 seconds
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 2000; // ms

// Serial → Wi-Fi buffering
String lineBuf;                                 // accumulates input
unsigned long lastSerialRx = 0;
const unsigned long SERIAL_IDLE_FLUSH_MS = 20;  // flush partial line after 20ms idle
const size_t MAX_LINE_LEN = 4096;               // safety cap

#ifndef LED_BUILTIN
#define LED_BUILTIN 48     // adjust if your board uses a different LED pin, or comment out
#endif

// ---------- helpers ----------
void logClientsCount() {
  int n = 0;
  for (int i = 0; i < MAX_CLIENTS; i++) if (clients[i] && clients[i].connected()) n++;
  Serial.printf("[INFO] Connected clients: %d\n", n);
}

void addClient(WiFiClient& c) {
  if (!c) return;
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!clients[i] || !clients[i].connected()) {
      clients[i].stop();             // ensure clean slot
      clients[i] = c;                // move handle
      clients[i].setNoDelay(true);   // lower latency
      // If your core supports keepalive, you can enable it:
      // clients[i].setKeepAlive(true, 60, 10, 3);
      Serial.printf("[INFO] Client added in slot %d, remote=%s:%u\n",
                    i,
                    clients[i].remoteIP().toString().c_str(),
                    clients[i].remotePort());
      logClientsCount();
      return;
    }
  }
  Serial.println("[WARN] Too many clients; rejecting new one.");
  c.stop();
}

void pruneClients() {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && !clients[i].connected()) {
      Serial.printf("[INFO] Client in slot %d disconnected.\n", i);
      clients[i].stop();
    }
  }
}

// Write one full line (msg + '\n') to a client; return bytes written total (or -1 on error).
int writeLineTo(WiFiClient& cl, const String& msg) {
  int n1 = cl.print(msg);
  int n2 = cl.print('\n');
  if (n1 < 0 || n2 < 0) return -1;
  return n1 + n2;
}

void broadcastLine(const String& msg) {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) {
      int n = writeLineTo(clients[i], msg);
      if (n < 0) {
        Serial.printf("[WARN] Write failed; dropping client slot %d\n", i);
        clients[i].stop();
      }
    }
  }
}

// ---------- Arduino ----------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(300); // small settle

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGW, apMASK);
  WiFi.setSleep(false); // improve AP stability
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, MAX_CLIENTS);
  if (!ok) {
    Serial.println("[FATAL] softAP start failed");
    for (;;) {
      int state = digitalRead(LED_BUILTIN);
      digitalWrite(LED_BUILTIN, !state); // blink fast on fatal
      delay(200);
    }
  }

  Serial.println();
  Serial.println("===== ESP32S3 Serial→WiFi Broadcaster =====");
  Serial.printf("AP SSID: %s  PASS: %s\n", AP_SSID, AP_PASSWORD);
  Serial.printf("AP IP:   %s  Port: %u\n", WiFi.softAPIP().toString().c_str(), SERVER_PORT);

  server.begin(SERVER_PORT);
  server.setNoDelay(true);

  // fire first heartbeat soon after boot
  lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
}

void loop() {
  // Accept new clients
  if (server.hasClient()) {
    WiFiClient c = server.available();
    if (c) addClient(c);
  }

  // --- Serial → Wi-Fi fan-out, low latency ---
  // Read all available serial bytes
  while (Serial.available()) {
    char ch = (char)Serial.read();
    lastSerialRx = millis();

    if (ch == '\r') continue;  // normalize CRLF

    if (ch == '\n') {
      // Flush complete line immediately
      if (!lineBuf.isEmpty()) {
        Serial.printf("[TX] %s\n", lineBuf.c_str());
        broadcastLine(lineBuf);
        lineBuf = "";
      } else {
        // Empty line -> forward empty line if you care; here we ignore
      }
    } else {
      // Accumulate
      lineBuf += ch;
      if (lineBuf.length() > MAX_LINE_LEN) {
        Serial.println("[WARN] Line too long; clearing buffer");
        lineBuf = "";
      }
    }
  }

  // If no newline was sent, flush a partial line after a short idle
  if (!lineBuf.isEmpty() && (millis() - lastSerialRx >= SERIAL_IDLE_FLUSH_MS)) {
    Serial.printf("[TX/idle-flush] %s\n", lineBuf.c_str());
    broadcastLine(lineBuf);
    lineBuf = "";
  }

  // --- Heartbeat every 5s ---
  unsigned long now = millis();
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // blink LED
    String hb = String("{\"type\":\"heartbeat\",\"t\":") + String(now) + "}";
    Serial.printf("[HB] %s\n", hb.c_str());
    broadcastLine(hb);
  }

  pruneClients();
  delay(1); // yield
}
