#include <TinyGPS++.h>
#include <ArduinoJson.h>

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <esp_netif.h>
#include <lwip/sockets.h>
#include <Preferences.h>

// GPS objects
TinyGPSPlus gps;
HardwareSerial mySerial(1);
Preferences prefs;

uint8_t currentRateHz = 10;
uint32_t lastFixEpoch = UINT32_MAX;  // gps.time.value() (hhmmsscc) of the last fix sent
unsigned long lastLineSentMs = 0;

// With no fresh fix for this long, send a NOFIX line so the app can tell a live link from
// a dead one - its stall watchdog reconnects after a few seconds of silence.
const unsigned long NO_FIX_HEARTBEAT_MS = 1500;

// Promoted out of loop() so it can stay non-blocking
WiFiClient wifiClient;
String wifiCmdBuffer;

uint8_t cfgPrt[] = {
  0xB5, 0x62,                          // Header
  0x06, 0x00,                          // Class and ID for CFG-PRT
  0x14, 0x00,                          // Length of the message (20 bytes)
  0x01, 0x00,                          // Port ID (UART1 or another port)
  0x00, 0x00,                          // Reserved
  0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,  // Baud rate settings (115200)
  0x01, 0x00, 0x03, 0x00,              // Reserved
  0x03, 0x00, 0x00, 0x00,              // Configuration settings
  0x00, 0x00, 0xBC, 0x5E               // Checksum and additional settings
};

void sendUBX(const uint8_t *msg, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    mySerial.write(msg[i]);
  }
}

// Frames and sends a UBX message; the 8-bit Fletcher checksum covers class, id, length and payload
void sendUbxMessage(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  const uint8_t header[4] = { cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t ckA = 0, ckB = 0;
  for (uint8_t b : header) { ckA += b; ckB += ckA; }
  for (uint16_t i = 0; i < len; i++) { ckA += payload[i]; ckB += ckA; }

  mySerial.write(0xB5);
  mySerial.write(0x62);
  mySerial.write(header, sizeof(header));
  mySerial.write(payload, len);
  mySerial.write(ckA);
  mySerial.write(ckB);
}

bool isSupportedRateHz(uint8_t hz) {
  return hz == 5 || hz == 10 || hz == 20 || hz == 25;
}

bool applyGpsRate(uint8_t hz) {
  if (!isSupportedRateHz(hz)) return false;
  uint16_t measRateMs = 1000 / hz;
  // UBX-CFG-RATE: measRate (ms), navRate = 1, timeRef = 1
  const uint8_t payload[6] = { (uint8_t)(measRateMs & 0xFF), (uint8_t)(measRateMs >> 8), 0x01, 0x00, 0x01, 0x00 };
  sendUbxMessage(0x06, 0x08, payload, sizeof(payload));
  currentRateHz = hz;
  return true;
}

// Turns off the NMEA sentences TinyGPS++ doesn't use (GLL, GSA, GSV, VTG), leaving GGA + RMC.
// With them on, 20-25 Hz output is more than 115200 baud can carry and sentences get dropped.
void disableUnusedNmea() {
  const uint8_t payload[] = {
    0x00, 0x01, 0x00, 0x00,        // UBX-CFG-VALSET: version 0, RAM layer, reserved
    0xCA, 0x00, 0x91, 0x20, 0x00,  // CFG-MSGOUT-NMEA_ID_GLL_UART1 = 0
    0xC0, 0x00, 0x91, 0x20, 0x00,  // CFG-MSGOUT-NMEA_ID_GSA_UART1 = 0
    0xC5, 0x00, 0x91, 0x20, 0x00,  // CFG-MSGOUT-NMEA_ID_GSV_UART1 = 0
    0xB1, 0x00, 0x91, 0x20, 0x00   // CFG-MSGOUT-NMEA_ID_VTG_UART1 = 0
  };
  sendUbxMessage(0x06, 0x8A, payload, sizeof(payload));
}

uint8_t loadPersistedRateHz() {
  prefs.begin("trackpro", true);
  uint8_t hz = prefs.getUChar("rate_hz", 10);
  prefs.end();
  return isSupportedRateHz(hz) ? hz : 10;
}

void persistRateHz(uint8_t hz) {
  prefs.begin("trackpro", false);
  // The app re-sends RATE on every (re)connect; only write flash when the value changes
  if (prefs.getUChar("rate_hz", 0) != hz) prefs.putUChar("rate_hz", hz);
  prefs.end();
}

// WiFi configuration constants
WiFiServer tcpServer(4210);
const char *AP_SSID = "TrackPro_AP";
const char *AP_PASSWORD = "trackpro123";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

void setupWiFiAP() {
  WiFi.enableAP(false);
  WiFi.disconnect(true);
  delay(100);

  // Set AP configuration
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);

  // Start AP with SSID, password, channel, visibility, max connections
  WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 1); // Channel 6, visible, max 1 connection

  delay(500); // Allow time for AP to initialize

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void configureClientSocket(WiFiClient &client) {
  int sock = client.fd();
  int enable = 1;
  int timeout_ms = 100;

  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));
  setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));
}

// No flush(): on the ESP32, WiFiClient::flush() clears unread *incoming* data, which could
// swallow a RATE command. TCP_NODELAY already sends each line immediately.
void sendLine(const String &line) {
  wifiClient.print(line);
  wifiClient.print("\n");
  lastLineSentMs = millis();
}

void handleCommandLine(const String &line, WiFiClient &out) {
  String trimmed = line;
  trimmed.trim();

  if (trimmed.startsWith("RATE:")) {
    int hz = trimmed.substring(5).toInt();
    if (applyGpsRate((uint8_t)hz)) {
      persistRateHz((uint8_t)hz);
      out.print("RATE_OK:");
      out.print(hz);
      out.print("\n");
      return;
    }
  }

  out.print("RATE_ERR\n");
}

void pollCommands(WiFiClient &transport, String &lineBuf) {
  while (transport.available() > 0) {
    char c = (char)transport.read();
    if (c == '\n') {
      handleCommandLine(lineBuf, transport);
      lineBuf = "";
    } else if (c != '\r') {
      lineBuf += c;
      if (lineBuf.length() > 32) lineBuf = "";  // guard: peer never sent '\n'
    }
  }
}

void acceptWifiClientIfNeeded() {
  if (wifiClient.connected()) return;

  wifiClient.stop();
  WiFiClient newClient = tcpServer.available();
  if (newClient) {
    wifiClient = newClient;
    configureClientSocket(wifiClient);
    wifiCmdBuffer = "";
    Serial.println("WiFi client connected");
  }
}

// Runs every loop() regardless of client state, so the UART RX buffer can't overflow while nothing is connected
void pumpGpsSerial() {
  while (mySerial.available() > 0) {
    gps.encode(mySerial.read());
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.setRxBufferSize(1024);
  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(2000);

  // Switch the module to 115200. If it kept 115200 from before (backup power) it can't read
  // this at 9600 and ignores it - either way both sides end up at 115200.
  sendUBX(cfgPrt, sizeof(cfgPrt));
  delay(1000);  // Let the module switch baud rate
  mySerial.updateBaudRate(115200);
  delay(100);

  // Configured at 115200 so it lands whichever baud rate the module started at
  disableUnusedNmea();
  currentRateHz = loadPersistedRateHz();
  applyGpsRate(currentRateHz);
  Serial.print("GPS update rate set to ");
  Serial.print(currentRateHz);
  Serial.println("Hz (persisted)");

  setupWiFiAP();

  // Start TCP server with optimized settings
  tcpServer.setNoDelay(true);
  tcpServer.begin();
  Serial.println("TCP server started.");
  Serial.println("Dumping GPS responses:");
  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}

void loop() {
  acceptWifiClientIfNeeded();
  pumpGpsSerial();

  if (!wifiClient.connected()) return;
  pollCommands(wifiClient, wifiCmdBuffer);

  // RMC and GGA each report the same fix, so dedupe on the GPS epoch time. No time-based
  // throttle, so output follows whatever rate the module runs at.
  if (gps.location.isUpdated() && gps.time.value() != lastFixEpoch) {
    lastFixEpoch = gps.time.value();
    sendLine(createGpsJson());
  } else if (millis() - lastLineSentMs >= NO_FIX_HEARTBEAT_MS) {
    sendLine("NOFIX");
  }
}

// Only called for a fresh fix, so "valid" is always true here. No-fix periods are sent as
// NOFIX lines instead of JSON the app would record as a real sample.
String createGpsJson() {
  StaticJsonDocument<200> doc;

  doc["latitude"] = gps.location.lat();
  doc["longitude"] = gps.location.lng();
  doc["altitude"] = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  doc["speed"] = gps.speed.kmph();
  doc["satellites"] = gps.satellites.isValid() ? gps.satellites.value() : 0;
  doc["valid"] = gps.location.isValid();
  doc["timestamp"] = createFormattedTimestamp();  // assigned as a String so it's copied, not held by pointer

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// HH:mm:ss.cc, all fields from the GPS's own clock (not millis()) so timing stays accurate
String createFormattedTimestamp() {
  int hour = gps.time.isValid() ? gps.time.hour() : 0;
  int minute = gps.time.isValid() ? gps.time.minute() : 0;
  int second = gps.time.isValid() ? gps.time.second() : 0;
  int centisecond = gps.time.isValid() ? gps.time.centisecond() : 0;

  return String(hour) + ":" + String(minute) + ":" + String(second) + "." + String(centisecond);
}
