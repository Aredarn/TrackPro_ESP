#include <TinyGPS++.h>
#include <ArduinoJson.h>

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <esp_netif.h>
#include <lwip/sockets.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// GPS and SoftwareSerial objects
TinyGPSPlus gps;
HardwareSerial mySerial(1);
BluetoothSerial SerialBT;
Preferences prefs;

// Variables to store GPS data
String lastTimestamp = "";
unsigned long lastUpdateTime = 0;  // For timing updates
uint8_t currentRateHz = 10;

// Promoted out of loop() so it can stay non-blocking and service both transports every iteration
WiFiClient wifiClient;
String wifiCmdBuffer;
String btCmdBuffer;
bool btWasConnected = false;

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

// UBX 8-bit Fletcher checksum, over Class+ID+Length+Payload only (not the sync bytes, not the checksum bytes)
void ubxChecksum(const uint8_t *buf, size_t len, uint8_t &ckA, uint8_t &ckB) {
  ckA = 0;
  ckB = 0;
  for (size_t i = 0; i < len; i++) {
    ckA += buf[i];
    ckB += ckA;
  }
}

// Fills outBuf (must be >= 14 bytes) with a UBX CFG-RATE message and returns its length
uint8_t buildCfgRateMsg(uint16_t measRateMs, uint8_t outBuf[14]) {
  outBuf[0] = 0xB5;
  outBuf[1] = 0x62;
  outBuf[2] = 0x06;  // Class: CFG
  outBuf[3] = 0x08;  // ID: RATE
  outBuf[4] = 0x06;  // Payload length LSB
  outBuf[5] = 0x00;  // Payload length MSB
  outBuf[6] = measRateMs & 0xFF;
  outBuf[7] = (measRateMs >> 8) & 0xFF;
  outBuf[8] = 0x01;  // navRate LSB
  outBuf[9] = 0x00;  // navRate MSB
  outBuf[10] = 0x01; // timeRef LSB
  outBuf[11] = 0x00; // timeRef MSB

  uint8_t ckA, ckB;
  ubxChecksum(&outBuf[2], 10, ckA, ckB);  // Class..Payload = 10 bytes
  outBuf[12] = ckA;
  outBuf[13] = ckB;
  return 14;
}

bool isSupportedRateHz(uint8_t hz) {
  return hz == 5 || hz == 10 || hz == 20 || hz == 25;
}

bool applyGpsRate(uint8_t hz) {
  if (!isSupportedRateHz(hz)) return false;
  uint16_t measRateMs = 1000 / hz;
  uint8_t msg[14];
  uint8_t len = buildCfgRateMsg(measRateMs, msg);
  sendUBX(msg, len);
  currentRateHz = hz;
  return true;
}

uint8_t loadPersistedRateHz() {
  prefs.begin("trackpro", true);
  uint8_t hz = prefs.getUChar("rate_hz", 10);
  prefs.end();
  return isSupportedRateHz(hz) ? hz : 10;
}

void persistRateHz(uint8_t hz) {
  prefs.begin("trackpro", false);
  prefs.putUChar("rate_hz", hz);
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

void setupBluetoothSPP() {
  SerialBT.begin("TrackPro_ESP32");
  Serial.println("Bluetooth SPP started as 'TrackPro_ESP32'");
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

// Templated (not Stream&/Print&) so WiFiClient and BluetoothSerial share one
// implementation without depending on which base class declares which method.

template <typename T>
void sendGpsUpdate(T &out, const String &currentGpsData) {
  out.print(currentGpsData);
  out.print("\n");
  out.flush();  // force immediate send, don't let it sit buffered
  Serial.println("GPS Update Sent: " + currentGpsData);
}

template <typename T>
void handleCommandLine(const String &line, T &out) {
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

template <typename T>
void pollCommands(T &transport, String &lineBuf) {
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
    char c = mySerial.read();
    Serial.write(c);  // TEMP DEBUG: raw NMEA passthrough to see what the module actually sends - remove once diagnosed
    gps.encode(c);
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.setRxBufferSize(1024);
  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(2000);

  currentRateHz = loadPersistedRateHz();
  applyGpsRate(currentRateHz);
  Serial.print("GPS update rate set to ");
  Serial.print(currentRateHz);
  Serial.println("Hz (persisted)");
  delay(1000);

  sendUBX(cfgPrt, sizeof(cfgPrt));
  delay(1000);  // Allow time for save and reboot

  mySerial.updateBaudRate(115200);
  delay(100);  // Allow time for save and reboot

  setupWiFiAP();
  setupBluetoothSPP();

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

  if (wifiClient.connected()) {
    pollCommands(wifiClient, wifiCmdBuffer);
  }

  if (SerialBT.hasClient()) {
    if (!btWasConnected) {
      btCmdBuffer = "";
      btWasConnected = true;
      Serial.println("Bluetooth client connected");
    }
    pollCommands(SerialBT, btCmdBuffer);
  } else if (btWasConnected) {
    btWasConnected = false;
    Serial.println("Bluetooth client disconnected");
  }

  if (gps.location.isUpdated()) {
    String currentTimestamp = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "." + String(gps.time.centisecond());

    if (currentTimestamp != lastTimestamp && millis() - lastUpdateTime >= 100) {
      lastUpdateTime = millis();
      lastTimestamp = currentTimestamp;
      String currentGpsData = createGpsJson();

      if (wifiClient.connected()) {
        sendGpsUpdate(wifiClient, currentGpsData);
      }
      if (SerialBT.hasClient()) {
        sendGpsUpdate(SerialBT, currentGpsData);
      }
    }
  }
}

String createGpsJson() {
  StaticJsonDocument<200> doc;

  // Report last-known values as-is instead of snapping to 0.0 on a momentary fix
  // loss - "valid" tells the app whether to trust this fix rather than the
  // firmware faking a jump to null island / a fake speed=0 spike.
  doc["latitude"] = gps.location.lat();
  doc["longitude"] = gps.location.lng();
  doc["altitude"] = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  doc["speed"] = gps.speed.kmph();
  doc["satellites"] = gps.satellites.isValid() ? gps.satellites.value() : 0;
  doc["valid"] = gps.location.isValid();
  doc["timestamp"] = createFormattedTimestamp().c_str();

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
