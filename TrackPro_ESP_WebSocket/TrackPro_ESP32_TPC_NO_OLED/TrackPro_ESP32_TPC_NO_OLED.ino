#include <TinyGPS++.h>
#include <Wire.h>
#include <ArduinoJson.h>

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <esp_netif.h>
#include <lwip/sockets.h>

#define GPSBaud 9600

// GPS and SoftwareSerial objects
TinyGPSPlus gps;
HardwareSerial mySerial(1);


// Variables to store GPS data
String lastTimestamp = "";
unsigned long lastUpdateTime = 0;  // For timing updates
bool gpsConnected = false;         // Flag for GPS connection
bool isConnected = false;

const uint8_t setRate10Hz[] = {
  0xB5, 0x62,  // UBX header
  0x06, 0x08,  // CFG-RATE
  0x06, 0x00,  // Payload length: 6 bytes
  0x64, 0x00,  // Measurement rate: 100ms (0x0064)
  0x01, 0x00,  // Navigation rate: 1
  0x01, 0x00,  // Time reference: UTC
  0x7A, 0x12   // Checksum
};

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

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(2000);

  sendUBX(setRate10Hz, sizeof(setRate10Hz));
  Serial.println("GPS update rate set to 10Hz");
  delay(1000); 

  sendUBX(cfgPrt, sizeof(cfgPrt));
  delay(1000);  // Allow time for save and reboot

  mySerial.updateBaudRate(115200);
  delay(100);  // Allow time for save and reboot

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
  WiFiClient client = tcpServer.available();

  if (client) {
    int sock = client.fd();
    int enable = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
    Serial.println("Client connected");
    isConnected = true;

    while (client.connected()) {
      // Handle GPS data

      if (mySerial.available() > 0) {
        while (mySerial.available() > 0) {
          gps.encode(mySerial.read());
        }
        gpsConnected = true;
      }

      if (gps.location.isUpdated()) {
        String currentTimestamp = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "." + String(gps.time.centisecond());

        if (currentTimestamp != lastTimestamp && millis() - lastUpdateTime >= 100) {
          lastUpdateTime = millis();
          lastTimestamp = currentTimestamp;
          String currentGpsData = createGpsJson();

          Serial.print(currentGpsData);
          sendGpsUpdate(client);
        }
      }
    }

    // Client disconnected
    isConnected = false;
    Serial.println("Client disconnected");
    client.stop();
  }
}


void sendGpsUpdate(WiFiClient &client) {
  String currentGpsData = createGpsJson();
  client.print(currentGpsData);
  client.print("\n");  // Explicit newline
  client.flush();      // Force immediate send
  Serial.println("GPS Update Sent: " + currentGpsData);
}

void sendDummyGpsUpdate(WiFiClient &client) {
  String currentGpsData = createDummyGpsJson();
  client.print(currentGpsData);
  client.print("\n");
  client.flush();
  Serial.println("GPS Update Sent: " + currentGpsData);
}

//For real data
String createGpsJson() {
  StaticJsonDocument<200> doc;  // Specify a size for the JSON document

  doc["latitude"] = gps.location.isValid() ? gps.location.lat() : 0.0;
  doc["longitude"] = gps.location.isValid() ? gps.location.lng() : 0.0;
  doc["altitude"] = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  doc["speed"] = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  doc["satellites"] = gps.satellites.isValid() ? gps.satellites.value() : 0;
  doc["timestamp"] = createFormattedTimestamp().c_str();

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

String createDummyGpsJson() {
  StaticJsonDocument<200> doc;  // Allocate memory for the JSON document

  doc["latitude"] = random(-900000, 900000) / 10000.0;
  doc["longitude"] = random(-1800000, 1800000) / 10000.0;
  doc["altitude"] = random(0, 5000);
  doc["speed"] = random(0, 230);
  doc["satellites"] = random(7, 12);

  char timestamp[16];
  snprintf(timestamp, sizeof(timestamp), "%02d:%02d:%02d.000",
           random(0, 24), random(0, 60), random(0, 60));
  doc["timestamp"] = timestamp;

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}




String createFormattedTimestamp() {
  // Get GPS time in HH:mm:ss.SSS format
  unsigned long currentMillis = millis();
  int millisPart = currentMillis % 1000;  // Milliseconds within the current second
  int hour = gps.time.isValid() ? gps.time.hour() : 0;
  int minute = gps.time.isValid() ? gps.time.minute() : 0;
  int second = gps.time.isValid() ? gps.time.second() : 0;

  return String(hour) + ":" + String(minute) + ":" + String(second) + "." + String(millisPart);
}
