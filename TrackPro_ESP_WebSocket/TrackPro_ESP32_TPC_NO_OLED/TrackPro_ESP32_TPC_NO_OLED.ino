#include <TinyGPS++.h>
#include <SoftwareSerial.h>
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
SoftwareSerial gpsSerial(4, 5);  // RX, TX

// Send UBX command
void sendUBX(byte *msg, uint8_t len) {
  byte ck[2];
  calcChecksum(msg, len, ck);
  gpsSerial.write(0xB5);  // Header
  gpsSerial.write(0x62);  // Header
  gpsSerial.write(msg, len);
  gpsSerial.write(ck[0]);
  gpsSerial.write(ck[1]);
}

void calcChecksum(byte *msg, uint8_t len, byte *ck) {
  ck[0] = ck[1] = 0;
  for (uint8_t i = 0; i < len; i++) {
    ck[0] += msg[i];
    ck[1] += ck[0];
  }
}

// Variables to store GPS data
String lastTimestamp = "";
unsigned long lastUpdateTime = 0;  // For timing updates
bool gpsConnected = false;         // Flag for GPS connection
bool isConnected = false;

// WiFi configuration constants
WiFiServer tcpServer(4210);
const char *AP_SSID = "TrackPro_AP";
const char *AP_PASSWORD = "trackpro123";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

void setupWiFiAP() {
  // First completely deinit WiFi
  WiFi.enableAP(false);
  WiFi.disconnect(true);
  delay(100);

  // Configure AP parameters using Arduino API
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);

  // Set custom WiFi parameters before starting
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.nvs_enable = 0;  // Disable NVS for this session
  esp_wifi_init(&cfg);

  // Set aggressive WiFi parameters
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Start AP with Arduino API
  WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 1);

  // Additional low-level configuration
  wifi_config_t wifi_config;
  esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
  wifi_config.ap.beacon_interval = 100;
  wifi_config.ap.max_connection = 1;
  esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

  // Add delay for AP stabilization
  delay(500);

  Serial.print("AP IP Address: ");
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
  gpsSerial.begin(GPSBaud);
  delay(2000);  // Allow GPS module to initialize


  byte set5Hz[] = { 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00 };
  sendUBX(set5Hz, sizeof(set5Hz));

  byte saveConfig[] = { 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendUBX(saveConfig, sizeof(saveConfig));

  // Initialize GPS...
  setupWiFiAP();

  // Start TCP server with optimized settings
  tcpServer.setNoDelay(true);
  tcpServer.begin();
  Serial.println("TCP server started.");
}

//ONLY FOR TESTING
/*
      if(gpsSerial.available() < 1 && millis() - lastUpdateTime >= 100)
      {
        lastUpdateTime = millis();
        sendDummyGpsUpdate(client);
      }
      */

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

      gpsConnected = false;
      if(gpsConnected == false && millis() - lastUpdateTime >= 100)
      {
        lastUpdateTime = millis();
        sendDummyGpsUpdate(client);
      }
      
      if (gpsSerial.available() > 0) {
        while (gpsSerial.available() > 0) {
          gps.encode(gpsSerial.read());
        }
        gpsConnected = true;
      }

      if (gps.location.isUpdated()) {
        String currentTimestamp = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "." + String(gps.time.centisecond());

        if (currentTimestamp != lastTimestamp && millis() - lastUpdateTime >= 100) {
          lastUpdateTime = millis();
          lastTimestamp = currentTimestamp;
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
