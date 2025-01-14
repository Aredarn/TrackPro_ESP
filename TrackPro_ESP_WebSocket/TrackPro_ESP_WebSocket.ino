#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define GPSBaud 9600

// GPS and SoftwareSerial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 5);  // RX, TX

// Wi-Fi credentials for AP mode
const char *ssid = "ESP8266";
const char *password = "yourpasswordhere";

// TCP server on port 4210
WiFiServer tcpServer(4210);

// Variables to store GPS data
String lastGpsData = "";
unsigned long lastUpdateTime = 0;  // For timing updates
bool gpsConnected = false;         // Flag for GPS connection

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);

  // Configure ESP8266 as an Access Point
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure AP IP address!");
    return;
  }

  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());  // Should display 192.168.4.1

  // Start TCP server
  tcpServer.begin();
  Serial.println("TCP server started on 192.168.4.1:4210");
}

void loop() {
  // Check if any client is connected
  WiFiClient client = tcpServer.available();
  if (client) {
    Serial.println("Client connected");
    while (client.connected()) {
      // Process GPS data
      gpsConnected = false;
      while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
        gpsConnected = true;
        if (gps.location.isUpdated()) {
          sendGpsUpdate(client);
        }
      }

      // Send dummy data if GPS is not connected
      if (!gpsConnected) {
        sendDummyData(client);
      }
    }
    // Client disconnected
    Serial.println("Client disconnected");
    client.stop();
  }
}

void sendGpsUpdate(WiFiClient &client) {
  String currentGpsData = createGpsJson();

  // Avoid sending the same data repeatedly
  if (currentGpsData != lastGpsData) {
    lastGpsData = currentGpsData;
    client.println(currentGpsData);  // Send to the connected client
    Serial.println("GPS Update Sent: " + currentGpsData);
  }
}

String createGpsJson() {
  // Create JSON-formatted GPS data
  String json = "{";
  json += "\"latitude\": " + String(gps.location.isValid() ? gps.location.lat() : 0, 6) + ",";
  json += "\"longitude\": " + String(gps.location.isValid() ? gps.location.lng() : 0, 6) + ",";
  json += "\"altitude\": " + String(gps.altitude.isValid() ? gps.altitude.meters() : 0) + ",";
  json += "\"speed\": " + String(gps.speed.isValid() ? gps.speed.kmph() : 0) + ",";
  json += "\"satellites\": " + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";
  json += "\"timestamp\": \"" + String(gps.time.isValid() ? gps.time.hour() : 0) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\"";
  json += "}";
  return json;
}

void sendDummyData(WiFiClient &client) {
  String dummyData = "{";
  dummyData += "\"latitude\": " + String(random(-900000, 900000) / 10000.0, 6) + ",";
  dummyData += "\"longitude\": " + String(random(-1800000, 1800000) / 10000.0, 6) + ",";
  dummyData += "\"altitude\": " + String(random(0, 5000)) + ",";
  dummyData += "\"speed\": " + String(random(0, 120)) + ",";
  dummyData += "\"satellites\": " + String(random(1, 12)) + ",";
  dummyData += "\"timestamp\": \"" + String(random(0, 24)) + ":" + String(random(0, 60)) + ":" + String(random(0, 60)) + "\"";
  dummyData += "}";
  delay(500);

  if (dummyData != lastGpsData) {
    lastGpsData = dummyData;
    client.println(dummyData);  // Send to the connected client
    Serial.println("Dummy Data Sent: " + dummyData);
  }
}
