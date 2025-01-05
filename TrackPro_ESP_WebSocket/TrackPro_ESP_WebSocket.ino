#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <WebSocketsServer.h>

#define GPSBaud 9600

// GPS and SoftwareSerial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 5); // RX, TX

// Wi-Fi credentials for AP mode
const char *ssid = "ESP8266";
const char *password = "yourpasswordhere";

// WebSocket server on port 81
WebSocketsServer webSocket(81);

// Variables to store GPS data
String lastGpsData = "";
unsigned long lastUpdateTime = 0; // For timing updates

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);

  // Configure ESP8266 as an Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");
}

void loop() {
  // Process GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      sendGpsUpdate();
    }
  }

  // Handle WebSocket events
  webSocket.loop();
}

void sendGpsUpdate() {
  String currentGpsData = createGpsJson();

  // Avoid sending the same data repeatedly
  if (currentGpsData != lastGpsData) {
    lastGpsData = currentGpsData;
    webSocket.broadcastTXT(currentGpsData); // Send to all connected clients
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
  json += "\"timestamp\": \"" + String(gps.time.isValid() ? gps.time.hour() : 0) + ":" +
          String(gps.time.minute()) + ":" + String(gps.time.second()) + "\"";
  json += "}";
  return json;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("Client #%u disconnected\n", num);
      break;
    case WStype_CONNECTED:
      Serial.printf("Client #%u connected\n", num);
      webSocket.sendTXT(num, lastGpsData); // Send the last known GPS data on connection
      break;
    case WStype_TEXT:
      Serial.printf("Received from client #%u: %s\n", num, payload);
      break;
    default:
      break;
  }
}
