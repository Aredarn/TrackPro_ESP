#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h> 
#include "SSD1306Wire.h"   
#define GPSBaud 9600

// GPS and SoftwareSerial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 5);  // RX, TX
SSD1306Wire display(0x3c, 14, 12);

// Send UBX command
void sendUBX(byte *msg, uint8_t len) {
  byte ck[2];
  calcChecksum(msg, len, ck);
  gpsSerial.write(0xB5); // Header
  gpsSerial.write(0x62); // Header
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
// Wi-Fi credentials for AP mode
const char *ssid = "ESP8266";
const char *password = "yourpasswordhere";

// TCP server on port 4210
WiFiServer tcpServer(4210);

// Variables to store GPS data
String lastTimestamp = "";  // Store the last timestamp sent
unsigned long lastUpdateTime = 0;  // For timing updates
bool gpsConnected = false;         // Flag for GPS connection

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  delay(2000); // Allow GPS module to initialize


  byte set5Hz[] = {0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
  sendUBX(set5Hz, sizeof(set5Hz));

  byte saveConfig[] = {0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendUBX(saveConfig, sizeof(saveConfig));

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

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

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(20, 20, "TCP server started");
  Serial.println("TCP server started.");
  delay(2000);
}

void loop() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  // The coordinates define the left starting point of the text
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(20, 20, "Waiting for client"); 
  display.display();
  WiFiClient client = tcpServer.available();
  if (client) {

  display.clear();
  display.setFont(ArialMT_Plain_10);

  // The coordinates define the left starting point of the text
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(20, 20, "Client connected"); 
  display.display();

  Serial.println("Client connected");


  while (client.connected()) {
  gpsConnected = false;
    
  while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
      gpsConnected = true; // Mark GPS as connected
  }

    if (gps.location.isUpdated()) {
        String currentTimestamp = String(gps.time.hour()) + ":" + 
                                  String(gps.time.minute()) + ":" + 
                                  String(gps.time.second()) + "." + 
                                  String(gps.time.centisecond());

        if (currentTimestamp != lastTimestamp && millis() - lastUpdateTime >= 40) {
            lastUpdateTime = millis();
            lastTimestamp = currentTimestamp;
            sendGpsUpdate(client);
        }
    }
}

    // Client disconnected
    Serial.println("Client disconnected");
    client.stop();
  }
}

void sendGpsUpdate(WiFiClient &client) {
  String currentGpsData = createGpsJson();
    client.println(currentGpsData);
    Serial.println("GPS Update Sent: " + currentGpsData);
  }

String createGpsJson() {
    char jsonBuffer[128]; // Allocate buffer for JSON string

    snprintf(jsonBuffer, sizeof(jsonBuffer),
        "{\"latitude\": %.6f, \"longitude\": %.6f, \"altitude\": %.2f, \"speed\": %.2f, \"satellites\": %d, \"timestamp\": \"%02d:%02d:%02d\"}",
        gps.location.isValid() ? gps.location.lat() : 0.0,
        gps.location.isValid() ? gps.location.lng() : 0.0,
        gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
        gps.speed.isValid() ? gps.speed.kmph() : 0.0,
        gps.satellites.isValid() ? gps.satellites.value() : 0,
        gps.time.isValid() ? gps.time.hour() : 0,
        gps.time.isValid() ? gps.time.minute() : 0,
        gps.time.isValid() ? gps.time.second() : 0
    );

    return String(jsonBuffer);
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

/*
  if (dummyData != lastGpsData) {
    lastGpsData = dummyData;
    client.println(dummyData);  // Send to the connected client
    Serial.println("Dummy Data Sent: " + dummyData);
  }*/
}
