#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "SSD1306Wire.h"
#include <ArduinoJson.h>
#define GPSBaud 9600

// GPS and SoftwareSerial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 5);  // RX, TX
SSD1306Wire display(0x3c, 14, 12);

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
// Wi-Fi credentials for AP mode
const char *ssid = "ESP8266";
const char *password = "yourpasswordhere";

// TCP server on port 4210
WiFiServer tcpServer(4210);


// Variables to store GPS data
String lastTimestamp = "";         // Store the last timestamp sent
unsigned long lastUpdateTime = 0;  // For timing updates
bool gpsConnected = false;         // Flag for GPS connection
bool isConnected = false;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  delay(2000);  // Allow GPS module to initialize


  byte set5Hz[] = { 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00 };
  sendUBX(set5Hz, sizeof(set5Hz));

  byte saveConfig[] = { 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
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
  WiFi.setSleep(false);
  Serial.println("Access Point started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());  // Should display 192.168.4.1

  // Start TCP server
  tcpServer.setNoDelay(true);
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

  Display();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(20, 20, "Waiting for client");
  display.display();
  WiFiClient client = tcpServer.available();

  if (client) {
      client.setRxBufferSize(128);  // Reduce from default 1460
      client.setTxBufferSize(128);


    Serial.println("Client connected");
    isConnected = true;
    Display();

    while (client.connected()) {


      //ONLY FOR TESTING
      /*
      if(gpsSerial.available() < 1 && millis() - lastUpdateTime >= 100)
      {
        lastUpdateTime = millis();
        sendDummyGpsUpdate(client);
      }
      */
      gpsConnected = false;
      if (gpsSerial.available() > 0) {
        while (gpsSerial.available() > 0) {
          gps.encode(gpsSerial.read());
        }
        gpsConnected = true;  // Mark GPS as connected
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
    Display();
    Serial.println("Client disconnected");
    client.stop();
  }
}


void Display() {

  display.clear();
  display.setFont(ArialMT_Plain_10);

  // The coordinates define the left starting point of the text
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(20, 20, isConnected ? "Client connected" : "Waiting for client");
  display.drawString(20, 35, gpsConnected ? "GPS signal OK" : "Waiting for GPS");
  display.display();
}


void sendGpsUpdate(WiFiClient &client) {
  String currentGpsData = createGpsJson();
  client.print(currentGpsData);
  client.print("\n");  // Explicit newline
  client.flush();  // Force immediate send
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
