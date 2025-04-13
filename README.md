This project demonstrates how to use an ESP to act as a GPS data server. The device collects GPS data using a GPS module (Mateksys m10q / GY-GPS6MV2) and streams it to connected clients over a WebSocket server. The ESP operates in Access Point (AP) mode, allowing direct client connections without the need for an external Wi-Fi network.

Features:

    - GPS Data Collection: Captures latitude, longitude, altitude, speed, number of satellites, and timestamp using the TinyGPS++ library.
    - TCP Communication: Streams real-time GPS data to to a device with port 4210
    - AP Mode: ESP hosts its own Wi-Fi network for easy client connection.

Setup

    Hardware Connections:
        Connect the GPS module to the ESP8266/ESP32 using pins D2 (GPIO4) for RX and D1 (GPIO5) for TX.
        Ensure the GPS module's baud rate is set to 9600.

    Dependencies:
        Install the following Arduino libraries:
            TinyGPS++
            WebSockets

    Wi-Fi Configuration:
        Update the ssid and password variables in the code to set your desired Access Point credentials.

    Flash the ESP8266:
        Upload the code to the ESP8266 / ESP32 using the Arduino IDE.

Tested with:

       - ESP8266 OLED with GY-GPS6MV2 at 5hz refresh
       - ESP32 C3 super mini with MatekSYS M10Q 
       - ESP32 38 PIN with MatekSYS M10Q (Working at 115200 baud rate at 10hz) [goal:25hz]

Used libraries:

       - HardwareSerial
