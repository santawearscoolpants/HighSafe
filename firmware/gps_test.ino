#include <TinyGPSPlus.h>

// For ESP32, use HardwareSerial instead of SoftwareSerial
HardwareSerial gpsSerial(1);  // Use UART1 for GPS

// Choose GPIO pins for GPS RX/TX
const int RXPin = 13;  // GPS TX --> ESP32 RX
const int TXPin = 12;  // GPS RX --> ESP32 TX

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);  // USB serial to PC
  gpsSerial.begin(9600, SERIAL_8N1, RXPin, TXPin);  // GPS serial
  Serial.println("GPS Google Maps Link Example");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();

    Serial.print("Latitude: ");
    Serial.println(lat, 6);
    Serial.print("Longitude: ");
    Serial.println(lon, 6);

    Serial.print("Google Maps Link: ");
    Serial.print("https://www.google.com/maps/search/?api=1&query=");
    Serial.print(lat, 6);
    Serial.print(",");
    Serial.println(lon, 6);

    Serial.println("------------------------------");
    delay(2000); // update every 2 sec
  }
}
