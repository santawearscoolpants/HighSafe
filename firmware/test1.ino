/*

Test 1

The code reads live GPS coordinates using a NEO-7M module and 
sends emergency SMS alerts with a Google Maps location link 
via the SIM800L module when either the accident or 
robbery button is pressed.

*/


#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// GPS pins
#define GPS_RX 16
#define GPS_TX 17

// SIM800L pins
#define SIM_TX 27
#define SIM_RX 26

// Button pins
#define ACCIDENT_BTN 32
#define ROBBERY_BTN 33

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);     // UART2 for GPS
HardwareSerial sim800(1);        // UART1 for SIM800L

// Contacts
const char* emergencyContact = "+233249097323"; 
const char* familyContact = "+233594849077";   
const char* policeContact = "+233244077854";   // Replace with nearest police station number

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  sim800.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);

  pinMode(ACCIDENT_BTN, INPUT_PULLDOWN);
  pinMode(ROBBERY_BTN, INPUT_PULLDOWN);

  delay(1000);
  Serial.println("System initialized.");
  sendAT("AT");
  sendAT("AT+CMGF=1"); // Text mode
  sendAT("AT+CSCS=\"GSM\""); // Character set
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (digitalRead(ACCIDENT_BTN) == HIGH) {
    Serial.println("Accident button pressed!");
    sendEmergencySMS(emergencyContact, "🚨 Accident Alert!");
    delay(2000);
  }

  if (digitalRead(ROBBERY_BTN) == HIGH) {
    Serial.println("Robbery button pressed!");
    sendEmergencySMS(familyContact, "🚨 Robbery Alert! [Sent to Family]");
    sendEmergencySMS(policeContact, "🚓 Robbery Alert! [Sent to Police]");
    delay(2000);
  }
}

void sendEmergencySMS(const char* number, String message) {
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();

    String fullMessage = message + "\nLocation: " + 
                         "https://www.google.com/maps/search/?api=1&query=" +
                         String(lat, 6) + "," + String(lng, 6);

    sim800.println("AT+CMGS=\"" + String(number) + "\"");
    delay(1000);
    sim800.print(fullMessage);
    delay(500);
    sim800.write(26); // CTRL+Z to send
    Serial.println("SMS sent to " + String(number));
  } else {
    Serial.println("GPS location not valid.");
  }
}

void sendAT(const char* cmd) {
  sim800.println(cmd);
  delay(1000);
  while (sim800.available()) {
    Serial.write(sim800.read());
  }
}
