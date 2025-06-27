/*
It monitors for physical tilt (fall/impact), 
listens for emergency button presses, gets your current location, 
and sends it as a Google Maps link via SMS to preset 
contacts for accident or robbery alerts.
*/

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>

// MPU6050
Adafruit_MPU6050 mpu;

// GPS
#define GPS_RX 16
#define GPS_TX 17
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// SIM800L
#define SIM_TX 27
#define SIM_RX 26
HardwareSerial sim800(1);

// Buttons
#define ACCIDENT_BTN 32
#define ROBBERY_BTN 33

// Contacts
const char* emergencyContact = "+233249097323"; 
const char* familyContact = "+233594849077";   
const char* policeContact = "+233244077854"; 

// Tilt threshold (degrees)
const float TILT_THRESHOLD = 45.0;
unsigned long lastTiltTime = 0;
const int tiltDuration = 2000; // milliseconds

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  sim800.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);

  pinMode(ACCIDENT_BTN, INPUT_PULLDOWN);
  pinMode(ROBBERY_BTN, INPUT_PULLDOWN);

  // MPU6050 setup
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(1000);
  Serial.println("System Ready");

  sendAT("AT");
  sendAT("AT+CMGF=1");
  sendAT("AT+CSCS=\"GSM\"");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Handle Buttons
  if (digitalRead(ACCIDENT_BTN) == HIGH) {
    Serial.println("Manual Accident Button Pressed");
    sendEmergencySMS(emergencyContact, "🚨 Manual Accident Alert!");
    delay(2000);
  }

  if (digitalRead(ROBBERY_BTN) == HIGH) {
    Serial.println("Manual Robbery Button Pressed");
    sendEmergencySMS(familyContact, "🚨 Robbery Alert! [Family]");
    sendEmergencySMS(policeContact, "🚓 Robbery Alert! [Police]");
    delay(2000);
  }

  // Auto tilt detection
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float roll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  if (abs(pitch) > TILT_THRESHOLD || abs(roll) > TILT_THRESHOLD) {
    if (millis() - lastTiltTime > tiltDuration) {
      Serial.println("⚠️ Tilt threshold exceeded! Sending auto accident alert...");
      sendEmergencySMS(emergencyContact, "🚨 Auto Accident Detected (Tilt > 45°)");
      lastTiltTime = millis();
    }
  }

  delay(500);
}

void sendEmergencySMS(const char* number, String message) {
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();

    String fullMessage = message + "\nLocation:\nhttps://www.google.com/maps/search/?api=1&query=" +
                         String(lat, 6) + "," + String(lng, 6);

    sim800.println("AT+CMGS=\"" + String(number) + "\"");
    delay(1000);
    sim800.print(fullMessage);
    delay(500);
    sim800.write(26); // Ctrl+Z to send
    Serial.println("📤 SMS sent to " + String(number));
  } else {
    Serial.println("⚠️ GPS not ready. Cannot send SMS.");
  }
}

void sendAT(const char* cmd) {
  sim800.println(cmd);
  delay(1000);
  while (sim800.available()) {
    Serial.write(sim800.read());
  }
}
