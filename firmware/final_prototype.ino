#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>

// Modules
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // GPS
HardwareSerial sim800(1);    // SIM800L

// Pin Configurations
#define GPS_RX 16
#define GPS_TX 17
#define SIM_TX 27
#define SIM_RX 26

#define ACCIDENT_BTN 32
#define ROBBERY_BTN 33
#define CANCEL_BTN 34
#define BUZZER_PIN 25

// Threshold
const float TILT_THRESHOLD = 45.0;     // degrees
const unsigned long BUZZER_DURATION = 10000; // 10s grace
unsigned long buzzerStartTime = 0;
bool buzzerActive = false;
bool alertPending = false;

// Contacts
const char* emergencyContact = "+233249097323"; 
const char* familyContact = "+233594849077";   
const char* policeContact = "+233244077854"; 

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  sim800.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);

  pinMode(ACCIDENT_BTN, INPUT_PULLDOWN);
  pinMode(ROBBERY_BTN, INPUT_PULLDOWN);
  pinMode(CANCEL_BTN, INPUT_PULLDOWN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  sendAT("AT");
  sendAT("AT+CMGF=1");
  sendAT("AT+CSCS=\"GSM\"");

  Serial.println("System Ready ✅");
}

void loop() {
  // Read GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Manual Alerts
  if (digitalRead(ACCIDENT_BTN) == HIGH) {
    Serial.println("Manual Accident Button Pressed");
    sendSMS(emergencyContact, "🚨 Manual Accident Alert!");
    delay(1000);
  }

  if (digitalRead(ROBBERY_BTN) == HIGH) {
    Serial.println("Manual Robbery Button Pressed");
    sendSMS(familyContact, "🚨 Robbery Alert! [Family]");
    sendSMS(policeContact, "🚓 Robbery Alert! [Police]");
    delay(1000);
  }

  // Cancel Alert
  if (buzzerActive && digitalRead(CANCEL_BTN) == HIGH) {
    Serial.println("⚠️ Alert Canceled by User.");
    buzzerActive = false;
    alertPending = false;
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Auto Tilt Detection
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float roll = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180.0 / PI;

  if (!buzzerActive && (abs(pitch) > TILT_THRESHOLD || abs(roll) > TILT_THRESHOLD)) {
    Serial.println("⚠️ Tilt threshold exceeded. Starting alert countdown...");
    buzzerActive = true;
    alertPending = true;
    buzzerStartTime = millis();
    digitalWrite(BUZZER_PIN, HIGH);
  }

  // Handle buzzer grace period timeout
  if (buzzerActive && alertPending && millis() - buzzerStartTime >= BUZZER_DURATION) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
    alertPending = false;
    Serial.println("⏱️ Grace period over. Sending Accident Alert.");
    sendSMS(emergencyContact, "🚨 Accident Detected! No response during grace period.");
  }

  delay(100); // Efficient timing loop
}

// Send SMS with location link if GPS is valid
void sendSMS(const char* number, const String& message) {
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();

    String fullMsg = message + "\n📍 https://www.google.com/maps/search/?api=1&query=" +
                     String(lat, 6) + "," + String(lng, 6);

    sim800.println("AT+CMGS=\"" + String(number) + "\"");
    delay(500);
    sim800.print(fullMsg);
    delay(300);
    sim800.write(26); // Ctrl+Z
    Serial.println("📤 SMS sent to " + String(number));
  } else {
    Serial.println("⚠️ GPS not fixed. SMS not sent.");
  }
}

// Send basic AT command and print response
void sendAT(const char* cmd) {
  sim800.println(cmd);
  delay(300);
  while (sim800.available()) {
    Serial.write(sim800.read());
  }
}
