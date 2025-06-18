#include <Wire.h>
#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>

// SIM and target
const char simPIN[] = "";
#define SMS_TARGET "+233249097323"

// Modem config
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 512

// Serial interfaces
#define SerialMon Serial
#define SerialAT Serial1
#define SerialGPS Serial2

// T-Call ESP32 Pins
#define MODEM_RST         5
#define MODEM_PWKEY       4
#define MODEM_POWER_ON    23
#define MODEM_TX          27
#define MODEM_RX          26
#define I2C_SDA           21
#define I2C_SCL           22
#define BUTTON_PIN        12   // Push button input
#define GPS_RX            34   // From GPS TX
#define GPS_TX            12   // Not used (set unused or -1)

// Power chip
#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

TinyGsm modem(SerialAT);
TinyGPSPlus gps;

bool setPowerBoostKeepOn(int en){
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  Wire.write(en ? 0x37 : 0x35);
  return Wire.endTransmission() == 0;
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  SerialMon.begin(115200);
  delay(10);

  Wire.begin(I2C_SDA, I2C_SCL);
  setPowerBoostKeepOn(1);

  // Power up modem
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Init SIM800
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  modem.restart();

  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }

  // Init GPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    SerialMon.println("Button pressed. Acquiring GPS...");
    if (getGPSLocation()) {
      String location = "Location: " + String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6);
      SerialMon.println("Sending SMS: " + location);
      if (modem.sendSMS(SMS_TARGET, location)) {
        SerialMon.println("SMS sent successfully!");
      } else {
        SerialMon.println("Failed to send SMS.");
      }
    } else {
      SerialMon.println("Could not get GPS location.");
    }
    delay(5000); // Debounce and prevent spam
  }
  delay(100);
}

bool getGPSLocation() {
  unsigned long start = millis();
  while (millis() - start < 10000) { // Try for 10 seconds
    while (SerialGPS.available()) {
      gps.encode(SerialGPS.read());
      if (gps.location.isUpdated()) {
        return true;
      }
    }
  }
  return false;
}
