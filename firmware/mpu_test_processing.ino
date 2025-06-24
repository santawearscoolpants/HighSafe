#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

Adafruit_MPU6050 mpu;
Madgwick filter; // Create Madgwick filter object

// Raw sensor bias
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("MPU6050 found!");

  // Calibrate gyro bias
  Serial.println("Calibrating gyro, keep still...");
  float sum_x = 0, sum_y = 0, sum_z = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum_x += g.gyro.x;
    sum_y += g.gyro.y;
    sum_z += g.gyro.z;
    delay(5);
  }
  gyro_bias_x = sum_x / samples;
  gyro_bias_y = sum_y / samples;
  gyro_bias_z = sum_z / samples;
  Serial.println("Gyro calibration done!");

  // Init filter
  filter.begin(100); // expected sample rate in Hz
  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Bias corrected gyro in rad/s
  float gx = g.gyro.x - gyro_bias_x;
  float gy = g.gyro.y - gyro_bias_y;
  float gz = g.gyro.z - gyro_bias_z;

  // Convert gyro to deg/s (Madgwick expects deg/s)
  gx *= 180.0 / PI;
  gy *= 180.0 / PI;
  gz *= 180.0 / PI;

  // Acceleration in m/s^2 -> convert to g (1g = 9.81 m/s^2)
  float ax = a.acceleration.x / 9.81;
  float ay = a.acceleration.y / 9.81;
  float az = a.acceleration.z / 9.81;

  // Compute dt
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Update Madgwick filter
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // Get Euler angles in degrees
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  // Send to Processing as CSV
  Serial.print(yaw);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(roll);

  delay(10); // ~100 Hz
}
