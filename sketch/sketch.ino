#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

/*
 * SURDAS: Assistive GPS + IMU Platform for Arduino UNO Q
 * STM32U585 Zephyr MCU Core
 *
 * Pinout:
 * - GPS (NEO-6M / M8N):
 *     GPS TX -> Arduino Pin 0 (RX)
 *     GPS RX -> Arduino Pin 1 (TX)
 * - IMU (MPU-6050):
 *     SDA -> Arduino SDA (Pin A4 / Qwiic)
 *     SCL -> Arduino SCL (Pin A5 / Qwiic)
 */

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

bool imuReady = false;
bool fallAlert = false;
unsigned long gpsBytesCount = 0;
unsigned long lastUpdate = 0;

// Format: $TEL,imu_ok,lat,lng,sats,speed_kmh,alt_m,ax,ay,az,gx,gy,gz,pitch,roll,fall,gps_bytes
String currentTelemetry = "$TEL,0,0.0,0.0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0";

String get_telemetry() {
  return currentTelemetry;
}

bool initIMU() {
  // Try default I2C address 0x68, then secondary address 0x69
  if (mpu.begin(0x68) || mpu.begin(0x69)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    digitalWrite(LED_BUILTIN, HIGH);
    return true;
  }
  return false;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize Hardware UART Serial1 for GPS (Pins 0/1)
  Serial1.begin(9600);

  // Initialize I2C Bus
  Wire.begin();
  delay(100);

  // Attempt initial IMU connection
  imuReady = initIMU();

  // Initialize Router Bridge to Linux Qualcomm MPU
  Bridge.begin();
  Bridge.provide_safe("get_telemetry", get_telemetry);
  Bridge.provide("get_telemetry_raw", get_telemetry);
}

void loop() {
  // 1. Process GPS Characters from Hardware Serial1
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gpsBytesCount++;
    gps.encode(c);
  }

  // 2. Periodic Telemetry Update (10Hz)
  if (millis() - lastUpdate >= 100) {
    lastUpdate = millis();

    // Re-check IMU connection if not yet connected
    if (!imuReady) {
      imuReady = initIMU();
    }

    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float pitch = 0, roll = 0;

    if (imuReady) {
      sensors_event_t a, g, temp;
      if (mpu.getEvent(&a, &g, &temp)) {
        ax = a.acceleration.x;
        ay = a.acceleration.y;
        az = a.acceleration.z;
        gx = g.gyro.x;
        gy = g.gyro.y;
        gz = g.gyro.z;

        // Pitch & Roll computation
        roll  = atan2(ay, az) * 180.0 / PI;
        pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

        // Fall detection
        float totalG = sqrt(ax * ax + ay * ay + az * az) / 9.81;
        if (totalG > 2.8 || (totalG < 0.25 && totalG > 0.02)) {
          fallAlert = true;
        } else {
          fallAlert = false;
        }
      }
    }

    // Build structured telemetry packet
    String tel = "$TEL,";
    tel += (imuReady ? "1" : "0");
    tel += ",";

    if (gps.location.isValid()) {
      tel += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    } else {
      tel += "0.0,0.0";
    }

    tel += ",";
    tel += String(gps.satellites.isValid() ? gps.satellites.value() : 0);
    tel += ",";
    tel += String(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 1);
    tel += ",";
    tel += String(gps.altitude.isValid() ? gps.altitude.meters() : 0.0, 1);
    tel += ",";
    tel += String(ax, 2) + "," + String(ay, 2) + "," + String(az, 2);
    tel += ",";
    tel += String(gx, 2) + "," + String(gy, 2) + "," + String(gz, 2);
    tel += ",";
    tel += String(pitch, 1) + "," + String(roll, 1);
    tel += ",";
    tel += (fallAlert ? "1" : "0");
    tel += ",";
    tel += String(gpsBytesCount);

    currentTelemetry = tel;

    // Push notification to Linux bridge
    Bridge.notify("on_telemetry", currentTelemetry);
  }
}
