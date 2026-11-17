#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

/*
 * SURDAS: Assistive GPS + IMU Core for Arduino UNO Q (4GB)
 * Microcontroller: STM32U585 running Zephyr RTOS
 *
 * Hardware Peripherals:
 * - GPS (NEO-6M / M8N): Hardware UART Serial1 (Pin 0 RX, Pin 1 TX)
 * - IMU (MPU-6050): I2C (Pins SDA, SCL / Qwiic)
 */

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

bool imuReady = false;
bool fallAlert = false;

// Format: $TEL,imu_ok,lat,lng,sats,speed_kmh,alt_m,ax,ay,az,gx,gy,gz,pitch,roll,fall
String currentTelemetry = "$TEL,0,0.0,0.0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0";
unsigned long lastUpdate = 0;

String get_telemetry() {
  return currentTelemetry;
}

void setup() {
  // Built-in status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Hardware UART Serial1 for GPS (9600 baud)
  Serial1.begin(9600);

  // I2C IMU (MPU6050)
  Wire.begin();
  if (mpu.begin()) {
    imuReady = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // Arduino Router Bridge to Qualcomm Linux MPU
  Bridge.begin();
  Bridge.provide("get_telemetry", get_telemetry);
}

void loop() {
  // 1. Ingest GPS NMEA sentences from Hardware Serial1
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // 2. Refresh Telemetry at 10Hz
  if (millis() - lastUpdate >= 100) {
    lastUpdate = millis();

    sensors_event_t a, g, temp;
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float pitch = 0, roll = 0;

    if (imuReady) {
      mpu.getEvent(&a, &g, &temp);
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;
      gx = g.gyro.x;
      gy = g.gyro.y;
      gz = g.gyro.z;

      // Estimate pitch and roll from accelerometer (in degrees)
      roll  = atan2(ay, az) * 180.0 / PI;
      pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

      // Fall / impact detection (magnitude threshold)
      float totalG = sqrt(ax * ax + ay * ay + az * az) / 9.81;
      if (totalG > 2.8 || (totalG < 0.25 && totalG > 0.01)) {
        fallAlert = true;
      } else {
        fallAlert = false;
      }
    }

    // Build payload: $TEL,imu_ok,lat,lng,sats,speed_kmh,alt_m,ax,ay,az,gx,gy,gz,pitch,roll,fall
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

    currentTelemetry = tel;
  }
}
