#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

/*
 * SURDAS: Smart Wearable Vision & Navigation Assistant for the Blind
 * Arduino UNO Q (STM32U585 MCU Core)
 *
 * Hardware Peripherals:
 * - GPS (NEO-6M / M8N): Hardware UART Serial1 (Pin 0 RX, Pin 1 TX)
 * - IMU (MPU-6050): I2C (SDA, SCL) for orientation & fall detection
 * - Haptic Feedback / Buzzers:
 *     Pin D5: Left Haptic Motor
 *     Pin D6: Center Haptic Motor / Buzzer
 *     Pin D7: Right Haptic Motor
 * - SOS Emergency Button: Pin D2 (INPUT_PULLUP)
 */

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

// Haptic & Alert Pins
const int HAPTIC_LEFT_PIN   = 5;
const int HAPTIC_CENTER_PIN = 6;
const int HAPTIC_RIGHT_PIN  = 7;
const int SOS_BUTTON_PIN    = 2;

bool imuReady = false;
bool fallDetected = false;
bool sosActive = false;

// Telemetry state string
// Format: $TEL,imu_ok,lat,lng,sats,speed,ax,ay,az,fall,sos
String currentTelemetry = "$TEL,0,0.0,0.0,0,0.0,0.0,0.0,0.0,0,0";
unsigned long lastTelemetryUpdate = 0;
unsigned long lastFallCheck = 0;

void setHaptics(bool left, bool center, bool right) {
  digitalWrite(HAPTIC_LEFT_PIN, left ? HIGH : LOW);
  digitalWrite(HAPTIC_CENTER_PIN, center ? HIGH : LOW);
  digitalWrite(HAPTIC_RIGHT_PIN, right ? HIGH : LOW);
}

// RPC method called by Python when an obstacle or hazard is detected
// cmd: 'L' (left obstacle), 'R' (right obstacle), 'C' (center obstacle/stop), 'S' (clear/silent)
bool haptic_feedback(String cmd) {
  if (cmd == "L") {
    setHaptics(true, false, false);
  } else if (cmd == "R") {
    setHaptics(false, false, true);
  } else if (cmd == "C" || cmd == "S_HAZARD") {
    setHaptics(true, true, true);
  } else {
    setHaptics(false, false, false);
  }
  return true;
}

// RPC method to query sensor telemetry
String get_telemetry() {
  return currentTelemetry;
}

void setup() {
  // Setup Haptics & SOS button
  pinMode(HAPTIC_LEFT_PIN, OUTPUT);
  pinMode(HAPTIC_CENTER_PIN, OUTPUT);
  pinMode(HAPTIC_RIGHT_PIN, OUTPUT);
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);
  setHaptics(false, false, false);

  // Initialize Hardware UART Serial1 for GPS (9600 baud)
  Serial1.begin(9600);

  // Initialize I2C IMU (MPU6050)
  Wire.begin();
  if (mpu.begin()) {
    imuReady = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Initialize Arduino Router Bridge to Qualcomm Linux MPU
  Bridge.begin();
  Bridge.provide("haptic_feedback", haptic_feedback);
  Bridge.provide("get_telemetry", get_telemetry);
}

void loop() {
  // 1. Read incoming GPS data
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // 2. Read SOS Button (Active LOW)
  if (digitalRead(SOS_BUTTON_PIN) == LOW) {
    sosActive = true;
    setHaptics(true, true, true);
  } else {
    sosActive = false;
  }

  // 3. Fall Detection check via IMU (10Hz)
  if (millis() - lastFallCheck >= 100) {
    lastFallCheck = millis();

    sensors_event_t a, g, temp;
    float ax = 0, ay = 0, az = 0;

    if (imuReady) {
      mpu.getEvent(&a, &g, &temp);
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;

      // Calculate total acceleration magnitude
      float totalG = sqrt(ax * ax + ay * ay + az * az) / 9.81;

      // Impact or sudden fall threshold (> 2.8g or near freefall < 0.25g)
      if (totalG > 2.8 || totalG < 0.25) {
        fallDetected = true;
      } else {
        fallDetected = false;
      }
    }

    // 4. Build Telemetry string
    // Format: $TEL,imu_ok,lat,lng,sats,speed_kmh,ax,ay,az,fall,sos
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
    tel += String(ax, 2) + "," + String(ay, 2) + "," + String(az, 2);
    tel += ",";
    tel += (fallDetected ? "1" : "0");
    tel += ",";
    tel += (sosActive ? "1" : "0");

    currentTelemetry = tel;
  }
}
