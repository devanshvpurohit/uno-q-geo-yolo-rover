#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

// GPS Configuration on Hardware UART (Serial1)
// On Arduino UNO Q:
// GPS Module TX -> Arduino Pin 0 (RX)
// GPS Module RX -> Arduino Pin 1 (TX)
TinyGPSPlus gps;

// IMU Configuration (MPU-6050 via I2C / Qwiic)
Adafruit_MPU6050 mpu;
bool imuAvailable = false;

// Motor Driver Pins (L298N / TB6612FNG)
const int ENA = 5;  // PWM Left
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 10; // PWM Right

const int DEFAULT_SPEED = 190;

String currentTelemetry = "$TEL,0.0,0.0,0.0,0.0,0.0,0.0";
unsigned long lastTelemetryUpdate = 0;

// Motor control implementation
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void executeCommand(char cmd) {
  switch (cmd) {
    case 'F': // Forward
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, DEFAULT_SPEED); analogWrite(ENB, DEFAULT_SPEED);
      break;

    case 'B': // Backward
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      analogWrite(ENA, DEFAULT_SPEED); analogWrite(ENB, DEFAULT_SPEED);
      break;

    case 'L': // Spin Left
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, DEFAULT_SPEED); analogWrite(ENB, DEFAULT_SPEED);
      break;

    case 'R': // Spin Right
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      analogWrite(ENA, DEFAULT_SPEED); analogWrite(ENB, DEFAULT_SPEED);
      break;

    case 'S': // Stop
    default:
      stopMotors();
      break;
  }
}

// RPC functions exposed to Python via Arduino Router Bridge
bool drive_motor(String cmd) {
  if (cmd.length() > 0) {
    executeCommand(cmd.charAt(0));
  }
  return true;
}

String get_telemetry() {
  return currentTelemetry;
}

void setup() {
  // Setup Motor Driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotors();

  // Initialize Hardware UART Serial1 for GPS (Pins D0/D1)
  Serial1.begin(9600);

  // Initialize I2C IMU
  Wire.begin();
  if (mpu.begin()) {
    imuAvailable = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Initialize Router Bridge to Linux Qualcomm MPU
  Bridge.begin();
  Bridge.provide("drive_motor", drive_motor);
  Bridge.provide("get_telemetry", get_telemetry);
}

void loop() {
  // 1. Ingest GPS data from hardware UART Serial1
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // 2. Periodically update telemetry cache
  if (millis() - lastTelemetryUpdate >= 100) { // 10Hz internal refresh
    lastTelemetryUpdate = millis();

    sensors_event_t a, g, temp;
    if (imuAvailable) {
      mpu.getEvent(&a, &g, &temp);
    } else {
      a.acceleration.x = 0; a.acceleration.y = 0; a.acceleration.z = 0;
      g.gyro.x = 0; g.gyro.y = 0; g.gyro.z = 0;
    }

    String tel = "$TEL,";
    if (gps.location.isValid()) {
      tel += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," + String(gps.speed.kmph(), 1);
    } else {
      tel += "0.0,0.0,0.0";
    }
    tel += "," + String(a.acceleration.x, 2) + "," + String(a.acceleration.y, 2) + "," + String(g.gyro.z, 2);
    currentTelemetry = tel;
  }
}
