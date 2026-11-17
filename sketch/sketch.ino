#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// GPS Configuration (NEO-6M / NEO-M8N)
// Connect GPS TX -> Arduino Pin 4 (RX), GPS RX -> Arduino Pin 3 (TX)
static const int GPS_RX_PIN = 4;
static const int GPS_TX_PIN = 3;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
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

unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 200; // 5Hz stream

void setup() {
  // Serial link to Qualcomm Linux MPU
  Serial.begin(115200);
  gpsSerial.begin(9600);

  // Motor Driver pin setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotors();

  // Initialize I2C IMU
  Wire.begin();
  if (mpu.begin()) {
    imuAvailable = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void loop() {
  // 1. Ingest GPS NMEA sentences
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. Read incoming commands from Qualcomm Linux MPU
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }

  // 3. Emit structured telemetry packet
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }
}

void handleCommand(char cmd) {
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

    case 'L': // Turn Left (counter-clockwise)
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, DEFAULT_SPEED); analogWrite(ENB, DEFAULT_SPEED);
      break;

    case 'R': // Turn Right (clockwise)
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

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void sendTelemetry() {
  sensors_event_t a, g, temp;
  if (imuAvailable) {
    mpu.getEvent(&a, &g, &temp);
  } else {
    a.acceleration.x = 0; a.acceleration.y = 0; a.acceleration.z = 0;
    g.gyro.x = 0; g.gyro.y = 0; g.gyro.z = 0;
  }

  // CSV format: $TEL,lat,lng,speed_kmh,pitch_ax,roll_ay,yaw_gz
  Serial.print("$TEL,");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
    Serial.print(",");
    Serial.print(gps.speed.kmph(), 1);
  } else {
    Serial.print("0.0,0.0,0.0");
  }
  Serial.print(",");
  Serial.print(a.acceleration.x, 2);
  Serial.print(",");
  Serial.print(a.acceleration.y, 2);
  Serial.print(",");
  Serial.println(g.gyro.z, 2);
}
