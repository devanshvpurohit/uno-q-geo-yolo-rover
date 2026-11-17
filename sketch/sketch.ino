#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

// =====================================================
// DEVICE CONFIGURATION
// =====================================================

uint8_t imuAddr = 0x68;

#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT  0x3B
#define REG_GYRO_XOUT   0x43

#define SERIAL_BAUD     115200

TinyGPSPlus gps;

// =====================================================
// IMU VARIABLES
// =====================================================

bool imuReady = false;
bool fallAlert = false;
String imuStatusMsg = "INITIALIZING";

int32_t gyroOffsetX = 0;
int32_t gyroOffsetY = 0;
int32_t gyroOffsetZ = 0;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

float roll = 0;
float pitch = 0;

bool orientationInitialized = false;

const float ACCEL_SCALE = 16384.0; // +/- 2g
const float GYRO_SCALE  = 131.0;   // +/- 250 deg/s

unsigned long lastIMUUpdate = 0;
unsigned long lastPrint = 0;
unsigned long lastIMURetry = 0;
unsigned long lastBaudSwitch = 0;

const uint32_t baudOptions[] = {9600, 38400, 115200};
uint8_t baudIdx = 0;

const unsigned long PRINT_INTERVAL = 1000;

// =====================================================
// LOW-LEVEL I2C FUNCTIONS
// =====================================================

bool writeRegister(byte reg, byte value) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(byte reg, byte *buffer, byte length) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);

  // Try repeated start, fallback to standard stop
  if (Wire.endTransmission(false) != 0) {
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) {
      return false;
    }
  }

  byte received = Wire.requestFrom((uint8_t)imuAddr, length);
  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (byte i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

int16_t combineBytes(byte highByte, byte lowByte) {
  return (int16_t)(((uint16_t)highByte << 8) | lowByte);
}

// =====================================================
// IMU INITIALIZATION & SCAN
// =====================================================

bool initializeIMU() {
  byte id = 0;

  // Try address 0x68 first, then 0x69
  imuAddr = 0x68;
  if (!readRegisters(REG_WHO_AM_I, &id, 1)) {
    imuAddr = 0x69;
    if (!readRegisters(REG_WHO_AM_I, &id, 1)) {
      imuStatusMsg = "I2C READ FAILED (Check SDA/SCL pins)";
      return false;
    }
  }

  if (id == 0x68) {
    imuStatusMsg = "MPU6050 (0x68)";
  } else if (id == 0x70 || id == 0x71) {
    imuStatusMsg = "MPU6500 (0x68)";
  } else {
    imuStatusMsg = "ID 0x" + String(id, HEX);
  }

  // Wake up IMU
  if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
    imuStatusMsg = "WAKEUP FAILED";
    return false;
  }
  delay(50);

  // Sample rate = 100 Hz
  writeRegister(REG_SMPLRT_DIV, 0x09);
  // Digital low-pass filter
  writeRegister(REG_CONFIG, 0x03);
  // Gyroscope range = +/- 250 deg/s
  writeRegister(REG_GYRO_CONFIG, 0x00);
  // Accelerometer range = +/- 2g
  writeRegister(REG_ACCEL_CONFIG, 0x00);
  delay(50);

  return true;
}

// =====================================================
// GYROSCOPE CALIBRATION
// =====================================================

void calibrateGyroscope() {
  const int samples = 500;
  int64_t sumX = 0, sumY = 0, sumZ = 0;
  int successfulSamples = 0;
  byte data[6];

  for (int i = 0; i < samples; i++) {
    if (readRegisters(REG_GYRO_XOUT, data, 6)) {
      int16_t rawX = combineBytes(data[0], data[1]);
      int16_t rawY = combineBytes(data[2], data[3]);
      int16_t rawZ = combineBytes(data[4], data[5]);

      sumX += rawX;
      sumY += rawY;
      sumZ += rawZ;

      successfulSamples++;
    }
    delay(2);
  }

  if (successfulSamples < 350) {
    imuReady = false;
    imuStatusMsg = "CALIB FAILED";
    return;
  }

  gyroOffsetX = sumX / successfulSamples;
  gyroOffsetY = sumY / successfulSamples;
  gyroOffsetZ = sumZ / successfulSamples;
  imuStatusMsg = "WORKING";
}

// =====================================================
// READ IMU DATA
// =====================================================

bool readIMU() {
  byte data[14];
  if (!readRegisters(REG_ACCEL_XOUT, data, 14)) {
    return false;
  }

  int16_t rawAX = combineBytes(data[0], data[1]);
  int16_t rawAY = combineBytes(data[2], data[3]);
  int16_t rawAZ = combineBytes(data[4], data[5]);

  int16_t rawGX = combineBytes(data[8], data[9]);
  int16_t rawGY = combineBytes(data[10], data[11]);
  int16_t rawGZ = combineBytes(data[12], data[13]);

  // Acceleration in g
  ax = rawAX / ACCEL_SCALE;
  ay = rawAY / ACCEL_SCALE;
  az = rawAZ / ACCEL_SCALE;

  // Gyroscope in degrees/second
  gx = (rawGX - gyroOffsetX) / GYRO_SCALE;
  gy = (rawGY - gyroOffsetY) / GYRO_SCALE;
  gz = (rawGZ - gyroOffsetZ) / GYRO_SCALE;

  // Fall detection
  float totalG = sqrt(ax * ax + ay * ay + az * az);
  if (totalG > 2.8 || (totalG < 0.25 && totalG > 0.02)) {
    fallAlert = true;
  } else {
    fallAlert = false;
  }

  return true;
}

// =====================================================
// ROLL AND PITCH CALCULATION
// =====================================================

void calculateOrientation(float dt) {
  float accelRoll = atan2(ay, az) * 180.0 / PI;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  if (!orientationInitialized) {
    roll = accelRoll;
    pitch = accelPitch;
    orientationInitialized = true;
    return;
  }

  // Complementary filter
  roll  = 0.98 * (roll + gx * dt) + 0.02 * accelRoll;
  pitch = 0.98 * (pitch + gy * dt) + 0.02 * accelPitch;
}

// =====================================================
// GPS PROCESSING & AUTO-BAUD
// =====================================================

void processGPS() {
  while (Serial1.available() > 0) {
    char incoming = Serial1.read();
    gps.encode(incoming);
  }

  // If 0 bytes received after 5 seconds, cycle baud rate (9600 -> 38400 -> 115200)
  if (gps.charsProcessed() == 0 && (millis() - lastBaudSwitch > 5000)) {
    lastBaudSwitch = millis();
    baudIdx = (baudIdx + 1) % 3;
    Serial1.end();
    Serial1.begin(baudOptions[baudIdx]);
  }
}

// =====================================================
// TELEMETRY PACKET FOR LINUX / WEB
// =====================================================

String get_telemetry() {
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
  tel += String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3);
  tel += ",";
  tel += String(gx, 2) + "," + String(gy, 2) + "," + String(gz, 2);
  tel += ",";
  tel += String(pitch, 1) + "," + String(roll, 1);
  tel += ",";
  tel += (fallAlert ? "1" : "0");
  tel += ",";
  tel += String(gps.charsProcessed());
  tel += ",";
  tel += imuStatusMsg;

  return tel;
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  // Register Bridge with Linux immediately
  Bridge.begin();
  Bridge.provide("get_telemetry", get_telemetry);

  Serial.begin(SERIAL_BAUD);
  delay(1000);

  // Initialize I2C Bus
  Wire.begin();
  Wire.setClock(100000);

  // Initialize GPS on Hardware Serial1 (Pins 0/1)
  Serial1.begin(baudOptions[baudIdx]);

  // Initial IMU try
  imuReady = initializeIMU();
  if (imuReady) {
    calibrateGyroscope();
  }

  lastIMUUpdate = millis();
  lastPrint = millis();
  lastIMURetry = millis();
  lastBaudSwitch = millis();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  processGPS();

  // Re-try IMU if disconnected every 2 seconds
  if (!imuReady && (millis() - lastIMURetry > 2000)) {
    lastIMURetry = millis();
    imuReady = initializeIMU();
    if (imuReady) {
      calibrateGyroscope();
    }
  }

  // 10Hz IMU computation
  if (imuReady && (millis() - lastIMUUpdate >= 100)) {
    unsigned long now = millis();
    float dt = (now - lastIMUUpdate) / 1000.0;
    lastIMUUpdate = now;

    if (readIMU() && dt > 0 && dt < 0.2) {
      calculateOrientation(dt);
    }
  }

  // 1Hz Bridge notification
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    Bridge.notify("on_telemetry", get_telemetry());
  }
}
