#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

// =====================================================
// DEVICE CONFIGURATION
// =====================================================

uint8_t imuAddress = 0x68; // Auto-scanned: 0x68 or 0x69

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
// IMU & TELEMETRY VARIABLES
// =====================================================

bool imuReady = false;
bool fallAlert = false;
int i2cDevicesFound = 0;

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
unsigned long lastBridgeSync = 0;
unsigned long lastBaudCheck = 0;

// GPS Baud cycling: 9600 -> 38400 -> 115200
const uint32_t baudRates[] = {9600, 38400, 115200};
uint8_t currentBaudIdx = 0;

const unsigned long PRINT_INTERVAL = 1000;
const unsigned long BRIDGE_INTERVAL = 100; // 10Hz sync with Linux MPU

String currentTelemetry = "$TEL,0,0.0,0.0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0";

// RPC method exposed to Linux MPU
String get_telemetry() {
  return currentTelemetry;
}

// =====================================================
// LOW-LEVEL I2C FUNCTIONS
// =====================================================

bool writeRegister(byte reg, byte value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(byte reg, byte *buffer, byte length) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);

  // Try repeated start, then standard stop if needed
  if (Wire.endTransmission(false) != 0) {
    Wire.beginTransmission(imuAddress);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) {
      return false;
    }
  }

  byte received = Wire.requestFrom((uint8_t)imuAddress, length);
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
// I2C SCANNER & IMU INITIALIZATION
// =====================================================

bool scanAndDetectIMU() {
  Serial.println("\n[I2C] Scanning bus for devices...");
  i2cDevicesFound = 0;
  uint8_t detected = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      i2cDevicesFound++;
      Serial.print("[I2C] Found device at 0x");
      Serial.println(addr, HEX);
      if (addr == 0x68 || addr == 0x69) {
        detected = addr;
      }
    }
  }

  if (detected != 0) {
    imuAddress = detected;
    Serial.print("[I2C] MPU-6050 selected at address 0x");
    Serial.println(imuAddress, HEX);
    return true;
  }

  Serial.println("[I2C] No MPU6050 found! If using A4/A5, try dedicated SDA/SCL pins near AREF.");
  return false;
}

bool initializeIMU() {
  if (!scanAndDetectIMU()) {
    return false;
  }

  byte id = 0;
  if (!readRegisters(REG_WHO_AM_I, &id, 1)) {
    Serial.println("[IMU] WHO_AM_I read failed.");
    return false;
  }

  Serial.print("[IMU] WHO_AM_I ID: 0x");
  Serial.println(id, HEX);

  // Wake up IMU
  if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
    Serial.println("[IMU] Wakeup failed.");
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

  Serial.println("[IMU] Initialization SUCCESS!");
  return true;
}

// =====================================================
// GYROSCOPE CALIBRATION
// =====================================================

void calibrateGyroscope() {
  Serial.println("\n================================");
  Serial.println("GYROSCOPE CALIBRATION");
  Serial.println("KEEP THE SENSOR STILL...");
  Serial.println("================================");

  const int samples = 300;
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

  if (successfulSamples < 200) {
    Serial.println("GYROSCOPE CALIBRATION FAILED");
    imuReady = false;
    return;
  }

  gyroOffsetX = sumX / successfulSamples;
  gyroOffsetY = sumY / successfulSamples;
  gyroOffsetZ = sumZ / successfulSamples;

  Serial.println("CALIBRATION COMPLETE");
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

  // Fall / impact detection
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
// GPS PROCESSING & AUTO-BAUD DETECT
// =====================================================

void processGPS() {
  while (Serial1.available() > 0) {
    char incoming = Serial1.read();
    gps.encode(incoming);
  }

  // If 0 bytes received after 6 seconds, try alternate baud rates (9600 -> 38400 -> 115200)
  if (gps.charsProcessed() == 0 && (millis() - lastBaudCheck > 6000)) {
    lastBaudCheck = millis();
    currentBaudIdx = (currentBaudIdx + 1) % 3;
    Serial1.end();
    Serial1.begin(baudRates[currentBaudIdx]);
    Serial.print("[GPS] Auto-baud trying: ");
    Serial.print(baudRates[currentBaudIdx]);
    Serial.println(" baud on Serial1 (Pins 0/1)");
  }
}

// =====================================================
// BUILD TELEMETRY PAYLOAD FOR LINUX / WEB
// =====================================================

void updateTelemetryPacket() {
  // Format: $TEL,imu_ok,lat,lng,sats,speed_kmh,alt_m,ax,ay,az,gx,gy,gz,pitch,roll,fall,gps_bytes
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

  currentTelemetry = tel;

  Bridge.notify("on_telemetry", currentTelemetry);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  // 1. Initialize Bridge immediately
  Bridge.begin();
  Bridge.provide_safe("get_telemetry", get_telemetry);
  Bridge.provide("get_telemetry", get_telemetry);

  Serial.println("\n################################");
  Serial.println(" GPS + IMU MONITORING SYSTEM");
  Serial.println("       ARDUINO UNO Q");
  Serial.println("################################");

  // 2. Initialize I2C
  Wire.begin();
  Wire.setClock(100000);

  // 3. Initialize GPS UART on Pins 0 (RX) and 1 (TX) at 9600
  Serial1.begin(baudRates[currentBaudIdx]);

  // 4. Initialize IMU
  imuReady = initializeIMU();
  if (imuReady) {
    calibrateGyroscope();
  }

  lastIMUUpdate = millis();
  lastPrint = millis();
  lastBridgeSync = millis();
  lastBaudCheck = millis();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  // 1. Process GPS Characters continuously
  processGPS();

  // 2. High-Frequency IMU & Bridge Sync (10Hz)
  if (millis() - lastBridgeSync >= BRIDGE_INTERVAL) {
    lastBridgeSync = millis();

    // Auto-retry IMU initialization if disconnected
    if (!imuReady) {
      imuReady = initializeIMU();
      if (imuReady) {
        calibrateGyroscope();
      }
    }

    if (imuReady) {
      if (readIMU()) {
        unsigned long now = millis();
        float dt = (lastIMUUpdate != 0) ? (now - lastIMUUpdate) / 1000.0 : 0.0;
        lastIMUUpdate = now;
        if (dt > 0 && dt < 0.2) {
          calculateOrientation(dt);
        }
      }
    }

    updateTelemetryPacket();
  }
}
