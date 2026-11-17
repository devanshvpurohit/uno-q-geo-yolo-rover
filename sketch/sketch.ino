#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

// =====================================================
// DEVICE CONFIGURATION
// =====================================================

#define IMU_ADDR        0x68

#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT  0x3B
#define REG_GYRO_XOUT   0x43

#define GPS_BAUD        9600
#define SERIAL_BAUD     115200

TinyGPSPlus gps;

// =====================================================
// IMU & TELEMETRY VARIABLES
// =====================================================

bool imuReady = false;
bool fallAlert = false;

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
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(byte reg, byte *buffer, byte length) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  byte received = Wire.requestFrom((uint8_t)IMU_ADDR, length);
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
// IMU INITIALIZATION
// =====================================================

bool initializeIMU() {
  byte id = 0;
  Serial.println("\nINITIALIZING IMU...");

  if (!readRegisters(REG_WHO_AM_I, &id, 1)) {
    Serial.println("IMU: WHO_AM_I READ FAILED (Check SDA/SCL wiring)");
    return false;
  }

  Serial.print("IMU WHO_AM_I: 0x");
  Serial.println(id, HEX);

  if (id == 0x68) {
    Serial.println("DEVICE IDENTIFIED: MPU6050");
  } else if (id == 0x70) {
    Serial.println("DEVICE IDENTIFIED: MPU6500");
  } else {
    Serial.println("UNKNOWN IMU DEVICE ID, attempting config anyway...");
  }

  // Wake up IMU
  if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
    Serial.println("IMU WAKEUP FAILED");
    return false;
  }
  delay(100);

  // Sample rate = 100 Hz
  writeRegister(REG_SMPLRT_DIV, 0x09);
  // Digital low-pass filter
  writeRegister(REG_CONFIG, 0x03);
  // Gyroscope range = +/- 250 deg/s
  writeRegister(REG_GYRO_CONFIG, 0x00);
  // Accelerometer range = +/- 2g
  writeRegister(REG_ACCEL_CONFIG, 0x00);
  delay(100);

  Serial.println("IMU INITIALIZATION SUCCESS");
  return true;
}

// =====================================================
// GYROSCOPE CALIBRATION
// =====================================================

void calibrateGyroscope() {
  Serial.println("\n================================");
  Serial.println("GYROSCOPE CALIBRATION");
  Serial.println("KEEP THE SENSOR COMPLETELY STILL");
  Serial.println("================================");

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
    delay(3);
  }

  if (successfulSamples < 400) {
    Serial.println("GYROSCOPE CALIBRATION FAILED");
    imuReady = false;
    return;
  }

  gyroOffsetX = sumX / successfulSamples;
  gyroOffsetY = sumY / successfulSamples;
  gyroOffsetZ = sumZ / successfulSamples;

  Serial.println("CALIBRATION COMPLETE");
  Serial.print("OFFSETS: X="); Serial.print(gyroOffsetX);
  Serial.print(" Y="); Serial.print(gyroOffsetY);
  Serial.print(" Z="); Serial.println(gyroOffsetZ);
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
// TEMPERATURE
// =====================================================

float readIMUTemperature() {
  byte data[2];
  if (!readRegisters(0x41, data, 2)) {
    return NAN;
  }
  int16_t rawTemp = combineBytes(data[0], data[1]);
  return (rawTemp / 340.0) + 36.53;
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
// GPS PROCESSING
// =====================================================

void processGPS() {
  while (Serial1.available() > 0) {
    char incoming = Serial1.read();
    gps.encode(incoming);
  }
}

// =====================================================
// PRINT FUNCTIONS (SERIAL MONITOR DEBUG)
// =====================================================

void printIMU() {
  Serial.println("\n========== IMU DATA ==========");
  if (!imuReady) {
    Serial.println("IMU STATUS: NOT INITIALIZED");
    return;
  }
  if (!readIMU()) {
    Serial.println("IMU STATUS: READ ERROR");
    return;
  }

  unsigned long now = millis();
  float dt = 0;
  if (lastIMUUpdate != 0) {
    dt = (now - lastIMUUpdate) / 1000.0;
  }
  lastIMUUpdate = now;

  if (dt > 0 && dt < 0.2) {
    calculateOrientation(dt);
  }

  Serial.println("IMU STATUS: WORKING");
  Serial.print("ACCEL (g): X="); Serial.print(ax, 3);
  Serial.print(" Y="); Serial.print(ay, 3);
  Serial.print(" Z="); Serial.println(az, 3);

  Serial.print("GYRO (deg/s): X="); Serial.print(gx, 2);
  Serial.print(" Y="); Serial.print(gy, 2);
  Serial.print(" Z="); Serial.println(gz, 2);

  Serial.print("ORIENTATION: ROLL="); Serial.print(roll, 2);
  Serial.print(" deg, PITCH="); Serial.print(pitch, 2); Serial.println(" deg");

  float temp = readIMUTemperature();
  if (!isnan(temp)) {
    Serial.print("TEMPERATURE: "); Serial.print(temp, 1); Serial.println(" C");
  }
  Serial.println("==============================");
}

void printGPS() {
  Serial.println("\n========== NEO-6M GPS ==========");
  Serial.print("GPS BYTES PROCESSED: "); Serial.println(gps.charsProcessed());
  Serial.print("VALID NMEA: "); Serial.println(gps.passedChecksum());
  Serial.print("FAILED NMEA: "); Serial.println(gps.failedChecksum());

  Serial.print("SATELLITES: ");
  if (gps.satellites.isValid()) {
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("SEARCHING...");
  }

  Serial.print("GPS FIX: ");
  if (gps.location.isValid()) {
    Serial.println("AVAILABLE");
    Serial.print("LAT: "); Serial.println(gps.location.lat(), 6);
    Serial.print("LNG: "); Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("NOT AVAILABLE (Move outdoors or near window)");
  }

  if (gps.speed.isValid()) {
    Serial.print("SPEED: "); Serial.print(gps.speed.kmph(), 1); Serial.println(" km/h");
  }
  if (gps.altitude.isValid()) {
    Serial.print("ALTITUDE: "); Serial.print(gps.altitude.meters(), 1); Serial.println(" m");
  }
  Serial.println("===============================");
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

  // Publish notification to Linux Router Bridge
  Bridge.notify("on_telemetry", currentTelemetry);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);

  Serial.println("\n################################");
  Serial.println(" GPS + IMU MONITORING SYSTEM");
  Serial.println("       ARDUINO UNO Q");
  Serial.println("################################");

  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("\nI2C INITIALIZED");

  // Initialize GPS UART on Pins 0 (RX) and 1 (TX)
  Serial1.begin(GPS_BAUD);
  Serial.println("GPS UART INITIALIZED ON SERIAL1 (PINS 0/1)");

  // Initialize IMU
  imuReady = initializeIMU();
  if (imuReady) {
    calibrateGyroscope();
  }

  // Initialize Bridge to Linux MPU
  Bridge.begin();
  Bridge.provide_safe("get_telemetry", get_telemetry);
  Bridge.provide("get_telemetry_raw", get_telemetry);

  Serial.println("\nSYSTEM INITIALIZATION COMPLETE");
  Serial.println("GPS AND IMU MONITOR STARTED");

  lastIMUUpdate = millis();
  lastPrint = millis();
  lastBridgeSync = millis();
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

    if (!imuReady) {
      imuReady = initializeIMU();
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

  // 3. Serial Monitor Diagnostic Printout every 1 second
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    printIMU();
    printGPS();
    Serial.println("\n################################");
  }
}
