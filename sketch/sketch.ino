#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

// =====================================================
// DEVICE CONFIGURATION
// =====================================================

#define IMU_ADDR       0x68

#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG     0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT 0x3B
#define REG_GYRO_XOUT  0x43

#define GPS_BAUD       9600
#define SERIAL_BAUD    115200

TinyGPSPlus gps;

// =====================================================
// IMU VARIABLES
// =====================================================

bool imuReady = false;

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

const unsigned long PRINT_INTERVAL = 1000;

// =====================================================
// BRIDGE TELEMETRY FUNCTION FOR LINUX MPU
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

  float totalG = sqrt(ax * ax + ay * ay + az * az);
  tel += ((totalG > 2.8 || (totalG < 0.25 && totalG > 0.02)) ? "1" : "0");
  tel += ",";
  tel += String(gps.charsProcessed());

  return tel;
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
    Serial.println("IMU: WHO_AM_I READ FAILED");
    return false;
  }

  Serial.print("IMU WHO_AM_I: 0x");
  Serial.println(id, HEX);

  if (id == 0x68) {
    Serial.println("DEVICE IDENTIFIED: MPU6050");
  } else if (id == 0x70) {
    Serial.println("DEVICE IDENTIFIED: MPU6500");
  } else {
    Serial.println("UNKNOWN IMU DEVICE ID");
    return false;
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

  // Gyroscope range = +/- 250 degrees/sec
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

  const int samples = 1000;
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

  if (successfulSamples < 900) {
    Serial.println("GYROSCOPE CALIBRATION FAILED");
    imuReady = false;
    return;
  }

  gyroOffsetX = sumX / successfulSamples;
  gyroOffsetY = sumY / successfulSamples;
  gyroOffsetZ = sumZ / successfulSamples;

  Serial.println("\nCALIBRATION COMPLETE");
  Serial.print("GYRO OFFSET X: "); Serial.println(gyroOffsetX);
  Serial.print("GYRO OFFSET Y: "); Serial.println(gyroOffsetY);
  Serial.print("GYRO OFFSET Z: "); Serial.println(gyroOffsetZ);
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
// PRINT IMU INFORMATION
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
  Serial.println("\nACCELEROMETER (g)");
  Serial.print("X: "); Serial.println(ax, 4);
  Serial.print("Y: "); Serial.println(ay, 4);
  Serial.print("Z: "); Serial.println(az, 4);

  Serial.println("\nGYROSCOPE (deg/s)");
  Serial.print("X: "); Serial.println(gx, 4);
  Serial.print("Y: "); Serial.println(gy, 4);
  Serial.print("Z: "); Serial.println(gz, 4);

  Serial.println("\nORIENTATION");
  Serial.print("ROLL: "); Serial.print(roll, 2); Serial.println(" degrees");
  Serial.print("PITCH: "); Serial.print(pitch, 2); Serial.println(" degrees");

  float temperature = readIMUTemperature();
  if (!isnan(temperature)) {
    Serial.print("TEMPERATURE: "); Serial.print(temperature, 2); Serial.println(" C");
  }

  Serial.println("==============================");
}

// =====================================================
// PRINT GPS INFORMATION
// =====================================================

void printGPS() {
  Serial.println("\n========== NEO-6M GPS ==========");

  Serial.print("GPS BYTES: "); Serial.println(gps.charsProcessed());
  Serial.print("VALID NMEA: "); Serial.println(gps.passedChecksum());
  Serial.print("FAILED NMEA: "); Serial.println(gps.failedChecksum());

  Serial.print("SATELLITES: ");
  if (gps.satellites.isValid()) {
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("NOT AVAILABLE");
  }

  Serial.print("GPS FIX: ");
  if (gps.location.isValid()) {
    Serial.println("AVAILABLE");
    Serial.print("LATITUDE: "); Serial.println(gps.location.lat(), 6);
    Serial.print("LONGITUDE: "); Serial.println(gps.location.lng(), 6);
    Serial.print("LOCATION AGE: "); Serial.print(gps.location.age()); Serial.println(" ms");
  } else {
    Serial.println("NOT AVAILABLE");
  }

  if (gps.altitude.isValid()) {
    Serial.print("ALTITUDE: "); Serial.print(gps.altitude.meters(), 2); Serial.println(" m");
  }
  if (gps.speed.isValid()) {
    Serial.print("SPEED: "); Serial.print(gps.speed.kmph(), 2); Serial.println(" km/h");
  }
  if (gps.hdop.isValid()) {
    Serial.print("HDOP: "); Serial.println(gps.hdop.hdop(), 2);
  }
  if (gps.course.isValid()) {
    Serial.print("COURSE: "); Serial.print(gps.course.deg(), 2); Serial.println(" degrees");
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.print("GPS DATE: ");
    Serial.print(gps.date.day()); Serial.print("/");
    Serial.print(gps.date.month()); Serial.print("/");
    Serial.println(gps.date.year());

    Serial.print("GPS UTC TIME: ");
    if (gps.time.hour() < 10) Serial.print("0");
    Serial.print(gps.time.hour()); Serial.print(":");
    if (gps.time.minute() < 10) Serial.print("0");
    Serial.print(gps.time.minute()); Serial.print(":");
    if (gps.time.second() < 10) Serial.print("0");
    Serial.println(gps.time.second());
  }

  Serial.println("===============================");
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  // Bridge initialization for communication with Linux MPU
  Bridge.begin();
  Bridge.provide("get_telemetry", get_telemetry);

  Serial.begin(SERIAL_BAUD);
  delay(2000);

  Serial.println("\n");
  Serial.println("################################");
  Serial.println(" GPS + IMU MONITORING SYSTEM");
  Serial.println("       ARDUINO UNO Q");
  Serial.println("################################");

  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("\nI2C INITIALIZED");

  // Initialize GPS UART on Hardware Serial1 (Pins 0/1)
  Serial1.begin(GPS_BAUD);
  Serial.println("GPS UART INITIALIZED");

  // Initialize IMU
  imuReady = initializeIMU();

  if (imuReady) {
    calibrateGyroscope();
  }

  Serial.println("\nSYSTEM INITIALIZATION COMPLETE");
  Serial.println("GPS AND IMU MONITOR STARTED");

  lastIMUUpdate = millis();
  lastPrint = millis();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  // Continuously process GPS data
  processGPS();

  // Print both sensors every second and update bridge
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    printIMU();
    printGPS();

    Serial.println("\n################################");

    // Push latest telemetry to Linux bridge
    Bridge.notify("on_telemetry", get_telemetry());
  }
}
