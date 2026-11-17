#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

// =====================================================
// DEVICE CONFIGURATION (USER'S WORKING BASE)
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

#define GPS_BAUD        9600
#define SERIAL_BAUD     115200

TinyGPSPlus gps;

// =====================================================
// IMU & SENSOR VARIABLES
// =====================================================

bool imuReady = false;
bool fallAlert = false;
unsigned long gpsRawBytes = 0;

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
unsigned long lastBaudTry = 0;

const uint32_t GPS_BAUD_LIST[] = {9600, 38400, 115200};
uint8_t gpsBaudIdx = 0;

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

  if (Wire.endTransmission(false) != 0) {
    return false;
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
// IMU INITIALIZATION
// =====================================================

bool initializeIMU() {
  byte id = 0;
  Serial.println("\nINITIALIZING IMU...");

  // Try default address 0x68 first, then alternate address 0x69
  imuAddr = 0x68;
  if (!readRegisters(REG_WHO_AM_I, &id, 1)) {
    imuAddr = 0x69;
    if (!readRegisters(REG_WHO_AM_I, &id, 1)) {
      imuAddr = 0x68;
      Serial.println("IMU: WHO_AM_I READ FAILED (Check SDA/SCL pins)");
      return false;
    }
  }

  Serial.print("IMU FOUND AT 0x");
  Serial.print(imuAddr, HEX);
  Serial.print(" - WHO_AM_I: 0x");
  Serial.println(id, HEX);

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
  Serial.println("KEEP THE SENSOR STILL...");
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
    delay(2);
  }

  if (successfulSamples > 50) {
    gyroOffsetX = sumX / successfulSamples;
    gyroOffsetY = sumY / successfulSamples;
    gyroOffsetZ = sumZ / successfulSamples;
    Serial.println("CALIBRATION COMPLETE");
    Serial.print("OFFSETS: X="); Serial.print(gyroOffsetX);
    Serial.print(" Y="); Serial.print(gyroOffsetY);
    Serial.print(" Z="); Serial.println(gyroOffsetZ);
  } else {
    Serial.println("CALIBRATION SKIPPED (Zero defaults)");
    gyroOffsetX = 0;
    gyroOffsetY = 0;
    gyroOffsetZ = 0;
  }
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

  // Gyroscope in deg/s
  gx = (rawGX - gyroOffsetX) / GYRO_SCALE;
  gy = (rawGY - gyroOffsetY) / GYRO_SCALE;
  gz = (rawGZ - gyroOffsetZ) / GYRO_SCALE;

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
// GPS PROCESSING & AUTO-BAUD CHECK
// =====================================================

void processGPS() {
  while (Serial1.available() > 0) {
    char incoming = Serial1.read();
    gpsRawBytes++;
    gps.encode(incoming);
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void checkGpsBaud() {
  // If 0 bytes received after 5 seconds, cycle baud rate automatically
  if (gpsRawBytes == 0 && (millis() - lastBaudTry > 5000)) {
    lastBaudTry = millis();
    gpsBaudIdx = (gpsBaudIdx + 1) % 3;
    Serial1.end();
    delay(50);
    Serial1.begin(GPS_BAUD_LIST[gpsBaudIdx]);
    Serial.print("TESTING GPS BAUD: ");
    Serial.println(GPS_BAUD_LIST[gpsBaudIdx]);
  }
}

// =====================================================
// TELEMETRY PAYLOAD FOR LINUX / WEB
// Note: Read-only format to prevent I2C bus race conditions
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
  tel += String(gpsRawBytes);
  tel += ",";
  tel += (imuReady ? "MPU6050_OK" : "NO_I2C_ACK");

  return tel;
}

// =====================================================
// PRINT DEBUG FUNCTIONS (SERIAL MONITOR)
// =====================================================

void printIMU() {
  Serial.println("\n========== IMU DATA ==========");
  if (!imuReady) {
    Serial.println("IMU STATUS: NOT INITIALIZED (Check SDA/SCL)");
    return;
  }
  Serial.println("IMU STATUS: WORKING");
  Serial.print("ACCEL (g): X="); Serial.print(ax, 3);
  Serial.print(" Y="); Serial.print(ay, 3);
  Serial.print(" Z="); Serial.println(az, 3);

  Serial.print("ORIENTATION: ROLL="); Serial.print(roll, 2);
  Serial.print(" deg, PITCH="); Serial.print(pitch, 2); Serial.println(" deg");
  Serial.println("==============================");
}

void printGPS() {
  Serial.println("========== NEO-6M GPS ==========");
  Serial.print("GPS RAW BYTES: "); Serial.println(gpsRawBytes);
  Serial.print("PARSED CHARS: "); Serial.println(gps.charsProcessed());
  Serial.print("SATELLITES: ");
  if (gps.satellites.isValid()) {
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("SEARCHING...");
  }
  if (gps.location.isValid()) {
    Serial.print("LAT: "); Serial.println(gps.location.lat(), 6);
    Serial.print("LNG: "); Serial.println(gps.location.lng(), 6);
  }
  Serial.println("================================");
}

// =====================================================
// REWRITTEN SETUP (RELIABLE HARDWARE INITIALIZATION)
// =====================================================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);

  // 1. Hardware Power Stabilization Delay (Crucial for MPU-6050 & GPS startup)
  delay(2000);

  Serial.println("\n");
  Serial.println("################################");
  Serial.println(" GPS + IMU MONITORING SYSTEM");
  Serial.println("       ARDUINO UNO Q");
  Serial.println("################################");

  // 2. Initialize I2C Bus
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("I2C INITIALIZED");

  // 3. Initialize GPS UART on Hardware Serial1 (Pins 0/1)
  Serial1.begin(GPS_BAUD);
  Serial.println("GPS UART INITIALIZED ON SERIAL1 (9600 BAUD)");

  // 4. Initialize IMU
  imuReady = initializeIMU();
  if (imuReady) {
    calibrateGyroscope();
  }

  // 5. Initialize Bridge to Linux MPU AFTER sensors are initialized!
  Bridge.begin();
  Bridge.provide("get_telemetry", get_telemetry);
  Serial.println("BRIDGE RPC INITIALIZED");

  Serial.println("\nSYSTEM INITIALIZATION COMPLETE");
  Serial.println("GPS AND IMU MONITOR STARTED");

  lastIMUUpdate = millis();
  lastPrint = millis();
  lastIMURetry = millis();
  lastBaudTry = millis();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  // Continuously ingest GPS data
  processGPS();
  checkGpsBaud();

  // Retry IMU connection every 2 seconds if not connected
  if (!imuReady && (millis() - lastIMURetry > 2000)) {
    lastIMURetry = millis();
    imuReady = initializeIMU();
    if (imuReady) {
      calibrateGyroscope();
    }
  }

  // High-frequency IMU orientation update (50Hz / 20ms)
  if (imuReady && (millis() - lastIMUUpdate >= 20)) {
    unsigned long now = millis();
    float dt = (now - lastIMUUpdate) / 1000.0;
    lastIMUUpdate = now;

    if (readIMU() && dt > 0.0 && dt < 0.2) {
      calculateOrientation(dt);
    }
  }

  // Periodic Telemetry sync to Bridge and Serial (every 1 second)
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    printIMU();
    printGPS();
    Bridge.notify("on_telemetry", get_telemetry());
  }
}
