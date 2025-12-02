#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_GPS.h>
#include <IntervalTimer.h>
#include <SD.h>
#include <SPI.h>

// -------------------- Hardware setup --------------------
#define GPSSerial Serial1   // GPS on pins 0=RX1, 1=TX1
#define BTSerial  Serial7   // Bluetooth on pins 7=RX2, 8=TX2
#define SD_CS_PIN BUILTIN_SDCARD

Adafruit_GPS GPS(&GPSSerial);
IntervalTimer gpsTimer;
File dataFile;

// IMU objects
Adafruit_LSM6DS3TRC lsm6ds3;   // accel + gyro
Adafruit_LIS3MDL lis3mdl;      // magnetometer

// -------------------- Timing --------------------
unsigned long lastIMU = 0;
unsigned long lastGPS = 0;
const unsigned long IMU_PERIOD_MS = 100; // 100 ms => 10 Hz

// -------------------- IMU cached data --------------------
sensors_event_t accelEvent, gyroEvent, magEvent, tempEvent;

// -------------------- Madgwick filter state --------------------
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion
float beta = 0.1f; // Madgwick gain (tune if needed)

// -------------------- GPS ISR --------------------
// ----------------------------------------------------
// GPS interrupt: Feed ONE character at a time to parser
// ----------------------------------------------------
void readGPSChar() {
  GPS.read();   // <<< FIXED: DO NOT read GPSSerial directly!
}


//--------------------Uptime-----------------------
unsigned long uptimeSeconds = 0;

void formatUptime(char *outStr) {
  unsigned long s = uptimeSeconds;
  unsigned long hours = s / 3600;
  unsigned long minutes = (s % 3600) / 60;
  unsigned long seconds = s % 60;

  sprintf(outStr, "%02lu:%02lu:%02lu", hours, minutes, seconds);
}


// -------------------- Madgwick update (simplified) --------------------
// gyro: radians/sec, accel: m/s^2, mag: microtesla (µT)
void MadgwickAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float dt) {
  // Avoid division by zero
  if (dt <= 0.0f) return;

  // Normalize accelerometer
  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm == 0.0f) return;
  ax /= norm; ay /= norm; az /= norm;

  // Normalize magnetometer
  norm = sqrtf(mx*mx + my*my + mz*mz);
  if (norm == 0.0f) return;
  mx /= norm; my /= norm; mz /= norm;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _4q0 = 4.0f * q0;
  float _4q1 = 4.0f * q1;
  float _4q2 = 4.0f * q2;
  float _8q1 = 8.0f * q1;
  float _8q2 = 8.0f * q2;
  float q0q0 = q0 * q0;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;

  // Reference direction of Earth's magnetic field
  float hx = mx * (q0q0 + q1q1 - q2q2 - q3q3)
           + my * (2.0f * (q1*q2 - q0*q3))
           + mz * (2.0f * (q1*q3 + q0*q2));
  float hy = mx * (2.0f * (q1*q2 + q0*q3))
           + my * (q0q0 - q1q1 + q2q2 - q3q3)
           + mz * (2.0f * (q2*q3 - q0*q1));
  float _2bx = sqrtf(hx * hx + hy * hy);
  float _2bz = mx * (2.0f * (q1*q3 - q0*q2))
             + my * (2.0f * (q2*q3 + q0*q1))
             + mz * (q0q0 - q1q1 - q2q2 + q3q3);

  // Gradient descent corrective step (approximated from Madgwick)
  float s0, s1, s2, s3;

  float _4bx = 2.0f * _2bx;
  float _4bz = 2.0f * _2bz;

  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q3 = q2 * q3;

  // Compute the objective function and Jacobian (omitted detailed derivation for brevity)
  // Using the standard Madgwick gradient descent step implementation
  s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
       + (-_4bz * q2 + _4bx * q3) * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
       + (-_4bx * q2 - _4bz * q3) * (mx * (q1q2 - q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1))
       + (_4bx * q1) * (mx * (q1q3 + q0q2) + my * (q2q3 - q0q1) + mz * (0.5f - q1q1 - q2q2));
  s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
       + (_4bz * q3 + _4bx * q2) * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
       + (_4bx * q1 + _4bz * q0) * (mx * (q1q2 - q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1))
       + (_4bx * q0 - _8q1 * _2bz) * (mx * (q1q3 + q0q2) + my * (q2q3 - q0q1) + mz * (0.5f - q1q1 - q2q2));
  s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
       + (-_8q2 * _2bx - _4bz * q0 + _4bx * q1) * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
       + (_4bx * q2 + _4bz * q1) * (mx * (q1q2 - q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1))
       + (_4bx * q0 - _8q2 * _2bz) * (mx * (q1q3 + q0q2) + my * (q2q3 - q0q1) + mz * (0.5f - q1q1 - q2q2));
  s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q0q0 * q3 - _2q0 * ay
       + (-_4bz * q1 + _4bx * q0) * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
       + (-_4bx * q3 + _4bz * q2) * (mx * (q1q2 - q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1))
       + (_4bx * q1) * (mx * (q1q3 + q0q2) + my * (q2q3 - q0q1) + mz * (0.5f - q1q1 - q2q2));

  // Normalize step magnitude
  norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
  if (norm == 0.0f) return;
  s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;

  // Rate of change of quaternion from gyroscope
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
  float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
  float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
  float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

  // Integrate to yield quaternion
  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  // Normalize quaternion
  norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (norm == 0.0f) return;
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

// Convert quaternion to Euler angles (radians returned; caller can convert if needed)
void quaternionToEulerDeg(float &yawDeg, float &pitchDeg, float &rollDeg) {
  // yaw (z-axis rotation)
  float yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
  // pitch (y-axis rotation)
  float sinp = 2.0f*(q0*q2 - q3*q1);
  float pitch;
  if (fabsf(sinp) >= 1.0f)
    pitch = copysignf(M_PI/2.0f, sinp);
  else
    pitch = asinf(sinp);
  // roll (x-axis rotation)
  float roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));

  // convert to degrees
  yawDeg   = yaw   * 180.0f / M_PI;
  pitchDeg = pitch * 180.0f / M_PI;
  rollDeg  = roll  * 180.0f / M_PI;
}

// -------------------- CSV Logging --------------------
void saveCSV(float yaw, float pitch, float roll) {
  dataFile = SD.open("log.csv", FILE_WRITE);
  if (dataFile) {
    // Date string
    char dateStr[11];
    sprintf(dateStr, "%02d/%02d/%04d", GPS.day, GPS.month, GPS.year + 2000);
    dataFile.print(dateStr); dataFile.print(",");

    // Time string
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", GPS.hour, GPS.minute, GPS.seconds);
    dataFile.print(timeStr); dataFile.print(",");

    char uptimeStr[12];
    formatUptime(uptimeStr);
    dataFile.print(uptimeStr); dataFile.print(",");

    dataFile.print((int)GPS.fix); dataFile.print(",");
    dataFile.print((int)GPS.satellites); dataFile.print(",");

    if (GPS.fix) {
      dataFile.print(GPS.latitude, 6); dataFile.print(",");
      dataFile.print(GPS.longitude, 6); dataFile.print(",");
      dataFile.print(GPS.altitude, 2);
    } else {
      dataFile.print(", ,");
    }
    dataFile.print(",");

    // Orientation (Yaw,Pitch,Roll) in degrees — keep CSV as Yaw,Pitch,Roll
    dataFile.print(yaw, 2); dataFile.print(",");
    dataFile.print(pitch, 2); dataFile.print(",");
    dataFile.print(roll, 2); dataFile.print(",");

    // Gyro (rad/s)
    dataFile.print(gyroEvent.gyro.x); dataFile.print(",");
    dataFile.print(gyroEvent.gyro.y); dataFile.print(",");
    dataFile.print(gyroEvent.gyro.z); dataFile.print(",");

    // Accel (m/s^2)
    dataFile.print(accelEvent.acceleration.x); dataFile.print(",");
    dataFile.print(accelEvent.acceleration.y); dataFile.print(",");
    dataFile.print(accelEvent.acceleration.z); dataFile.print(",");

    // Mag (uT)
    dataFile.print(magEvent.magnetic.x); dataFile.print(",");
    dataFile.print(magEvent.magnetic.y); dataFile.print(",");
    dataFile.println(magEvent.magnetic.z);

    dataFile.close();
  }
}

// -------------------- IMU Read & Print --------------------
void readIMU() {
  // LSM6DS3 accel + gyro
  lsm6ds3.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  // LIS3MDL mag
  lis3mdl.getEvent(&magEvent);
}

void printIMU_lineFormat(float pitch, float roll, float yaw) {
  // Matches format your Python GUI expects:
  // Orientation: <pitch>, <roll>, <yaw>; Angular Velocity: <gx>, <gy>, <gz>; Linear Acceleration: <ax>, <ay>, <az>; Magnetic Field: <mx>, <my>, <mz>

  // Use 2 decimals for orientation, 3 for sensors (you can change formatting as needed)
  String imuData = "";

  imuData += "Orientation: " + String(roll, 2) + ", " + String(yaw, 2) + ", " + String(pitch, 2) + "; ";
  imuData += "Angular Velocity: " + String(gyroEvent.gyro.x, 3) + ", " + String(gyroEvent.gyro.y, 3) + ", " + String(gyroEvent.gyro.z, 3) + "; ";
  imuData += "Linear Acceleration: " + String(accelEvent.acceleration.x, 3) + ", " + String(accelEvent.acceleration.y, 3) + ", " + String(accelEvent.acceleration.z, 3) + "; ";
  imuData += "Magnetic Field: " + String(magEvent.magnetic.x, 3) + ", " + String(magEvent.magnetic.y, 3) + ", " + String(magEvent.magnetic.z, 3);

  // Print identical line to both Serial and BTSerial
  Serial.println(imuData);
  BTSerial.println(imuData);
}

// -------------------- GPS Print --------------------
void printGPS() {
  String gpsData = "";
  gpsData += "Time: " + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "; ";
  gpsData += "Fix: " + String((int)GPS.fix) + "; ";
  gpsData += "Satellites: " + String((int)GPS.satellites) + "; ";

  if (GPS.fix) {
    gpsData += "Latitude: " + String(GPS.latitude, 6) + "; ";
    gpsData += "Longitude: " + String(GPS.longitude, 6) + "; ";
    gpsData += "Elevation: " + String(GPS.altitude, 2);
  } else {
    gpsData += "Latitude: ---; Longitude: ---; Elevation: ---";
  }

  Serial.println(gpsData);
  BTSerial.println(gpsData);
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);
  GPSSerial.begin(9600);
  //while (!Serial) delay(10);

  Serial.println("=== Teensy 4.1 IMU + GPS + Bluetooth + CSV (LSM6DS3TRC + LIS3MDL) ===");

  // ---- SD Card ----
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
  } else {
    if (SD.exists("log.csv")) SD.remove("log.csv");
    dataFile = SD.open("log.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Date,Time,UpTime,Fix,Satellites,Latitude,Longitude,Elevation,"
                       "Yaw,Pitch,Roll,"
                       "GyroX,GyroY,GyroZ,"
                       "AccelX,AccelY,AccelZ,"
                       "MagX,MagY,MagZ");
      dataFile.close();
    }
    Serial.println("SD initialized and CSV header created");
  }

  // ---- GPS ----
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  gpsTimer.begin(readGPSChar, 10000); // 1ms ISR
  Serial.println("GPS initialized");

  // ---- IMU ----
  Wire1.begin();                     // initialize Wire1 explicitly
  Wire1.setClock(100000);            // lower I2C speed for stability (optional)

  if (!lsm6ds3.begin_I2C(0x6A, &Wire1)) {
    Serial.println("Failed to find LSM6DS3 on Wire1!");
    while (1);
  }
  Serial.println("LSM6DS3 initialized");

  if (!lis3mdl.begin_I2C(0x1C, &Wire1)) {
    Serial.println("Failed to find LIS3MDL on Wire1!");
    while (1);
  }
  Serial.println("LIS3MDL initialized");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  Serial.println("IMU initialized (LSM6DS3 + LIS3MDL)");
}

// -------------------- Main Loop --------------------
void loop() {
  // Parse GPS
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

  // IMU every IMU_PERIOD_MS
  if (millis() - lastIMU >= IMU_PERIOD_MS) {
    unsigned long now = millis();
    float dt = (now - lastIMU) / 1000.0f;
    if (dt <= 0.0f) dt = IMU_PERIOD_MS / 1000.0f; // fallback
    lastIMU = now;

    readIMU();

    // Extract sensor values (units from Adafruit libs):
    // gyroEvent.gyro.*  -> radians/sec
    // accelEvent.acceleration.* -> m/s^2
    // magEvent.magnetic.* -> microtesla (uT)

    float gx = gyroEvent.gyro.x;
    float gy = gyroEvent.gyro.y;
    float gz = gyroEvent.gyro.z;

    float ax = accelEvent.acceleration.x;
    float ay = accelEvent.acceleration.y;
    float az = accelEvent.acceleration.z;

    float mx = magEvent.magnetic.x;
    float my = magEvent.magnetic.y;
    float mz = magEvent.magnetic.z;

    // Run Madgwick fusion
    MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    // Compute Euler angles (degrees)
    float yawDeg, pitchDeg, rollDeg;
    quaternionToEulerDeg(yawDeg, pitchDeg, rollDeg);

    // Note: your Python parser expects Orientation: pitch, roll, yaw
    float pitch_to_print = pitchDeg;
    float roll_to_print  = rollDeg;
    float yaw_to_print   = yawDeg;

    // Print IMU line in exact format your GUI expects
    printIMU_lineFormat(pitch_to_print, roll_to_print, yaw_to_print);

    // Save CSV (CSV keeps order Yaw,Pitch,Roll)
    saveCSV(yawDeg, pitchDeg, rollDeg);
  }

  // GPS print every 1 s
  if (millis() - lastGPS >= 1000) {
    lastGPS = millis();
    printGPS();
    uptimeSeconds++;
    char uptimeStr[12];
    formatUptime(uptimeStr);

    Serial.print("Uptime: ");
    Serial.println(uptimeStr);

    BTSerial.print("Uptime: ");
    BTSerial.println(uptimeStr);
  }
}
