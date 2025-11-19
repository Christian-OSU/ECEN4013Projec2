#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <IntervalTimer.h>
#include <SD.h>
#include <SPI.h>

// -------------------- Hardware setup --------------------
#define GPSSerial Serial1   // GPS on pins 0=RX1, 1=TX1
#define BTSerial  Serial7   // Bluetooth on pins 7=RX2, 8=TX2
#define SD_CS_PIN BUILTIN_SDCARD

Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
IntervalTimer gpsTimer;
File dataFile;

// -------------------- Timing --------------------
unsigned long lastIMU = 0;
unsigned long lastGPS = 0;

// -------------------- IMU cached data --------------------
sensors_event_t imuEvent;
imu::Vector<3> gyro, accel, mag;

// -------------------- GPS ISR --------------------
void readGPSChar() {
  GPS.read();
}

// -------------------- CSV Logging --------------------
// -------------------- CSV Logging --------------------
// -------------------- CSV Helper --------------------
void saveCSV() {
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

    dataFile.print((int)GPS.fix); dataFile.print(",");
    dataFile.print((int)GPS.satellites); dataFile.print(",");

    if (GPS.fix) {
      dataFile.print(GPS.latitude, 6); dataFile.print(",");
      dataFile.print(GPS.longitude, 6); dataFile.print(",");
      dataFile.print(GPS.altitude, 2); // Elevation
    } else {
      dataFile.print(", ,");
    }
    dataFile.print(",");

    dataFile.print(-imuEvent.orientation.y); dataFile.print(",");
    dataFile.print(imuEvent.orientation.z); dataFile.print(",");
    dataFile.print(imuEvent.orientation.x); dataFile.print(",");

    dataFile.print(gyro.x()); dataFile.print(",");
    dataFile.print(gyro.y()); dataFile.print(",");
    dataFile.print(gyro.z()); dataFile.print(",");

    dataFile.print(accel.x()); dataFile.print(",");
    dataFile.print(accel.y()); dataFile.print(",");
    dataFile.print(accel.z()); dataFile.print(",");

    dataFile.print(mag.x()); dataFile.print(",");
    dataFile.print(mag.y()); dataFile.print(",");
    dataFile.println(mag.z());

    dataFile.close();
  }
}



// -------------------- IMU Read & Print --------------------
void readIMU() {
  bno.getEvent(&imuEvent);
  gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}

void printIMU() {
  String imuData = "";
  imuData += "Orientation: " + String(-imuEvent.orientation.y, 2) + ", "
                        + String(imuEvent.orientation.z, 2) + ", "
                        + String(imuEvent.orientation.x, 2) + "; ";

  imuData += "Angular Velocity: " + String(gyro.x(), 2) + ", "
                             + String(gyro.y(), 2) + ", "
                             + String(gyro.z(), 2) + "; ";

  imuData += "Linear Acceleration: " + String(accel.x(), 2) + ", "
                                  + String(accel.y(), 2) + ", "
                                  + String(accel.z(), 2) + "; ";

  imuData += "Magnetic Field: " + String(mag.x(), 2) + ", "
                              + String(mag.y(), 2) + ", "
                              + String(mag.z(), 2);

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

  Serial.println("=== Teensy 4.1 IMU + GPS + Bluetooth + CSV ===");

  // ---- SD Card ----
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
  } else {
    if (SD.exists("log.csv")) SD.remove("log.csv");
    dataFile = SD.open("log.csv", FILE_WRITE);
    if (dataFile) {
              dataFile.println("Date,Time,Fix,Satellites,Latitude,Longitude,Elevation,"
                 "OriX,OriY,OriZ,"
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
  Wire1.setClock(100000); // lower I2C speed for stability
  if (!bno.begin()) {
    Serial.println("BNO055 init failed");
    while (1);
  }
  bno.setExtCrystalUse(true);
  Serial.println("IMU initialized");
}

// -------------------- Main Loop --------------------
void loop() {
  // Parse GPS
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

  // IMU every 100 ms
  if (millis() - lastIMU >= 100) {
    lastIMU = millis();
    readIMU();
    printIMU();
    saveCSV();
  }

  // GPS print every 1 s
  if (millis() - lastGPS >= 1000) {
    lastGPS = millis();
    printGPS();
  }
}
