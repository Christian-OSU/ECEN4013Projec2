#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

// -------------------- Hardware setup --------------------
#define GPSSerial Serial1   // GPS on pins 0=RX1, 1=TX1
#define BTSerial  Serial7   // Bluetooth on pins 7=RX2, 8=TX2

Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

// -------------------- Timing --------------------
unsigned long lastIMU = 0;
unsigned long lastGPS = 0;

void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);
  GPSSerial.begin(9600);
  while (!Serial) delay(10);

  Serial.println("=== Teensy 4.1 IMU + GPS + Bluetooth ===");

  // ---- GPS setup ----
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Location & fix data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  // ---- IMU setup ----
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  delay(1000);
  Serial.println("System initialized.\n");
}

void loop() {
  // Always read GPS data
  GPS.read();

  // -------------------- IMU every 75 ms --------------------
  if (millis() - lastIMU >= 75) {
    lastIMU = millis();

    sensors_event_t event;
    bno.getEvent(&event);

    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Build IMU data
    String imuData = "";
    imuData += "Orientation: ";
    imuData += String(-event.orientation.y, 2) + ", ";
    imuData += String(event.orientation.z, 2) + ", ";
    imuData += String(event.orientation.x, 2) + "\n";

    imuData += "Angular Velocity: ";
    imuData += String(gyro.x(), 2) + ", ";
    imuData += String(gyro.y(), 2) + ", ";
    imuData += String(gyro.z(), 2) + "\n";

    imuData += "Linear Acceleration: ";
    imuData += String(accel.x(), 2) + ", ";
    imuData += String(accel.y(), 2) + ", ";
    imuData += String(accel.z(), 2) + "\n";

    imuData += "Magnetic Field: ";
    imuData += String(mag.x(), 2) + ", ";
    imuData += String(mag.y(), 2) + ", ";
    imuData += String(mag.z(), 2) + "\n";

    imuData += "\n";

    // Send IMU data to Serial + Bluetooth
    Serial.print(imuData);
    BTSerial.print(imuData);
  }

  // -------------------- GPS every 1s --------------------
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      if (millis() - lastGPS >= 1000) {
        lastGPS = millis();

        String gpsData = "";
        gpsData += "Time: ";
        gpsData += String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "\n";
        gpsData += "Fix: " + String((int)GPS.fix) + "\n";
        gpsData += "Satellites: " + String((int)GPS.satellites) + "\n";
        gpsData += "Latitude: " + String(GPS.latitude, 6) + "\n";
        gpsData += "Longitude: " + String(GPS.longitude, 6) + "\n";
        gpsData += "\n";

        Serial.print(gpsData);
        BTSerial.print(gpsData);
      }
    }
  }
}
