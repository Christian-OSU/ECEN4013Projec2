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

// -------------------- Setup --------------------
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

// -------------------- Loop --------------------
void loop() {
  // Always read GPS characters as they come in
  char c = GPS.read();

  // If a full sentence has arrived, parse it
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return; // wait for valid data
    }
  }

  // -------------------- IMU every 75 ms --------------------
  if (millis() - lastIMU >= 75) {
    lastIMU = millis();

    sensors_event_t event;
    bno.getEvent(&event);

    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Build compact IMU message using printf (faster, no heap)
    char imuBuf[256];
    snprintf(imuBuf, sizeof(imuBuf),
      "Orientation (deg): %.2f, %.2f, %.2f\n"
      "Angular Vel (rad/s): %.2f, %.2f, %.2f\n"
      "Linear Accel (m/s^2): %.2f, %.2f, %.2f\n"
      "Mag Field (uT): %.2f, %.2f, %.2f\n\n",
      -event.orientation.y, event.orientation.z, event.orientation.x,
      gyro.x(), gyro.y(), gyro.z(),
      accel.x(), accel.y(), accel.z(),
      mag.x(), mag.y(), mag.z()
    );

    Serial.print(imuBuf);
    BTSerial.print(imuBuf);
  }

  // -------------------- GPS every 1s --------------------
  if (millis() - lastGPS >= 1000) {
    lastGPS = millis();

    //if (GPS.fix) {
      char gpsBuf[128];
      snprintf(gpsBuf, sizeof(gpsBuf),
        "Time: %02d:%02d:%02d\nFix: %d  \nSatellites: %d\nLatitude: %.6f  \nLongitude: %.6f\n\n",
        GPS.hour, GPS.minute, GPS.seconds,
        (int)GPS.fix, (int)GPS.satellites,
        GPS.latitude, GPS.longitude
      );
      Serial.print(gpsBuf);
      BTSerial.print(gpsBuf);
    //} else {
    //  Serial.println("Waiting for GPS fix...");
    //}
  }
}
