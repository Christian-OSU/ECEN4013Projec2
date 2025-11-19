#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <IntervalTimer.h>

// -------------------- Hardware setup --------------------
#define GPSSerial Serial1   // GPS on pins 0=RX1, 1=TX1
#define BTSerial  Serial7   // Bluetooth on pins 7=RX2, 8=TX2

Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

// -------------------- Timing --------------------
unsigned long lastIMU = 0;
unsigned long lastGPS = 0;

// Timer for continuous GPS reads
IntervalTimer gpsTimer;

// Function that runs every 1 ms to keep reading GPS data
void readGPSChar() {
  GPS.read();   // Must be very fast — do NOT add Serial prints here
}

// -------------------- Helper Functions --------------------
void printIMU() {
  sensors_event_t event;
  bno.getEvent(&event);

  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Batch IMU readings into ONE transmission block
  String imuData = "";
  imuData += "Orientation: " + String(-event.orientation.y, 2) + ", " +
              String(event.orientation.z, 2) + ", " +
              String(event.orientation.x, 2) + "; ";
  imuData += "Angular Velocity: " + String(gyro.x(), 2) + ", " +
              String(gyro.y(), 2) + ", " +
              String(gyro.z(), 2) + "; ";
  imuData += "Linear Acceleration: " + String(accel.x(), 2) + ", " +
              String(accel.y(), 2) + ", " +
              String(accel.z(), 2) + "; ";
  imuData += "Magnetic Field: " + String(mag.x(), 2) + ", " +
              String(mag.y(), 2) + ", " +
              String(mag.z(), 2) + "\n";

  Serial.print(imuData);
  BTSerial.print(imuData);
}


void printGPS() {
  String gpsData = "";
  gpsData += "Time: " + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds);
  gpsData += "; Fix: " + String((int)GPS.fix);
  gpsData += "; Satellites: " + String((int)GPS.satellites);

  if (GPS.fix) {
    gpsData += "; Latitude: " + String(GPS.latitude, 6);
    gpsData += "; Longitude: " + String(GPS.longitude, 6);
    gpsData += "; Elevation: " + String(GPS.altitude, 2);
  } else {
    gpsData += "; Latitude: ---; Longitude: ---; Elevation: ---";
  }

  gpsData += "\n";  // end of line

  Serial.print(gpsData);
  BTSerial.print(gpsData);
}



// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);
  GPSSerial.begin(9600);  // Correct baud rate for most GPS modules
  //while (!Serial) delay(10);

  Serial.println("=== Teensy 4.1 IMU + GPS + Bluetooth ===");

  // ---- GPS setup ----
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  // Start interrupt-based GPS reading
  gpsTimer.begin(readGPSChar, 10000); 

  // ---- IMU setup ----
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  delay(1000);
  Serial.println("System initialized.\n");
}

// -------------------- Main Loop --------------------
void loop() {
  // Parse GPS data when new sentence received
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // Read IMU every 75 ms
  if (millis() - lastIMU >= 100) {
    lastIMU = millis();
    printIMU();
  }

  // Print GPS data every 1 s (always)
  if (millis() - lastGPS >= 1000) {
    lastGPS = millis();
    printGPS();
  }
}
