#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  GPSSerial.begin(9600);
  delay(1000);

  bno.setExtCrystalUse(true);
}


void loop(void){
//IMU______________________________________________________________________
  // Orientation (Euler angles)
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("Orientation (Yaw, Pitch, Roll): ");
  Serial.print(-event.orientation.y); Serial.print(", ");
  Serial.print(event.orientation.z); Serial.print(", ");
  Serial.println(event.orientation.x);

  // Angular velocity (rad/s)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print("Angular Velocity (X, Y, Z): ");
  Serial.print(gyro.x()); Serial.print(", ");
  Serial.print(gyro.y()); Serial.print(", ");
  Serial.println(gyro.z());

  // Linear acceleration (m/s^2)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Serial.print("Linear Acceleration (X, Y, Z): ");
  Serial.print(accel.x()); Serial.print(", ");
  Serial.print(accel.y()); Serial.print(", ");
  Serial.println(accel.z());

  // Magnetic field (µT)
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print("Magnetic Field (X, Y, Z): ");
  Serial.print(mag.x()); Serial.print(", ");
  Serial.print(mag.y()); Serial.print(", ");
  Serial.println(mag.z());

  delay(100); // Slight delay for readability
//GPS ___________________________________________________________________________________
   
   GPS.read();
if (GPS.newNMEAreceived()) {
  if (GPS.parse(GPS.lastNMEA())) {
    // Only runs when a full sentence is parsed
    Serial.print("Time: ");
    Serial.print(GPS.hour); Serial.print(":");
    Serial.print(GPS.minute); Serial.print(":");
    Serial.println(GPS.seconds);

    Serial.print("Fix: "); Serial.println((int)GPS.fix);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    Serial.print("Latitude: "); Serial.println(GPS.latitude, 6);
    Serial.print("Longitude: "); Serial.println(GPS.longitude, 6);
  }
}


   
    /*while (Serial.available()) {
    char c = Serial.read();
    GPSSerial.write(c);
  }
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }
    Serial.println(); // Blank line between samples*/

}
