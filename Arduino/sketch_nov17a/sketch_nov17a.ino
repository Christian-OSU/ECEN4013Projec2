#include <Adafruit_GPS.h>

#define GPSSerial Serial1
int ledPin = 2;

// Create GPS object using Serial1
Adafruit_GPS gps(&GPSSerial);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  GPSSerial.begin(9600);
  Serial.println("GPS test starting... ");
  pinMode(ledPin, OUTPUT);

  // Example GPS configuration commands
  gps.sendCommand(PGCMD_ANTENNA);
  
}

void loop() {
  // USB -> GPS passthrough
  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.write(c);
  }

  // GPS -> USB passthrough
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }
  
  digitalWrite(ledPin, HIGH);
}
