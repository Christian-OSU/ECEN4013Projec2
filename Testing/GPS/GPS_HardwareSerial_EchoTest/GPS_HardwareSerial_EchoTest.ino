// Test code for Ultimate GPS Using Hardware Serial
// (e.g. GPS for Leonardo, Flora or FeatherWing)
//
// This code shows how to test a passthru between USB and hardware serial
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada
#include <Adafruit_GPS.h>

Adafruit_GPS gps(&GPSSerial);


// what's the name of the hardware serial port?
#define GPSSerial Serial1
int ledPin=2;

void setup() {
  // make this baud rate fast enough to we aren't waiting on it
  Serial.begin(115200);

  // wait for hardware serial to appear
  while (!Serial) delay(10);

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);
  Serial.println("GPS test starting... ");
  pinMode(ledPin, OUTPUT);
}


void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.print(c + "LET");
    
    //Serial.print("ETTST");
  }
  //Serial.print("BETWEEN");
  //Serial.println();
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.print(c);
  }
  gps.sendCommand(PGCMD_ANTENNA)
  digitalWrite(ledPin,HIGH);

}
