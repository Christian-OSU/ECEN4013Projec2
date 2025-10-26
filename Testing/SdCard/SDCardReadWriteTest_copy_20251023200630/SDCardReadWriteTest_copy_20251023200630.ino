#include <SD.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) ;

  Serial.println("Checking internal SD...");

  if (SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialized successfully.");
  } else {
    Serial.println("SD card initialization failed.");
  }
  // Create and write to a file
  File testFile = SD.open("test.txt", FILE_WRITE);
  if (testFile) {
    Serial.println("Writing to test.txt...");
    testFile.println("Teensy 4.1 SD card write test successful!");
    testFile.close();
    Serial.println("Write complete.");
  } else {
    Serial.println("Failed to open test.txt for writing.");
    return;
  }

  // Read back the file
  testFile = SD.open("test.txt");
  if (testFile) {
    Serial.println("Reading test.txt...");
    while (testFile.available()) {
      Serial.write(testFile.read());
    }
    testFile.close();
    Serial.println("\nRead complete.");
  } else {
    Serial.println("Failed to open test.txt for reading.");
  }
}

void loop() {}

