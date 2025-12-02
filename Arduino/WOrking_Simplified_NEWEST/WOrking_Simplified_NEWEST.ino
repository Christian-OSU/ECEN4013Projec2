#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <SD.h>
#include <SPI.h>

// ============================================================
// HARDWARE CONFIGURATION
// ============================================================
#define GPS_SERIAL Serial1           // GPS module (RX1/TX1)
#define USB_OUT Serial            // USB output (115200 baud)
#define BT_SERIAL Serial7            // HC-05 Bluetooth module (9600 baud)
#define SD_CS_PIN BUILTIN_SDCARD     // Teensy 4.1 built-in SD

// Output control flags
#define ENABLE_USB_OUTPUT true       // Set to false to disable USB output
#define ENABLE_BT_OUTPUT true         // Set to false to disable Bluetooth output
#define ENABLE_SD_OUTPUT true         // Set to false to disable SD card output

// IMU sensors
Adafruit_LSM6DS3TRC lsm6ds3;  // Accel + Gyro
Adafruit_LIS3MDL lis3mdl;     // Magnetometer

// SD Card file handle
File dataFile;

// System status flags
bool sdReady = false;
bool imuReady = false;

// ============================================================
// DATA STRUCTURES
// ============================================================
struct GPSData {
  float latitude = 0.0;      // Decimal degrees
  float longitude = 0.0;     // Decimal degrees
  float altitude = 0.0;      // Meters above MSL (Mean Sea Level)
  uint8_t satellites = 0;    // Number of locked satellites
  bool valid = false;        // Data validity flag
};

struct IMUData {
  // Angular velocity (rad/s)
  float gyro_x = 0.0;
  float gyro_y = 0.0;
  float gyro_z = 0.0;
  
  // Acceleration (m/s²)
  float accel_x = 0.0;
  float accel_y = 0.0;
  float accel_z = 0.0;
  
  // Magnetic field (µT)
  float mag_x = 0.0;
  float mag_y = 0.0;
  float mag_z = 0.0;
};

GPSData gps;
IMUData imu;

// ============================================================
// GPS PARSING (NMEA)
// ============================================================
char nmeaBuffer[100];
uint8_t nmeaIndex = 0;

// Convert NMEA format (DDMM.MMMM) to decimal degrees
float nmeaToDecimal(float raw, char hemisphere) {
  int degrees = (int)(raw / 100);
  float minutes = raw - (degrees * 100);
  float decimal = degrees + (minutes / 60.0);
  
  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal = -decimal;
  }
  return decimal;
}

void parseGGA(String fields[], int count) {
  // $GPGGA - Global Positioning System Fix Data
  if (count < 10) return;
  
  // Fix quality (0 = invalid, 1+ = valid)
  int fixQuality = fields[6].toInt();
  gps.valid = (fixQuality > 0);
  
  if (gps.valid) {
    // Latitude
    gps.latitude = nmeaToDecimal(fields[2].toFloat(), fields[3][0]);
    
    // Longitude
    gps.longitude = nmeaToDecimal(fields[4].toFloat(), fields[5][0]);
    
    // Number of satellites
    gps.satellites = fields[7].toInt();
    
    // Altitude above MSL (Mean Sea Level) in meters
    gps.altitude = fields[9].toFloat();
  }
}

void parseNMEA(String sentence) {
  if (!sentence.startsWith("$")) return;
  
  // Remove checksum
  int checksumPos = sentence.indexOf('*');
  if (checksumPos > 0) {
    sentence = sentence.substring(0, checksumPos);
  }
  
  // Split into fields
  String fields[20];
  int fieldCount = 0;
  int startPos = 0;
  
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence[i] == ',') {
      fields[fieldCount++] = sentence.substring(startPos, i);
      startPos = i + 1;
    }
  }
  fields[fieldCount++] = sentence.substring(startPos);
  
  // Parse GGA sentences (contains all data we need)
  if (fields[0].endsWith("GGA")) {
    parseGGA(fields, fieldCount);
  }
}

// ============================================================
// IMU READING
// ============================================================
void readIMU() {
  sensors_event_t accel, gyro, mag, temp;
  
  // Read sensors
  lsm6ds3.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);
  
  // Store gyro data (already in rad/s)
  imu.gyro_x = gyro.gyro.x;
  imu.gyro_y = gyro.gyro.y;
  imu.gyro_z = gyro.gyro.z;
  
  // Store acceleration data (already in m/s²)
  imu.accel_x = accel.acceleration.x;
  imu.accel_y = accel.acceleration.y;
  imu.accel_z = accel.acceleration.z;
  
  // Store magnetic field (already in µT)
  imu.mag_x = mag.magnetic.x;
  imu.mag_y = mag.magnetic.y;
  imu.mag_z = mag.magnetic.z;
}

// ============================================================
// DATA OUTPUT
// ============================================================
String buildCSVLine() {
  String line = "";
  
  // GPS Data
  line += String(gps.latitude, 6) + ",";
  line += String(gps.longitude, 6) + ",";
  line += String(gps.altitude, 2) + ",";
  line += String(gps.satellites) + ",";
  
  // IMU Data
  line += String(imu.gyro_x, 4) + ",";
  line += String(imu.gyro_y, 4) + ",";
  line += String(imu.gyro_z, 4) + ",";
  line += String(imu.accel_x, 4) + ",";
  line += String(imu.accel_y, 4) + ",";
  line += String(imu.accel_z, 4) + ",";
  line += String(imu.mag_x, 4) + ",";
  line += String(imu.mag_y, 4) + ",";
  line += String(imu.mag_z, 4);
  
  return line;
}

void printHumanReadable(Stream &output) {
  output.println("--- GPS DATA ---");
  output.print("  Latitude:    "); output.print(gps.latitude, 6); output.println(" deg");
  output.print("  Longitude:   "); output.print(gps.longitude, 6); output.println(" deg");
  output.print("  Altitude:    "); output.print(gps.altitude, 2); output.println(" m (MSL)");
  output.print("  Satellites:  "); output.println(gps.satellites);
  
  output.println("--- IMU DATA ---");
  output.print("  Gyro (rad/s):  X="); output.print(imu.gyro_x, 4);
  output.print(" Y="); output.print(imu.gyro_y, 4);
  output.print(" Z="); output.println(imu.gyro_z, 4);
  
  output.print("  Accel (m/s²):  X="); output.print(imu.accel_x, 4);
  output.print(" Y="); output.print(imu.accel_y, 4);
  output.print(" Z="); output.println(imu.accel_z, 4);
  
  output.print("  Mag (µT):      X="); output.print(imu.mag_x, 4);
  output.print(" Y="); output.print(imu.mag_y, 4);
  output.print(" Z="); output.println(imu.mag_z, 4);
  output.println();
}

void outputData() {
  String csvLine = buildCSVLine();
  
  // 1. SD Card - write and flush (keep file open)
  if (ENABLE_SD_OUTPUT && sdReady && dataFile) {
    dataFile.println(csvLine);
    dataFile.flush();  // Ensure data is written
  }
  
  // 2. USB Serial - human readable format (only if connected)
  if (ENABLE_USB_OUTPUT && USB_OUT) {
    printHumanReadable(USB_OUT);
  }
  
  // 3. Bluetooth (HC-05) - human readable format
  if (ENABLE_BT_OUTPUT) {
    BT_SERIAL.println(csvLine);
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  // Initialize LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize serial ports
  USB_OUT.begin(115200);
  GPS_SERIAL.begin(9600);
  BT_SERIAL.begin(9600);  // HC-05 Bluetooth module
  
  // Small delay for serial ports to initialize
  delay(500);
  USB_OUT.println("\n\n=== GPS + IMU Data Logger ===");
  USB_OUT.println("Elevation Model: MSL (Mean Sea Level)");
  USB_OUT.println();
  USB_OUT.println();
  
  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    USB_OUT.println("ERROR: SD card initialization failed!");
  } else {
    USB_OUT.println("SD card initialized");
    
    // Create new file with header
    if (SD.exists("data.csv")) {
      SD.remove("data.csv");
    }
    
    dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Lat,Lon,Alt_MSL,Satellites,GyroX_rad/s,GyroY_rad/s,GyroZ_rad/s,AccelX_m/s2,AccelY_m/s2,AccelZ_m/s2,MagX_uT,MagY_uT,MagZ_uT");
      dataFile.flush();
      USB_OUT.println("CSV file created and ready");
      sdReady=true;
    } else {
      USB_OUT.println("ERROR: Could not open data.csv");
      sdReady=false;
    }
  }
  
  // Initialize I2C for IMU
  Wire1.begin();
  Wire1.setClock(400000);  // 400kHz I2C
  
  // Initialize LSM6DS3 (Accel + Gyro)
  if (!lsm6ds3.begin_I2C(0x6A, &Wire1)) {
    if (USB_OUT) USB_OUT.println("ERROR: LSM6DS3 not found!");
    imuReady = false;
  }
  if (USB_OUT) USB_OUT.println("LSM6DS3 initialized");
  
  // Initialize LIS3MDL (Magnetometer)
  if (!lis3mdl.begin_I2C(0x1C, &Wire1)) {
    if (USB_OUT) USB_OUT.println("ERROR: LIS3MDL not found!");
    imuReady = false;
    while (1) {
      delay(1000);  // Halt if IMU critical
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink LED
    }
  }
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  if (USB_OUT) USB_OUT.println("LIS3MDL initialized");
  
  imuReady = true;
  
  if (USB_OUT) {
    USB_OUT.println();
    USB_OUT.println("System ready - logging data at 10 Hz");
    USB_OUT.println("Active outputs:");
    if (ENABLE_SD_OUTPUT && sdReady) USB_OUT.println("  - SD Card (CSV)");
    if (ENABLE_USB_OUTPUT) USB_OUT.println("  - USB Serial (Human Readable)");
    if (ENABLE_BT_OUTPUT) USB_OUT.println("  - Bluetooth HC-05 (CSV)");
    USB_OUT.println("=====================================");
    USB_OUT.println();
  }
}

// ============================================================
// MAIN LOOP
// ============================================================
unsigned long lastOutput = 0;
const unsigned long OUTPUT_INTERVAL = 100;  // 10 Hz (1000ms)

void loop() {
  // Process GPS data
  while (GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();
    
    if (c == '\n') {
      nmeaBuffer[nmeaIndex] = '\0';
      parseNMEA(String(nmeaBuffer));
      nmeaIndex = 0;
    } 
    else if (c != '\r') {
      if (nmeaIndex < sizeof(nmeaBuffer) - 1) {
        nmeaBuffer[nmeaIndex++] = c;
      }
    }
  }
  
  // Read IMU and output data at fixed interval
  if (millis() - lastOutput >= OUTPUT_INTERVAL) {
    lastOutput = millis();
    
    if (imuReady) {
      readIMU();
      outputData();
    }
  }
}