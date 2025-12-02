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
#define USB_SERIAL Serial            // USB output (115200 baud)
#define BT_SERIAL Serial7            // HC-05 Bluetooth module (9600 baud)
#define SD_CS_PIN BUILTIN_SDCARD     // Teensy 4.1 built-in SD

// IMU sensors
Adafruit_LSM6DS3TRC lsm6ds3;  // Accel + Gyro
Adafruit_LIS3MDL lis3mdl;     // Magnetometer

// SD Card file handle
File dataFile;

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

void outputData() {
  String csvLine = buildCSVLine();
  
  // 1. SD Card - write and flush (keep file open)
  if (dataFile) {
    dataFile.println(csvLine);
    dataFile.flush();  // Ensure data is written
  }
  
  // 2. USB Serial - human readable format
  USB_SERIAL.println("--- GPS DATA ---");
  USB_SERIAL.print("  Latitude:    "); USB_SERIAL.print(gps.latitude, 6); USB_SERIAL.println(" deg");
  USB_SERIAL.print("  Longitude:   "); USB_SERIAL.print(gps.longitude, 6); USB_SERIAL.println(" deg");
  USB_SERIAL.print("  Altitude:    "); USB_SERIAL.print(gps.altitude, 2); USB_SERIAL.println(" m (MSL)");
  USB_SERIAL.print("  Satellites:  "); USB_SERIAL.println(gps.satellites);
  
  USB_SERIAL.println("--- IMU DATA ---");
  USB_SERIAL.print("  Gyro (rad/s):  X="); USB_SERIAL.print(imu.gyro_x, 4);
  USB_SERIAL.print(" Y="); USB_SERIAL.print(imu.gyro_y, 4);
  USB_SERIAL.print(" Z="); USB_SERIAL.println(imu.gyro_z, 4);
  
  USB_SERIAL.print("  Accel (m/s²):  X="); USB_SERIAL.print(imu.accel_x, 4);
  USB_SERIAL.print(" Y="); USB_SERIAL.print(imu.accel_y, 4);
  USB_SERIAL.print(" Z="); USB_SERIAL.println(imu.accel_z, 4);
  
  USB_SERIAL.print("  Mag (µT):      X="); USB_SERIAL.print(imu.mag_x, 4);
  USB_SERIAL.print(" Y="); USB_SERIAL.print(imu.mag_y, 4);
  USB_SERIAL.print(" Z="); USB_SERIAL.println(imu.mag_z, 4);
  USB_SERIAL.println();
  
  // 3. Bluetooth (HC-05) - CSV format for easy parsing
  BT_SERIAL.println(csvLine);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  // Initialize serial ports
  USB_SERIAL.begin(115200);
  GPS_SERIAL.begin(9600);
  BT_SERIAL.begin(9600);  // HC-05 Bluetooth module
  
  // Wait for USB Serial to be ready (but don't wait forever)
  unsigned long startTime = millis();
  while (!USB_SERIAL && (millis() - startTime < 3000)) {
    delay(10);
  }
  
  delay(500);
  USB_SERIAL.println("\n\n=== GPS + IMU Data Logger ===");
  USB_SERIAL.println("Elevation Model: MSL (Mean Sea Level)");
  USB_SERIAL.println();
  USB_SERIAL.println();
  
  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    USB_SERIAL.println("ERROR: SD card initialization failed!");
  } else {
    USB_SERIAL.println("SD card initialized");
    
    // Create new file with header
    if (SD.exists("data.csv")) {
      SD.remove("data.csv");
    }
    
    dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Lat,Lon,Alt_MSL,Satellites,GyroX_rad/s,GyroY_rad/s,GyroZ_rad/s,AccelX_m/s2,AccelY_m/s2,AccelZ_m/s2,MagX_uT,MagY_uT,MagZ_uT");
      dataFile.flush();
      USB_SERIAL.println("CSV file created and ready");
    } else {
      USB_SERIAL.println("ERROR: Could not open data.csv");
    }
  }
  
  // Initialize I2C for IMU
  Wire1.begin();
  Wire1.setClock(400000);  // 400kHz I2C
  
  // Initialize LSM6DS3 (Accel + Gyro)
  if (!lsm6ds3.begin_I2C(0x6A, &Wire1)) {
    USB_SERIAL.println("ERROR: LSM6DS3 not found!");
    while (1) delay(10);
  }
  USB_SERIAL.println("LSM6DS3 initialized");
  
  // Initialize LIS3MDL (Magnetometer)
  if (!lis3mdl.begin_I2C(0x1C, &Wire1)) {
    USB_SERIAL.println("ERROR: LIS3MDL not found!");
    while (1) delay(10);
  }
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  USB_SERIAL.println("LIS3MDL initialized");
  
  USB_SERIAL.println();
  USB_SERIAL.println("System ready - logging data at 10 Hz");
  USB_SERIAL.println("=====================================");
  USB_SERIAL.println();
}

// ============================================================
// MAIN LOOP
// ============================================================
unsigned long lastOutput = 0;
const unsigned long OUTPUT_INTERVAL = 1000;  // 10 Hz (100ms)

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
    
    readIMU();
    outputData();
  }
}