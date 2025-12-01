#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <SD.h>
#include <SPI.h>

// =========================
// HARDWARE CONFIGURATION
// =========================
#define GPS_SERIAL Serial1           // GPS module (RX1/TX1)
#define USB_OUT Serial               // USB output (115200 baud)
#define BT_SERIAL Serial7            // HC-05 Bluetooth module (9600 baud)
#define SD_CS_PIN BUILTIN_SDCARD     // Teensy 4.1 built-in SD

// Output flags
#define ENABLE_USB_OUTPUT true
#define ENABLE_BT_OUTPUT true
#define ENABLE_SD_OUTPUT true

// IMU sensors
Adafruit_LSM6DS3TRC lsm6ds3;
Adafruit_LIS3MDL lis3mdl;

// SD card file
File dataFile;

// System flags
bool sdReady = false;
bool imuReady = false;
int uptimeSeconds=0;

// =========================
// DATA STRUCTURES
// =========================
struct GPSData {
  float latitude = 0.0;
  float longitude = 0.0;
  float altitude = 0.0;
  uint8_t satellites = 0;
  bool valid = false;
};

struct IMUData {
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  float accel_x = 0, accel_y = 0, accel_z = 0;
  float mag_x = 0, mag_y = 0, mag_z = 0;
};

GPSData gps;
IMUData imu;

// =========================
// GPS PARSING
// =========================
#define NMEA_BUF_SIZE 128
char nmeaBuffer[NMEA_BUF_SIZE];
uint8_t nmeaIndex = 0;

float nmeaToDecimal(float raw, char hemi) {
  int deg = (int)(raw / 100);
  float min = raw - deg * 100;
  float dec = deg + min / 60.0;
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return dec;
}

void parseGGA(char* fields[], int count) {
  if (count < 10) return;

  int fix = atoi(fields[6]);
  gps.valid = (fix > 0);
  if (!gps.valid) return;

  gps.latitude = nmeaToDecimal(atof(fields[2]), fields[3][0]);
  gps.longitude = nmeaToDecimal(atof(fields[4]), fields[5][0]);
  gps.satellites = atoi(fields[7]);
  gps.altitude = atof(fields[9]);
}

void parseNMEA(char* sentence, size_t len) {
  if (len < 6 || sentence[0] != '$') return;

  // Check GGA
  if (strncmp(sentence + 3, "GGA", 3) != 0) return;

  // Split fields in-place
  char* fields[20];
  uint8_t count = 0;
  fields[count++] = sentence;

  for (size_t i = 0; i < len && count < 20; i++) {
    if (sentence[i] == ',') {
      sentence[i] = '\0';
      fields[count++] = &sentence[i + 1];
    }
  }

  parseGGA(fields, count);
}

void processGPSChar(char c) {
  if (c == '\n' || c == '\r') {
    if (nmeaIndex > 0) {
      nmeaBuffer[nmeaIndex] = '\0';
      parseNMEA(nmeaBuffer, nmeaIndex);
      nmeaIndex = 0;
    }
    return;
  }

  if (nmeaIndex < NMEA_BUF_SIZE - 1) {
    nmeaBuffer[nmeaIndex++] = c;
  } else {
    // Buffer overflow – reset
    nmeaIndex = 0;
  }
}

// =========================
// IMU READING
// =========================
void readIMU() {
  sensors_event_t accel, gyro, mag, temp;
  lsm6ds3.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  imu.gyro_x = gyro.gyro.x;
  imu.gyro_y = gyro.gyro.y;
  imu.gyro_z = gyro.gyro.z;

  imu.accel_x = accel.acceleration.x;
  imu.accel_y = accel.acceleration.y;
  imu.accel_z = accel.acceleration.z;

  imu.mag_x = mag.magnetic.x;
  imu.mag_y = mag.magnetic.y;
  imu.mag_z = mag.magnetic.z;
}

// =========================
// DATA OUTPUT
// =========================
String buildCSVLine() {
  String line;
  line += String(uptimeSeconds) + ",";
  line += String(gps.latitude, 6) + ",";
  line += String(gps.longitude, 6) + ",";
  line += String(gps.altitude, 2) + ",";
  line += String(gps.satellites) + ",";
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

void printHumanReadable(Stream &out) {
  unsigned long hours = uptimeSeconds / 3600;
  unsigned long minutes = (uptimeSeconds % 3600) / 60;
  unsigned long seconds = uptimeSeconds % 60;
  
  out.print("Uptime: ");
  if (hours < 10) out.print("0");
    out.print(hours);
  out.print(":");
  if (minutes < 10) out.print("0");
    out.print(minutes);
  out.print(":");
  if (seconds < 10) out.print("0");
    out.println(seconds);

  out.print("Lat: "); out.print(gps.latitude,6);
  out.print(", Lon: "); out.print(gps.longitude,6);
  out.print(", Alt: "); out.print(gps.altitude,2);
  out.print(", Sat: "); out.println(gps.satellites);

  out.print("Gyro: "); out.print(imu.gyro_x,4);
  out.print(","); out.print(imu.gyro_y,4);
  out.print(","); out.println(imu.gyro_z,4);

  out.print("Accel: "); out.print(imu.accel_x,4);
  out.print(","); out.print(imu.accel_y,4);
  out.print(","); out.println(imu.accel_z,4);

  out.print("Mag: "); out.print(imu.mag_x,4);
  out.print(","); out.print(imu.mag_y,4);
  out.print(","); out.println(imu.mag_z,4);
}

void outputData() {
  String csv = buildCSVLine();

  if (ENABLE_SD_OUTPUT && sdReady && dataFile) {
    dataFile.println(csv);
  }

  if (ENABLE_USB_OUTPUT) printHumanReadable(USB_OUT);
  if (ENABLE_BT_OUTPUT) printHumanReadable(BT_SERIAL); // Use CSV for BT
}

// =========================
// SETUP
// =========================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  USB_OUT.begin(115200);
  GPS_SERIAL.begin(9600);
  BT_SERIAL.begin(9600);

  delay(500);
  USB_OUT.println("=== Teensy GPS + IMU Logger ===");

  // Initialize SD
  if (ENABLE_SD_OUTPUT && SD.begin(SD_CS_PIN)) {
    dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Uptime(s),Lat,Lon,Alt(MSL),Sat,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,MagX,MagY,MagZ");
      sdReady = true;
    }
  }

  // I2C for IMU
  Wire1.begin();
  Wire1.setClock(400000);

  imuReady = lsm6ds3.begin_I2C(0x6A, &Wire1) && lis3mdl.begin_I2C(0x1C, &Wire1);
  if (!imuReady) {
    USB_OUT.println("IMU not found!");
    while (1) { digitalWrite(LED_BUILTIN, millis() % 500 < 250); }
  }

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
}

// =========================
// LOOP
// =========================
const unsigned long OUTPUT_INTERVAL = 100; // 10 Hz
unsigned long lastOutput = 0;


void loop() {
  // GPS processing
  while (GPS_SERIAL.available()) {
    processGPSChar(GPS_SERIAL.read());
  }

  // Data output
  if (millis() - lastOutput >= OUTPUT_INTERVAL) {
    lastOutput = millis();

    if (imuReady) {
      readIMU();
      outputData();
    }
    // UpTime
    static unsigned long lastUptimeUpdate = 0;
    
    if (millis() - lastUptimeUpdate >= 1000) {
      lastUptimeUpdate = millis();
      uptimeSeconds++;
    }
    // Flush SD every second
    static unsigned long lastFlush = 0;
    if (sdReady && millis() - lastFlush >= 1000) {
      dataFile.flush();
      lastFlush = millis();
    }
  }
}
