#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <SD.h>
#include <SPI.h>

#define GPS_SERIAL Serial1
#define USB_OUT Serial
#define BT_SERIAL Serial7
#define SD_CS_PIN BUILTIN_SDCARD
#define TIMEZONE_OFFSET -6
#define ENABLE_USB_OUTPUT true
#define ENABLE_BT_OUTPUT true
#define ENABLE_SD_OUTPUT true

Adafruit_LSM6DS3TRC lsm6ds3;
Adafruit_LIS3MDL lis3mdl;
File dataFile;

bool sdReady = false;
bool imuReady = false;
int uptimeSeconds = 0;

struct GPSData {
  float latitude = 0.0, longitude = 0.0, altitude = 0.0;
  uint8_t satellites = 0;
  bool valid = false;
  uint8_t hour = 0, minute = 0, second = 0;
  uint8_t day = 0, month = 0;
  uint16_t year = 0;
  bool timeValid = false;
  uint8_t local_hour = 0, local_day = 0, local_month = 0;
  uint16_t local_year = 0;
};

struct IMUData {
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  float accel_x = 0, accel_y = 0, accel_z = 0;
  float mag_x = 0, mag_y = 0, mag_z = 0;
};

GPSData gps;
IMUData imu;

void convertToLocalTime() {
  if (!gps.timeValid || gps.year == 0) return;
  
  int local_hour = (int)gps.hour + TIMEZONE_OFFSET;
  int local_day = gps.day;
  int local_month = gps.month;
  int local_year = gps.year;
  
  if (local_hour < 0) {
    local_hour += 24;
    local_day--;
    if (local_day < 1) {
      local_month--;
      if (local_month < 1) {
        local_month = 12;
        local_year--;
      }
      int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      if (local_month == 2 && (local_year % 4 == 0 && (local_year % 100 != 0 || local_year % 400 == 0))) {
        local_day = 29;
      } else {
        local_day = daysInMonth[local_month - 1];
      }
    }
  } else if (local_hour >= 24) {
    local_hour -= 24;
    local_day++;
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int maxDays = daysInMonth[local_month - 1];
    if (local_month == 2 && (local_year % 4 == 0 && (local_year % 100 != 0 || local_year % 400 == 0))) {
      maxDays = 29;
    }
    if (local_day > maxDays) {
      local_day = 1;
      local_month++;
      if (local_month > 12) {
        local_month = 1;
        local_year++;
      }
    }
  }
  
  gps.local_hour = local_hour;
  gps.local_day = local_day;
  gps.local_month = local_month;
  gps.local_year = local_year;
}

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

  char* timeStr = fields[1];
  if (timeStr && strlen(timeStr) >= 6) {
    gps.hour = (timeStr[0] - '0') * 10 + (timeStr[1] - '0');
    gps.minute = (timeStr[2] - '0') * 10 + (timeStr[3] - '0');
    gps.second = (timeStr[4] - '0') * 10 + (timeStr[5] - '0');
    gps.timeValid = true;
  }

  gps.latitude = nmeaToDecimal(atof(fields[2]), fields[3][0]);
  gps.longitude = nmeaToDecimal(atof(fields[4]), fields[5][0]);
  gps.satellites = atoi(fields[7]);
  gps.altitude = atof(fields[9]);
}

void parseRMC(char* fields[], int count) {
  // fields: [0] = "$GPRMC" etc, [1]=time, [2]=status, [3]=lat, [4]=N/S, [5]=lon, [6]=E/W, [7]=speed, [8]=track, [9]=date, ...
  if (count < 10) return;

  // Debug outputs (you can comment these out once working)
  USB_OUT.print("\t[Parsing RMC] Status: ");
  if (fields[2]) USB_OUT.print(fields[2]);
  USB_OUT.print(" DateField: ");
  if (fields[9]) USB_OUT.println(fields[9]);
  else USB_OUT.println("(none)");

  // Only parse if date field present
  if (fields[9] && strlen(fields[9]) >= 6) {
    char* dateStr = fields[9];
    if (dateStr[0] >= '0' && dateStr[0] <= '9') {
      gps.day = (dateStr[0] - '0') * 10 + (dateStr[1] - '0');
      gps.month = (dateStr[2] - '0') * 10 + (dateStr[3] - '0');
      gps.year = 2000 + (dateStr[4] - '0') * 10 + (dateStr[5] - '0');
      USB_OUT.print("\t>> Parsed Date: ");
      USB_OUT.print(gps.year); USB_OUT.print("-");
      USB_OUT.print(gps.month); USB_OUT.print("-");
      USB_OUT.println(gps.day);
    }
  }

  // Parse time if present (fields[1] is hhmmss.sss)
  if (fields[1] && strlen(fields[1]) >= 6) {
    char* t = fields[1];
    gps.hour = (t[0] - '0') * 10 + (t[1] - '0');
    gps.minute = (t[2] - '0') * 10 + (t[3] - '0');
    gps.second = (t[4] - '0') * 10 + (t[5] - '0');
    gps.timeValid = true;
  }

  // Status: 'A' = valid, 'V' = void
  if (fields[2] && fields[2][0] == 'A') {
    gps.valid = true;
  } else {
    // don't clear positional data if status is V; just mark invalid if needed
    // gps.valid = false; // optional
  }

  // Parse lat/lon from RMC too (fields indices match GGA)
  if (fields[3] && fields[5]) {
    gps.latitude = nmeaToDecimal(atof(fields[3]), fields[4][0]);
    gps.longitude = nmeaToDecimal(atof(fields[5]), fields[6][0]);
  }
}

void parseNMEA(char* sentence, size_t len) {
  if (len < 6 || sentence[0] != '$') return;

  USB_OUT.print("RAW >> ");
  USB_OUT.println(sentence);

  bool isGGA = (strstr(sentence, "GGA") != NULL);
  bool isRMC = (strstr(sentence, "RMC") != NULL);
  
  if (!isGGA && !isRMC) return;

  char* fields[20];
  uint8_t count = 0;
  fields[count++] = sentence;

  for (size_t i = 0; i < len && count < 20; i++) {
    if (sentence[i] == ',') {
      sentence[i] = '\0';
      fields[count++] = &sentence[i + 1];
    }
  }

  if (isGGA) parseGGA(fields, count);
  else if (isRMC) parseRMC(fields, count);
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
    nmeaIndex = 0; // overflow guard
  }
}

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

String formatUptime() {
  unsigned long hours = uptimeSeconds / 3600;
  unsigned long minutes = (uptimeSeconds % 3600) / 60;
  unsigned long seconds = uptimeSeconds % 60;
  char buffer[9];
  sprintf(buffer, "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(buffer);
}

String buildCSVLine() {
  String line = formatUptime() + ",";
  
  if (gps.timeValid && gps.year > 0) {
    convertToLocalTime();
    char dateBuffer[11], timeBuffer[9];
    sprintf(dateBuffer, "%04d-%02d-%02d", gps.local_year, gps.local_month, gps.local_day);
    sprintf(timeBuffer, "%02d:%02d:%02d", gps.local_hour, gps.minute, gps.second);
    line += String(dateBuffer) + "," + String(timeBuffer) + ",";
  } else {
    line += ",,";
  }
  
  line += String(gps.latitude, 6) + "," + String(gps.longitude, 6) + ",";
  line += String(gps.altitude, 2) + "," + String(gps.satellites) + ",";
  line += String(imu.gyro_x, 4) + "," + String(imu.gyro_y, 4) + "," + String(imu.gyro_z, 4) + ",";
  line += String(imu.accel_x, 4) + "," + String(imu.accel_y, 4) + "," + String(imu.accel_z, 4) + ",";
  line += String(imu.mag_x, 4) + "," + String(imu.mag_y, 4) + "," + String(imu.mag_z, 4);
  return line;
}

void printHumanReadable(Stream &out) {
  out.print("Uptime: ");
  out.println(formatUptime());
  
  if (gps.timeValid && gps.year > 0) {
    convertToLocalTime();
    out.print("Local Date: ");
    out.print(gps.local_year); out.print("-");
    if (gps.local_month < 10) out.print("0");
    out.print(gps.local_month); out.print("-");
    if (gps.local_day < 10) out.print("0");
    out.println(gps.local_day);
    
    out.print("Local Time: ");
    if (gps.local_hour < 10) out.print("0");
    out.print(gps.local_hour); out.print(":");
    if (gps.minute < 10) out.print("0");
    out.print(gps.minute); out.print(":");
    if (gps.second < 10) out.print("0");
    out.print(gps.second);
    out.print(" (UTC");
    if (TIMEZONE_OFFSET >= 0) out.print("+");
    out.print(TIMEZONE_OFFSET);
    out.println(")");
  }

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
  if (ENABLE_SD_OUTPUT && sdReady && dataFile) dataFile.println(csv);
  if (ENABLE_USB_OUTPUT) printHumanReadable(USB_OUT);
  if (ENABLE_BT_OUTPUT) printHumanReadable(BT_SERIAL);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  USB_OUT.begin(115200);
  GPS_SERIAL.begin(9600);

  delay(100);
  // Request only GGA and RMC
  GPS_SERIAL.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(100);
  
  BT_SERIAL.begin(9600);
  delay(500);
  USB_OUT.println("=== Teensy GPS + IMU Logger ===");

  if (ENABLE_SD_OUTPUT && SD.begin(SD_CS_PIN)) {
    USB_OUT.println("SD card initialized");
    if (SD.exists("data.csv")) {
      USB_OUT.println("Deleting existing data.csv...");
      SD.remove("data.csv");
    }
    dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Uptime,Local_Date,Local_Time,Lat,Lon,Alt_MSL,Sat,GyroX_rad/s,GyroY_rad/s,GyroZ_rad/s,AccelX_m/s2,AccelY_m/s2,AccelZ_m/s2,MagX_uT,MagY_uT,MagZ_uT");
      USB_OUT.println("CSV file created");
      sdReady = true;
    }
  }

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
  USB_OUT.println("System ready - logging at 10 Hz");
}

const unsigned long OUTPUT_INTERVAL = 100;
unsigned long lastOutput = 0;

void loop() {
  while (GPS_SERIAL.available()) {
    processGPSChar(GPS_SERIAL.read());
  }

  if (millis() - lastOutput >= OUTPUT_INTERVAL) {
    lastOutput = millis();
    if (imuReady) {
      readIMU();
      outputData();
    }
  }
  
  static unsigned long lastUptimeUpdate = 0;
  if (millis() - lastUptimeUpdate >= 1000) {
    lastUptimeUpdate = millis();
    uptimeSeconds++;
  }
  
  static unsigned long lastFlush = 0;
  if (sdReady && millis() - lastFlush >= 1000) {
    dataFile.flush();
    lastFlush = millis();
  }
}
