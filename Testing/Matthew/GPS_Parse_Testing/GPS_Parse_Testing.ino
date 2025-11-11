// Human-readable GPS NMEA reader
// Reads full NMEA lines from hardware serial (Serial1), validates checksum,
// parses GGA and RMC, and prints user-friendly labeled output to USB Serial.

#define GPSSerial Serial1
#define GPS_BAUD 9600
#define USB_BAUD 115200

void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial) { delay(10); } // for native USB boards
  GPSSerial.begin(GPS_BAUD);
  Serial.println("GPS human-readable monitor starting...");
}

void loop() {
  if (GPSSerial.available()) {
    String line = GPSSerial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    if (line.charAt(0) != '$') return;
    if (!validNMEAChecksum(line)) {
      Serial.println("[BAD CHECKSUM] " + line);
      return;
    }

    if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA") || line.startsWith("$GLGGA")) {
      showGGA(line);
    } else if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC") || line.startsWith("$GLRMC")) {
      showRMC(line);
    }
  }

  // Optional passthrough: forward any characters typed in USB terminal to GPS
  if (Serial.available()) {
    GPSSerial.write(Serial.read());
  }
}

// -------------------- Utilities --------------------

bool validNMEAChecksum(const String &s) {
  int asterisk = s.indexOf('*');
  if (asterisk < 0) return false;
  uint8_t cs = 0;
  for (int i = 1; i < asterisk; ++i) cs ^= (uint8_t)s.charAt(i);
  String hex = s.substring(asterisk + 1);
  if (hex.length() < 2) return false;
  char buf[3] = { hex.charAt(0), hex.charAt(1), 0 };
  uint8_t given = (uint8_t) strtol(buf, NULL, 16);
  return cs == given;
}

float nmeaToDecimal(const String &nm, const String &dir) {
  if (nm.length() < 3) return NAN;
  float val = nm.toFloat();
  float degrees = floor(val / 100.0);
  float minutes = val - (degrees * 100.0);
  float dec = degrees + minutes / 60.0;
  if (dir == "S" || dir == "W") dec = -dec;
  return dec;
}

// Split by commas into parts array (no more than maxParts)
int splitFields(const String &s, String parts[], int maxParts) {
  int idx = 0;
  int start = 0;
  for (int i = 0; i < s.length() && idx < maxParts; ++i) {
    if (s.charAt(i) == ',') {
      parts[idx++] = s.substring(start, i);
      start = i + 1;
    }
  }
  if (idx < maxParts) parts[idx++] = s.substring(start);
  return idx;
}

// -------------------- Display helpers --------------------

String formatUTC(const String &utc) {
  if (utc.length() < 6) return String("");
  String hh = utc.substring(0, 2);
  String mm = utc.substring(2, 4);
  String ss = utc.substring(4);
  // strip fractional seconds if present
  int dot = ss.indexOf('.');
  if (dot > 0) ss = ss.substring(0, dot);
  return hh + ":" + mm + ":" + ss + " UTC";
}

String formatDate(const String &ddmmyy) {
  if (ddmmyy.length() != 6) return String("");
  String d = ddmmyy.substring(0, 2);
  String m = ddmmyy.substring(2, 4);
  String y = ddmmyy.substring(4, 6);
  return d + "/" + m + "/20" + y;
}

// -------------------- Parsers and pretty print --------------------

void showGGA(const String &line) {
  // remove leading $ and trailing checksum segment for splitting
  int star = line.indexOf('*');
  String core = (star > 0) ? line.substring(1, star) : line.substring(1);
  String parts[15];
  splitFields(core, parts, 15);

  String utc = parts[1];
  String latN = parts[2]; String latDir = parts[3];
  String lonN = parts[4]; String lonDir = parts[5];
  String fix = parts[6];
  String sats = parts[7];
  String hdop = parts[8];
  String alt = parts[9];

  Serial.println("----- GGA (Fix data) -----");
  Serial.print("UTC Time: "); Serial.println(formatUTC(utc));
  if (fix == "0") {
    Serial.println("Fix: NO FIX");
    Serial.println();
    return;
  } else {
    Serial.print("Fix: "); Serial.println(fix); // 1 = GPS, 2 = DGPS, etc.
  }

  float lat = nmeaToDecimal(latN, latDir);
  float lon = nmeaToDecimal(lonN, lonDir);

  Serial.print("Latitude: "); Serial.println(String(lat, 6));
  Serial.print("Longitude: "); Serial.println(String(lon, 6));
  Serial.print("Altitude: "); Serial.print(alt); Serial.println(" m");
  Serial.print("Number of Satellites: "); Serial.println(sats);
  Serial.print("HDOP: "); Serial.println(hdop);
  Serial.println();
}

void showRMC(const String &line) {
  int star = line.indexOf('*');
  String core = (star > 0) ? line.substring(1, star) : line.substring(1);
  String parts[20];
  splitFields(core, parts, 20);

  String utc = parts[1];
  String status = parts[2];
  String latN = parts[3]; String latDir = parts[4];
  String lonN = parts[5]; String lonDir = parts[6];
  String sog = parts[7]; // knots
  String track = parts[8];
  String date = parts[9];

  Serial.println("----- RMC (Recommended Minimum) -----");
  Serial.print("UTC Time: "); Serial.println(formatUTC(utc));
  Serial.print("Date: "); Serial.println(formatDate(date));

  if (status != "A") {
    Serial.println("Status: INVALID (V)");
    Serial.println();
    return;
  } else {
    Serial.println("Status: VALID (A)");
  }

  float lat = nmeaToDecimal(latN, latDir);
  float lon = nmeaToDecimal(lonN, lonDir);
  float speedKph = sog.toFloat() * 1.852;

  Serial.print("Latitude: "); Serial.println(String(lat, 6));
  Serial.print("Longitude: "); Serial.println(String(lon, 6));
  Serial.print("Speed: "); Serial.print(String(speedKph, 2)); Serial.println(" km/h");
  Serial.print("Track (bearing): "); Serial.print(track); Serial.println(" °");
  Serial.println();
}
