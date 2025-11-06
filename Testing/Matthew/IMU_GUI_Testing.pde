import processing.serial.*;

Serial myPort;
float pitch = 0, roll = 0, yaw = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float magX = 0, magY = 0, magZ = 0;
String time, fix, sat, lat, longi;

// --- Rolling buffer settings for strip charts
final int BUF_LEN = 300;        // number of samples to keep
int bufIndex = 0;               // next write index (circular)

float[] accelXbuf = new float[BUF_LEN];
float[] accelYbuf = new float[BUF_LEN];
float[] accelZbuf = new float[BUF_LEN];

float[] gyroXbuf = new float[BUF_LEN];
float[] gyroYbuf = new float[BUF_LEN];
float[] gyroZbuf = new float[BUF_LEN];

float[] magXbuf = new float[BUF_LEN];
float[] magYbuf = new float[BUF_LEN];
float[] magZbuf = new float[BUF_LEN];

// sampling control (push to buffers at fixed rate)
final int SAMPLE_MS = 30;         // push a sample every 30 ms (adjust)
int lastPushTime = 0;

// autoscale state (smoothed)
float accelVScale = 16.0f;
float gyroVScale  = 8.0f;
float magVScale   = 200.0f;

// smoothing factors for autoscale (0..1, lower = slower)
final float SCALE_SMOOTH = 0.08f;
final float MAX_CLAMP_FACTOR = 4.0f; // don't autoscale beyond this factor quickly
final float PEAK_MARGIN = 1.25f;     // scale = peak * PEAK_MARGIN

void setup() {
  pixelDensity(1);
  println(Serial.list());
  size(900, 600, P3D); // wider to make room for charts
  String portName = "COM3";  // change this if needed
  try{
    myPort = new Serial(this, portName, 115200);
    myPort.bufferUntil('\n');
    println("Connected to " + portName);
  } catch (Exception e){
    println("Error opening serial port " + portName + ": " + e.getMessage());
    // continue without serial for testing if desired
  }

  // initialize buffers with zeros
  for (int i=0; i<BUF_LEN; i++){
    accelXbuf[i]=accelYbuf[i]=accelZbuf[i]=0;
    gyroXbuf[i]=gyroYbuf[i]=gyroZbuf[i]=0;
    magXbuf[i]=magYbuf[i]=magZbuf[i]=0;
  }
  lastPushTime = millis();
}

void draw() {
  background(30);
  fill(255);
  textSize(14);
  textAlign(LEFT, TOP);
  int y = 10;

  text("IMU",10,y); y+=18;
  text("----------------------------------",10,y); y+=24;
  text("Orientation (Yaw, Pitch, Roll):", 10, y); y += 18;
  text(nf(yaw, 1, 2) + ", " + nf(pitch, 1, 2) + ", " + nf(roll, 1, 2), 20, y); y += 24;

  text("Angular Velocity (rad/s):", 10, y); y += 18;
  text(nf(gyroX, 1, 2) + ", " + nf(gyroY, 1, 2) + ", " + nf(gyroZ, 1, 2), 20, y); y += 24;

  text("Acceleration (m/s²):", 10, y); y += 18;
  text(nf(accelX, 1, 2) + ", " + nf(accelY, 1, 2) + ", " + nf(accelZ, 1, 2), 20, y); y += 24;

  text("Magnetic Field (µT):", 10, y); y += 18;
  text(nf(magX, 1, 2) + ", " + nf(magY, 1, 2) + ", " + nf(magZ, 1, 2), 20, y); y +=24;

  text("GPS",12,y); y+=18;
  text("----------------------------------",10,y); y+=24;

  text("Time: " + time, 10, y); y+=24;
  text("GPS Fix: " + fix, 10, y); y+=24;
  text("Satellites Locked: " + sat, 10, y); y+=24;
  text("Latitude: " + lat, 10, y); y+=24;
  text("Longitude: " + longi, 10, y); y+=24;

  // --- timed push into buffers (steady time axis) ---
  int now = millis();
  if (now - lastPushTime >= SAMPLE_MS) {
    // optional small smoothing of incoming values (helps remove serial jitter)
    float smoothAlpha = 0.2f; // 0 = keep old, 1 = immediate jump

    // previous index (last sample) to lerp from
    int prevIdx = (bufIndex + BUF_LEN - 1) % BUF_LEN;

    // push accel
    accelXbuf[bufIndex] = lerp(accelXbuf[prevIdx], accelX, smoothAlpha);
    accelYbuf[bufIndex] = lerp(accelYbuf[prevIdx], accelY, smoothAlpha);
    accelZbuf[bufIndex] = lerp(accelZbuf[prevIdx], accelZ, smoothAlpha);
    // push gyro
    gyroXbuf[bufIndex] = lerp(gyroXbuf[prevIdx], gyroX, smoothAlpha);
    gyroYbuf[bufIndex] = lerp(gyroYbuf[prevIdx], gyroY, smoothAlpha);
    gyroZbuf[bufIndex] = lerp(gyroZbuf[prevIdx], gyroZ, smoothAlpha);
    // push mag
    magXbuf[bufIndex] = lerp(magXbuf[prevIdx], magX, smoothAlpha);
    magYbuf[bufIndex] = lerp(magYbuf[prevIdx], magY, smoothAlpha);
    magZbuf[bufIndex] = lerp(magZbuf[prevIdx], magZ, smoothAlpha);

    // --- PEAK-BASED autoscale: compute recent peak from buffers ---
    // compute max absolute over entire buffer for each sensor group
    float peakGyro = 0;
    float peakAccel = 0;
    float peakMag = 0;
    for (int i=0; i<BUF_LEN; i++){
      peakGyro = max(peakGyro, abs(gyroXbuf[i]));
      peakGyro = max(peakGyro, abs(gyroYbuf[i]));
      peakGyro = max(peakGyro, abs(gyroZbuf[i]));

      peakAccel = max(peakAccel, abs(accelXbuf[i]));
      peakAccel = max(peakAccel, abs(accelYbuf[i]));
      peakAccel = max(peakAccel, abs(accelZbuf[i]));

      peakMag = max(peakMag, abs(magXbuf[i]));
      peakMag = max(peakMag, abs(magYbuf[i]));
      peakMag = max(peakMag, abs(magZbuf[i]));
    }

    // derive targets with margin and apply clamping to avoid runaway
    float targetGyro = max(1e-3f, peakGyro * PEAK_MARGIN);
    targetGyro = min(targetGyro, gyroVScale * MAX_CLAMP_FACTOR);
    gyroVScale = lerp(gyroVScale, targetGyro, SCALE_SMOOTH);

    float targetAccel = max(1e-3f, peakAccel * PEAK_MARGIN);
    targetAccel = min(targetAccel, accelVScale * MAX_CLAMP_FACTOR);
    accelVScale = lerp(accelVScale, targetAccel, SCALE_SMOOTH);

    float targetMag = max(1e-3f, peakMag * PEAK_MARGIN);
    targetMag = min(targetMag, magVScale * MAX_CLAMP_FACTOR);
    magVScale = lerp(magVScale, targetMag, SCALE_SMOOTH);

    // advance circular index (bufIndex always points to next write position)
    bufIndex = (bufIndex + 1) % BUF_LEN;
    lastPushTime = now;
  }

  // ----- 3D model area (left / center) -----
  pushMatrix();
  lights();
  translate(width*0.33f, height/2, 0);
  rotateZ(radians(-roll)); //yaw interpreted as roll
  rotateX(radians(pitch));
  rotateY(radians(-yaw)); //roll interpreted as yaw
  fill(100, 200, 250);
  box(150, 30, 100); // rectangular box (the box is drawn here)

  // Red arrow (negative X direction — out the left side)
  pushMatrix();
  translate(-80, 0, 0);  // Move to the left of the box
  fill(255, 0, 0);       // Red
  stroke(255, 0, 0);
  strokeWeight(2);
  beginShape();
  vertex(0, -10, -10);
  vertex(-20, -10, -10);
  vertex(-20, 10, -10);
  vertex(0, 10, -10);
  endShape(CLOSE);
  beginShape();
  vertex(-20, -10, -10);
  vertex(-40, 0, -10);
  vertex(-20, 10, -10);
  endShape(CLOSE);
  popMatrix();

  // Green arrow (positive Z direction — upward relative to box)
  pushMatrix();
  translate(0, -60, 0);  // Move above the box
  fill(0, 255, 0);
  stroke(0, 255, 0);
  strokeWeight(2);
  beginShape();
  vertex(-10, 0, 0);
  vertex(10, 0, 0);
  vertex(10, -20, 0);
  vertex(-10, -20, 0);
  endShape(CLOSE);
  beginShape();
  vertex(10, -20, 0);
  vertex(0, -40, 0);
  vertex(-10, -20, 0);
  endShape(CLOSE);
  popMatrix();

  // Blue arrow (negative Y direction — out the top/behind side)
  pushMatrix();
  translate(0, 0, -60);  // Position behind the box (adjust Z as needed)
  fill(0, 0, 255);       // Blue
  stroke(0, 0, 255);
  strokeWeight(2);
  beginShape();
  vertex(0, -10, 0);  // Tail left
  vertex(0, 10, 0);   // Tail right
  vertex(0, 10, -20); // Shaft tip
  vertex(0, -10, -20);
  endShape(CLOSE);
  beginShape();
  vertex(0, 10, -20);
  vertex(0, 0, -40);  // Point
  vertex(0, -10, -20);
  endShape(CLOSE);
  popMatrix();

  popMatrix(); // end 3D model area

  // ----- Strip charts area (right side) -----
  float chartX = width*0.66f + 10;
  float chartW = width*0.33f - 20;
  float chartH = (height - 40) / 3.0f; // three stacked charts
  float chartY = 10;

  // Draw accel chart
  drawStripChart(chartX, chartY, chartW, chartH, "Acceleration (m/s²)", accelXbuf, accelYbuf, accelZbuf, accelVScale);

  // Draw gyro chart
  chartY += chartH + 10;
  drawStripChart(chartX, chartY, chartW, chartH, "Gyro (rad/s)", gyroXbuf, gyroYbuf, gyroZbuf, gyroVScale);

  // Draw mag chart
  chartY += chartH + 10;
  drawStripChart(chartX, chartY, chartW, chartH, "Mag (µT)", magXbuf, magYbuf, magZbuf, magVScale);

  // optional: show info
  fill(200);
  textSize(12);
  textAlign(RIGHT, BOTTOM);
  text("Samp ms: " + SAMPLE_MS + "  BufLen: " + BUF_LEN, width-10, height-6);
}

// Helper: draw a 3-channel rolling strip chart
// Helper: draw a 3-channel rolling strip chart (legend moved below title)
void drawStripChart(float x, float y, float w, float h, String title,
                    float[] xb, float[] yb, float[] zb, float vscale) {
  pushMatrix();
  translate(x, y);
  // background and frame
  fill(20);
  stroke(80);
  rect(0, 0, w, h);
  fill(160);
  textSize(12);
  textAlign(LEFT, TOP);
  text(title, 6, 4);

  // legend area just under the title
  float legendY = 4 + 14; // title y + title height (approx)
  float legendX = w - 110;
  float boxSz = 10;
  noStroke();
  // X
  fill(255, 100, 100);
  rect(legendX, legendY, boxSz, boxSz);
  fill(200);
  textAlign(LEFT, TOP);
  text("X", legendX + boxSz + 4, legendY);
  // Y
  fill(100, 255, 100);
  rect(legendX + 34, legendY, boxSz, boxSz);
  fill(200);
  text("Y", legendX + 34 + boxSz + 4, legendY);
  // Z
  fill(100, 140, 255);
  rect(legendX + 68, legendY, boxSz, boxSz);
  fill(200);
  text("Z", legendX + 68 + boxSz + 4, legendY);

  // draw gridlines and zero line (start below legend)
  stroke(60);
  strokeWeight(1);
  int gridN = 4;
  float topPad = legendY + boxSz + 6; // leave room for title+legend
  for (int i=0; i<=gridN; i++){
    float gy = map(i, 0, gridN, topPad, h-6);
    line(6, gy, w-6, gy);
  }
  // zero line at center according to vscale
  stroke(100);
  float zy = map(0, -vscale, vscale, h-6, topPad);
  line(6, zy, w-6, zy);

  // show current vscale in corner
  noStroke();
  fill(200);
  textAlign(RIGHT, TOP);
  text(nf(vscale, 1, 2), w-10, 4);

  // plot three channels (use drawing area offset topPad)
  pushMatrix();
  translate(0, topPad);
  noFill();
  strokeWeight(1.5f);
  drawBufferLine(xb, color(255, 100, 100), w, h - topPad, vscale);
  drawBufferLine(yb, color(100, 255, 100), w, h - topPad, vscale);
  drawBufferLine(zb, color(100, 140, 255), w, h - topPad, vscale);
  popMatrix();

  popMatrix();
}


// draw one buffer line (circular) onto the current chart area
void drawBufferLine(float[] buf, int col, float w, float h, float vscale) {
  stroke(col);
  noFill();
  beginShape();
  // oldest sample index is bufIndex (since bufIndex points to next write)
  int oldest = bufIndex;
  for (int i=0; i<BUF_LEN; i++){
    int idx = (oldest + i) % BUF_LEN;
    float vx = map(i, 0, BUF_LEN-1, 6, w-6);
    float val = buf[idx];
    // clamp value to a safe range to avoid crazy spikes from skewing plot
    float clampVal = constrain(val, -vscale * 1.5f, vscale * 1.5f);
    float vy = map(clampVal, -vscale, vscale, h-6, 24);
    vertex(vx, vy);
  }
  endShape();
}

// Serial parsing: update the latest sensor variables only (no direct buffer writes here)
void serialEvent(Serial myPort) {
  String inString = trim(myPort.readStringUntil('\n'));
  if (inString == null) return;
  // println("INSTRING: " + inString);
  if (inString.startsWith("Orientation")) {
    String[] parts = split(inString, ":");
    if (parts.length > 1) {
      float[] values = float(split(trim(parts[1]), ","));
      if (values.length == 3) {
        yaw = values[2];
        pitch = values[0];
        roll = values[1];
      }
    }
  } else if (inString.startsWith("Angular Velocity")) {
    String[] parts = split(inString, ":");
    if (parts.length > 1) {
      float[] values = float(split(trim(parts[1]), ","));
      if (values.length == 3) {
        gyroX = values[0];
        gyroY = values[1];
        gyroZ = values[2];
      }
    }
  } else if (inString.startsWith("Linear Acceleration")) {
    String[] parts = split(inString, ":");
    if (parts.length > 1) {
      float[] values = float(split(trim(parts[1]), ","));
      if (values.length == 3) {
        accelX = values[0];
        accelY = values[1];
        accelZ = values[2];
      }
    }
  } else if (inString.startsWith("Magnetic Field")) {
    String[] parts = split(inString, ":");
    if (parts.length > 1) {
      float[] values = float(split(trim(parts[1]), ","));
      if (values.length == 3) {
        magX = values[0];
        magY = values[1];
        magZ = values[2];
      }
    }
  } else if (inString.startsWith("Time")){
    String[] parts = split(inString, ":");
    if (parts.length >= 4){
      time = parts[1] + ":" + parts[2] + ":" + parts[3];
    }
  } else if (inString.startsWith("Fix")){
    String[] parts = split(inString, ":");
    if (parts.length >=1){
      fix = parts[1];
    }
  } else if (inString.startsWith("Satellites")){
    String[] parts = split(inString, ":");
    if (parts.length >= 1){ sat = parts[1];}
  } else if (inString.startsWith("Latitude")){
    String[] parts = split(inString, ":");
    if (parts.length >=1) { lat=parts[1];}
  } else if (inString.startsWith("Longitude")){
    String[] parts = split(inString, ":");
    if (parts.length >=1) { longi=parts[1];}
  }
}
