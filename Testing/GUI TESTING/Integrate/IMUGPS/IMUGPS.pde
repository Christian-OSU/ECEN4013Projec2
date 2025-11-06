import processing.serial.*;

Serial myPort;
float pitch, roll, yaw;
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float magX, magY, magZ;
String time, fix, sat, lat, longi;

void setup() {
  pixelDensity(1);
  println(Serial.list());
  size(600, 600, P3D);
  String portName = "COM3";  // change this if needed
  try{
    myPort = new Serial(this, portName, 115200);
    myPort.bufferUntil('\n');
    println("Connected to " + portName);
  } catch (Exception e){
    println("Error opening serial port " + portName + ": " + e.getMessage());
    exit();
  }
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



  lights();
  translate(width/2, height/2, 0);
  rotateZ(radians(-roll)); //yaw interpreted as roll
  rotateX(radians(pitch));
  rotateY(radians(-yaw)); //roll interpreted as yaw
  fill(100, 200, 250);
  box(150, 30, 100);// Draw forward-pointing arrow
  pushMatrix();
  translate(0, 0, -60);  // Position above the box (adjust Z as needed)
  fill(255, 0, 0);       // Red arrow
  stroke(255, 0, 0);
  strokeWeight(2);

  beginShape();
  vertex(0, -10, 0);  // Tail left
  vertex(0, 10, 0);   // Tail right
  vertex(0, 10, -20); // Shaft tip
  vertex(0, -10, -20);
  endShape(CLOSE);

  // Arrowhead
  beginShape();
  vertex(0, 10, -20);
  vertex(0, 0, -40);  // Point
  vertex(0, -10, -20);
  endShape(CLOSE);

  popMatrix();
  
// Green arrow (positive Y direction — upward relative to box)
pushMatrix();
translate(0, -60, 0);  // Move above the box
fill(0, 255, 0);
stroke(0, 255, 0);
strokeWeight(2);

// Shaft
beginShape();
vertex(-10, 0, 0);
vertex(10, 0, 0);
vertex(10, -20, 0);
vertex(-10, -20, 0);
endShape(CLOSE);

// Arrowhead
beginShape();
vertex(10, -20, 0);
vertex(0, -40, 0);
vertex(-10, -20, 0);
endShape(CLOSE);
popMatrix();

// Blue arrow (negative X direction — out the left side)
pushMatrix();
translate(-80, 0, 0);  // Move to the left of the box
fill(0, 0, 255);       // Blue
stroke(0, 0, 255);
strokeWeight(2);

// Shaft
beginShape();
vertex(0, -10, -10);
vertex(-20, -10, -10);
vertex(-20, 10, -10);
vertex(0, 10, -10);
endShape(CLOSE);

// Arrowhead
beginShape();
vertex(-20, -10, -10);
vertex(-40, 0, -10);
vertex(-20, 10, -10);
endShape(CLOSE);
popMatrix();
}

void serialEvent(Serial myPort) {
  String inString = trim(myPort.readStringUntil('\n'));
  print("INSTRING: " + inString);
  println();
  if (inString != null) {
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
}
