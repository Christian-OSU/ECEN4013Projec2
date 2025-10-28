import processing.serial.*;

Serial myPort;
float yaw, pitch, roll;

void setup() {
  pixelDensity(1);
  println(Serial.list());
  size(600, 600, P3D);
  String portName = "COM9";  // change this if needed
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
  textSize(16);
  textAlign(LEFT, TOP);
  text("Gyro: " + pitch +", " + roll + ", " + yaw, 10, 10);
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
  if (inString != null) {
    float[] values = float(split(inString, ','));
    if (values.length == 3) {
      pitch = values[0];
      roll = values[1];
      yaw = values[2];
    }
    print("X: " + pitch + " Y: " + roll + " Z: " + yaw);
    println();
  }
}
