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
  rotateY(radians(yaw)); //roll interpreted as yaw
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
