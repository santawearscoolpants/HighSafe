// ✅ Correct Processing sketch (for 3D visualization)
// DO NOT put Arduino includes here!

import processing.serial.*;

Serial myPort;
String myString = null;

float yaw = 0;
float pitch = 0;
float roll = 0;

void setup() {
  size(600, 600, P3D);
  println(Serial.list());
  
  // Choose your port index!
  myPort = new Serial(this, Serial.list()[6], 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(200);
  lights();
  
  translate(width/2, height/2, 0);

  // Apply rotations for yaw, pitch, roll
  rotateY(radians(yaw));
  rotateX(radians(pitch));
  rotateZ(radians(roll));

  fill(100, 100, 250);
  box(200, 50, 100);
}

void serialEvent(Serial myPort) {
  myString = myPort.readStringUntil('\n');
  if (myString != null) {
    myString = trim(myString);
    String[] values = split(myString, ',');
    if (values.length == 3) {
      yaw = float(values[0]);
      pitch = float(values[1]);
      roll = float(values[2]);
    }
  }
}
