/**
 * What is this file?  
 * This is a processing.org sketch and is *NOT* loaded onto your arduino.  
 * Instead the processing development environment runs this sketch as a 3d application on your computer.
 * The sketch connect to your arduino's serial port and reads the roll/pitch/yaw data and displays this
 * as 3d models.  This sketch assumes you've loaded RollPitchYaw.ino onto your nano iot 33 (or similar) board
 * 
 * License LGPL2.1+
 */

import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial arduino;

float gyroRoll,          gyroPitch,          gyroYaw;            // units degrees (expect major drift)
float gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw;   // units degrees (expect minor drift)
float accRoll,           accPitch,           accYaw;             // units degrees (roll and pitch noisy, yaw not possible)
float complementaryRoll, complementaryPitch, complementaryYaw;   // units degrees (excellent roll, pitch, yaw minor drift)

float x1, y1 ,z1;
float x2, y2, z2;

// NOTE: You need to provide or discover a portName.  Often it's at a fixed index (e.g. 0, 1) in the serial list.  Or you may prefer to hardcode it.  Look at commented lines below for examples.
int portIndex = 0;
//int portIndex = 1;

//String portName = Serial.list()[portIndex];
//String portName = "/dev/ttyACM0";  // example Linux portname
//String portName = "COM1";            // example windows portname
String portName = "/dev/cu.usbmodem14501";



long lastDataTime = millis();
boolean connected = false;

void setup() {
  
  size (1600, 800, P3D);
  arduino = new Serial(this, portName, 1000000);
 }

void draw() {
  
  if (millis() - lastDataTime > 1000 && connected == true) {
    // we were connected, but have received no data for 1s.  Lets reconnect
    connected = false;
    arduino.stop();
    // best to wait for serial port to free up
    delay(250);
    try{
      arduino = new Serial(this, portName, 1000000);
    } catch (RuntimeException ex) {
      // todo: look for better way to handle errors.  the following is equivalent to 'retry'. 
      connected = true;
      println(ex);
    }
  }
  textSize(22);
  translate(width/2, height/2, 0);
  if (connected) {
    background(0,150,0); // gray
    text("Connected to: " + portName, -300, -300);
  } else {
    background(150,0,0); // red
    text("Not connected to: " + portName + ", check portName/portIndex", -300, -300);
  }
  
  line(0, 0, x1*10, y1*10);
  line(0, 0, x1*10, z1*10);
    line(0, 0, y1*10, z1*10);
  line(10, 10, 100, 100);
  drawBox("gyro (raw)\n","✓ has yaw\n× major drift", -450, gyroPitch, gyroRoll, gyroYaw);
  drawBox("gyro (corrected)\n","✓ has yaw\n× minor drift", -150, gyroCorrectedPitch, gyroCorrectedRoll, gyroCorrectedYaw);
  //drawBox("accel (raw)\n","✓ zero drift\n× no yaw + noisy", 150, accPitch, accRoll, accYaw);
  //drawBox("complementary\n","✓great pitch/roll\n× minor yaw drift", 450, complementaryPitch, complementaryRoll, complementaryYaw);
  
}

float p, r ,y;

/* This draws a moving 3d box representing roll/pitch/yaw including textual stats */
void drawBox(String name, String description, int xShift, float pitch, float roll, float yaw) {
  pushMatrix();
  // Rotate the object
  translate(xShift,0,0);
  rotateX(radians(-pitch));
  rotateZ(radians(-roll));
  rotateY(radians(yaw));
  
  textSize(16);
    
  fill(0, 76, 183);
  box (160, 40, 300); // Draw box
  fill(255, 255, 255);
  text(name, -60, 5, 151);
  popMatrix();
  text( name 
        + description 
        + "\npitch: " + (pitch)
        + "\nroll: " + (roll)
        + "\nyaw: " + (yaw)
        , xShift -20, 200);
  
}

float smooth = 0.8;

void serialEvent (Serial myPort) {
  
  String data = myPort.readStringUntil('\n');
  // if you got any bytes other than the linefeed:
  if (data != null) {
    connected = true;
    lastDataTime = millis();
    data = trim(data);
    
    String items[] = split(data, ',');
    if (items.length > 1) {
      //--- Roll,Pitch in degrees
      gyroRoll = float(items[1]);
      gyroPitch = float(items[0]);
      gyroYaw = float(items[2]);
      
      x2 = x1;
      y2 = y1;
      z2 = z1;
      x1 = gyroCorrectedRoll = gyroCorrectedRoll * smooth +  (1-smooth)* float(items[3]);
      y1 = gyroCorrectedPitch = gyroCorrectedPitch * smooth +  (1-smooth)*float(items[4]);
      z1 =gyroCorrectedYaw = gyroCorrectedYaw * smooth +  (1-smooth)*float(items[5]);
  }
  }
}
