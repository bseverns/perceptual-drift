// -----------------------------------------------------------------------------
// PerceptualDrift_Tracker — Processing sketch
// Webcam-based coarse gesture → OSC
// Sends: /pd/alt, /pd/lat, /pd/yaw, /pd/crowd, /pd/consent
//
// Required libraries (install via Sketch → Import Library... → Add Library):
// * video (Processing core) — https://processing.org/reference/libraries/video/
// * oscP5 — https://www.sojamo.de/libraries/oscP5/
// * netP5 — ships with oscP5
//
// Inspiration: Kyle McDonald’s ofxCv optical flow experiments and the cult-classic
// Golan Levin crowd interaction sketches.  This version purposely keeps the math
// simple so you can explain it to safety officers in a pinch.
// -----------------------------------------------------------------------------

import processing.video.*;
import oscP5.*;
import netP5.*;

Capture cam;
OscP5 osc;
NetAddress dst;

float alt=0, lat=0, yaw=0, crowd=0;
boolean consent=false;
int threshold = 35;
PImage prev;

void setup(){
  size(960,540);
  String[] cams = Capture.list();
  // Pick the first camera unless you override in code.  Replace with an index
  // or name from ``cams`` if you have multiple USB cameras attached.
  cam = new Capture(this, cams.length>0? cams[0]: 0);
  cam.start();
  osc = new OscP5(this, 0);
  dst = new NetAddress("127.0.0.1", 9000);
  prev = createImage(width, height, RGB);
}

void draw(){
  background(0);
  if (cam.available()) cam.read();
  image(cam, 0,0, width, height);

  // optical flow proxy: frame diff magnitude.  We compare the current frame to
  // the previous frame and count pixels above ``threshold`` as "motion".
  PImage diff = createImage(width, height, RGB);
  cam.loadPixels(); prev.loadPixels(); diff.loadPixels();
  int motionCount=0;
  for (int i=0; i<pixels.length; i++){
    color c1 = cam.pixels[i];
    color c0 = prev.pixels[i];
    float d = abs(brightness(c1) - brightness(c0));
    if (d > threshold){
      diff.pixels[i] = color(255);
      motionCount++;
    } else diff.pixels[i] = color(0);
  }
  diff.updatePixels();
  tint(255, 100);
  image(diff, 0,0);

  // centroid for lateral mapping
  float cx=0, cy=0;
  for (int y=0; y<height; y+=8){
    for (int x=0; x<width; x+=8){
      if (brightness(diff.get(x,y))>128){ cx+=x; cy+=y; }
    }
  }
  float normX = map(cx, 0, (width*height)/64.0, -1, 1);
  float normY = map(cy, 0, (width*height)/64.0, -1, 1);

  lat = constrain(normX, -1, 1);
  alt = constrain(-normY, -1, 1);
  yaw = constrain(lat*0.2, -1, 1);
  crowd = constrain(motionCount / float(pixels.length) * 5.0, 0, 1);

  sendOSC("/pd/lat", lat);
  sendOSC("/pd/alt", alt);
  sendOSC("/pd/yaw", yaw);
  sendOSC("/pd/crowd", crowd);
  sendOSC("/pd/consent", consent?1:0);

  // HUD overlay helps with calibration / explaining to the crew what the drone
  // currently "feels" from the crowd.  Green bar = consent granted.
  noStroke(); fill(consent? color(0,255,0): color(255,0,0));
  rect(0, height-10, width*(consent?1:0.25), 10);
  fill(255);
  text(String.format("lat %.2f alt %.2f yaw %.2f crowd %.2f consent %s", lat, alt, yaw, crowd, consent), 10, 20);

  prev.copy(cam, 0,0, cam.width, cam.height, 0,0, width, height);
}

// consent toggle — space bar flips on/off.  Consider mapping this to a foot
// switch or physical button in the gallery so the facilitator controls arming.
// If you need to remap the arming key, this is the block to edit.
void keyReleased(){
  if (key == ' ') consent = !consent;
}

void sendOSC(String addr, float v){
  OscMessage m = new OscMessage(addr);
  m.add(v);
  osc.send(m, dst);
}
