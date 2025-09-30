// PerceptualDrift_Tracker — Processing sketch
// Webcam-based coarse gesture → OSC
// Sends: /pd/alt, /pd/lat, /pd/yaw, /pd/crowd, /pd/consent
//
// HOW TO READ THIS FILE (aka: no spelunking required):
// 1. Processing (https://processing.org/) wraps Java with friendlier syntax.
// 2. The Capture class (processing.video) slurps frames from any webcam.
// 3. oscP5 (http://www.sojamo.de/libraries/oscP5/) handles Open Sound Control networking.
// 4. We diff consecutive frames to get a "poor man's" motion detector, then map centroids into drone controls.
// 5. Everything is annotated so you can fork the logic or port it into TouchDesigner/Max/openFrameworks without guessing.
//
// Bonus references: check the Nature of Code "Optical Flow" chapter and CNMAT's OSC primer if you're new to this territory.

import processing.video.*; // Realtime camera input from Processing's video library
import oscP5.*;            // OSC networking helper (UDP under the hood)
import netP5.*;            // Barebones UDP sockets that oscP5 leans on

Capture cam;               // Webcam source
OscP5 osc;                 // OSC server/client for broadcasting motion data
NetAddress dst;            // Where to fling the OSC packets (host + port)

// Normalized control values that mirror the drone-side expectations
float alt=0, lat=0, yaw=0, crowd=0;
boolean consent=false;     // Soft-arm toggle (hit space to flip)
int threshold = 35;        // Brightness delta that counts as "movement" (tweak for your venue lighting)
PImage prev;               // Buffer to hold the previous frame for diffing

void setup(){
  size(960,540);                      // Render window. Adjust to match your camera aspect ratio.
  String[] cams = Capture.list();     // Grab every available camera device name.
  // Create the Capture object. If we have cameras, use the first entry; otherwise default to device index 0.
  cam = new Capture(this, cams.length>0? cams[0]: 0);
  cam.start();                        // Fire up the webcam stream.
  osc = new OscP5(this, 0);           // oscP5 needs a local port even if we only send. 0 = pick any open port.
  dst = new NetAddress("127.0.0.1", 9000); // OSC destination (bridge listens on localhost:9000 by default).
  prev = createImage(width, height, RGB);  // Allocate the frame buffer used for differencing.
}

void draw(){
  background(0);
  if (cam.available()) cam.read();    // Pull the next frame if the camera has one ready.
  image(cam, 0,0, width, height);     // Draw the raw camera feed for operator feedback.

  // --- MOTION ESTIMATION ---------------------------------------------------
  // We're using frame differencing instead of full optical flow because it's
  // cheap, stable enough for crowd gestures, and keeps CPU usage low on old laptops.
  PImage diff = createImage(width, height, RGB);
  cam.loadPixels(); prev.loadPixels(); diff.loadPixels();
  int motionCount=0;                  // How many pixels tripped the threshold this frame.
  for (int i=0; i<pixels.length; i++){
    color c1 = cam.pixels[i];         // Current frame pixel
    color c0 = prev.pixels[i];        // Previous frame pixel
    float d = abs(brightness(c1) - brightness(c0)); // Brightness delta gives a crude motion proxy
    if (d > threshold){
      diff.pixels[i] = color(255);    // White pixel in the diff image marks motion
      motionCount++;
    } else diff.pixels[i] = color(0); // Otherwise stay black
  }
  diff.updatePixels();
  tint(255, 100);                     // Ghost the diff overlay so the live feed remains visible
  image(diff, 0,0);                   // Overlay the diff image on top of the camera feed

  // --- CENTROID / CONTROL MAPPING -----------------------------------------
  // Scan the diff image in blocks to find a coarse centroid. Stepping by 8px keeps it light.
  float cx=0, cy=0;
  for (int y=0; y<height; y+=8){
    for (int x=0; x<width; x+=8){
      if (brightness(diff.get(x,y))>128){ cx+=x; cy+=y; }
    }
  }
  // Normalize the centroid to [-1, 1] ranges so they match what the mapper expects.
  float normX = map(cx, 0, (width*height)/64.0, -1, 1);
  float normY = map(cy, 0, (width*height)/64.0, -1, 1);

  lat = constrain(normX, -1, 1);      // Left/right roll
  alt = constrain(-normY, -1, 1);     // Invert Y so moving up increases altitude
  yaw = constrain(lat*0.2, -1, 1);    // Borrow a touch of lateral motion for yaw bias (makes turns feel organic)
  crowd = constrain(motionCount / float(pixels.length) * 5.0, 0, 1); // Motion density drives crowd energy

  // --- CONSENT GATE --------------------------------------------------------
  // Tap the space bar to toggle consent. Keep it off during rehearsals to park the drone.
  if (keyPressed && key==' ') consent = !consent;

  // --- OSC OUTPUT ----------------------------------------------------------
  // We broadcast every frame. Downstream code will throttle to saner rates.
  sendOSC("/pd/lat", lat);
  sendOSC("/pd/alt", alt);
  sendOSC("/pd/yaw", yaw);
  sendOSC("/pd/crowd", crowd);
  sendOSC("/pd/consent", consent?1:0);

  // --- HUD ----------------------------------------------------------------
  // UI overlay so operators can see the numbers without opening extra tools.
  noStroke();
  fill(consent? color(0,255,0): color(255,0,0));
  rect(0, height-10, width*(consent?1:0.25), 10); // Consent bar = full width when enabled, small red stub when off
  fill(255);
  text(String.format("lat %.2f alt %.2f yaw %.2f crowd %.2f consent %s", lat, alt, yaw, crowd, consent), 10, 20);

  // Copy the current frame into `prev` for the next loop.
  prev.copy(cam, 0,0, cam.width, cam.height, 0,0, width, height);
}

void sendOSC(String addr, float v){
  // Minimal helper so all OSC sends go through one spot.
  OscMessage m = new OscMessage(addr);
  m.add(v);                          // OSC arguments remain floats (Processing will coerce booleans anyway)
  osc.send(m, dst);
}
