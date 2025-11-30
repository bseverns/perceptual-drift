// -----------------------------------------------------------------------------
// PerceptualDrift_Tracker — Processing sketch
// Webcam-based coarse gesture → OSC
// Sends: /pd/alt, /pd/lat, /pd/yaw, /pd/crowd, /pd/consent
//
// Camera is mounted overhead (or steeply downward) looking at the floor.  The
// drawn consent zone is an explicit participation boundary: stepping into the
// region signals opt-in; stepping out is opt-out.  Keep it visible so the crew
// can explain the ethics at a glance.  A bottom-of-screen overlay now also
// explains that the system is “resting” when nobody has stepped in.
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
boolean consentArmed=true; // facilitator arming; space bar toggles this gate
boolean consentZoneActive=false;
int consentState = 0; // 0 = no one in zone, 1 = at least one centroid in bounds
int lastConsentState = 0;
int lastConsentChangeFrame = 0;
boolean consentEnabled = true; // flip false if running in a legacy setup without consent signals
int threshold = 35;
PImage prev;

// Consent zone in screen coordinates (top-down camera view)
// Rectangular by default; tweak ratios to reshape without hunting through code.
float consentXRatio = 0.25;
float consentYRatio = 0.25;
float consentWRatio = 0.5;
float consentHRatio = 0.5;
float consentX, consentY, consentW, consentH;

// Debounce so brushing the edge does not flicker consent.
int consentHoldFrames = 10;
int consentFramesRemaining = 0;

ArrayList<PVector> centroids = new ArrayList<PVector>();
int lastConsentSent = -1;
int lastConsentCountSent = -1;
int lastBroadcastConsentCount = -1;

void setup(){
  size(960,540);
  updateConsentZoneDimensions();
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

  // centroid for lateral mapping — we average the active sample positions so the
  // "center of motion" follows the crowd instead of the raw sum of pixels.
  centroids.clear();
  float cx=0, cy=0;
  int activeSamples = 0;
  for (int y=0; y<height; y+=8){
    for (int x=0; x<width; x+=8){
      if (brightness(diff.get(x,y))>128){
        cx += x;
        cy += y;
        activeSamples++;
      }
    }
  }
  float normX = 0;
  float normY = 0;
  if (activeSamples > 0){
    cx /= activeSamples;
    cy /= activeSamples;
    centroids.add(new PVector(cx, cy));
    float avgNormX = cx / float(width);
    float avgNormY = cy / float(height);
    normX = map(avgNormX, 0, 1, -1, 1);
    normY = map(avgNormY, 0, 1, -1, 1);
  }

  lat = constrain(normX, -1, 1);
  alt = constrain(-normY, -1, 1);
  yaw = constrain(lat*0.2, -1, 1);
  crowd = constrain(motionCount / float(pixels.length) * 5.0, 0, 1);

  // consent zoning — count bodies inside the opt-in rectangle and debounce
  int consentCount = 0;
  for (PVector c : centroids){
    if (pointInConsentZone(c.x, c.y)) consentCount++;
  }
  int consentInt = consentCount>0?1:0;
  if (consentInt != consentState){
    lastConsentState = consentState;
    consentState = consentInt;
    lastConsentChangeFrame = frameCount;
  }
  updateConsentState(consentCount);

  sendOSC("/pd/lat", lat);
  sendOSC("/pd/alt", alt);
  sendOSC("/pd/yaw", yaw);
  sendOSC("/pd/crowd", crowd);
  sendConsentIfChanged();

  // HUD overlay helps with calibration / explaining to the crew what the drone
  // currently "feels" from the crowd.  Green bar = consent granted.
  drawConsentZoneOverlay();
  noStroke(); fill(consent? color(0,255,0): color(255,0,0));
  rect(0, height-10, width*(consent?1:0.25), 10);
  fill(255);
  text(String.format("lat %.2f alt %.2f yaw %.2f crowd %.2f consent %s", lat, alt, yaw, crowd, consent), 10, 20);

  // Consent overlay:
  // When no one is inside the consent zone (consentState == 0), we draw a
  // message to make the system's "resting" state visible. The camera feed and
  // zone remain visible underneath so facilitators can still narrate what the
  // system sees.
  drawConsentRestingOverlay();

  prev.copy(cam, 0,0, cam.width, cam.height, 0,0, width, height);
}

// consent toggle — space bar flips on/off.  Consider mapping this to a foot
// switch or physical button in the gallery so the facilitator controls arming.
// If you need to remap the arming key, this is the block to edit.
void keyReleased(){
  if (key == ' '){
    consentArmed = !consentArmed;
    // Recompute state so the HUD and OSC reflect the arming gate immediately.
    updateConsentState(consentZoneActive?1:0);
  }
}

void sendOSC(String addr, float v){
  OscMessage m = new OscMessage(addr);
  m.add(v);
  osc.send(m, dst);
}

void updateConsentZoneDimensions(){
  consentX = width * consentXRatio;
  consentY = height * consentYRatio;
  consentW = width * consentWRatio;
  consentH = height * consentHRatio;
}

boolean pointInConsentZone(float x, float y){
  return x >= consentX && x <= consentX + consentW &&
         y >= consentY && y <= consentY + consentH;
}

void updateConsentState(int consentCount){
  if (consentCount > 0){
    consentFramesRemaining = consentHoldFrames;
  } else if (consentFramesRemaining > 0){
    consentFramesRemaining--;
  }

  consentZoneActive = consentFramesRemaining > 0;
  boolean newConsent = consentArmed && consentZoneActive;
  if (newConsent != consent){
    consent = newConsent;
  }
  lastConsentCountSent = consentCount;
}

void drawConsentZoneOverlay(){
  // Visualize the zone — transparent gray when idle, soft green when active.
  noStroke();
  if (consentZoneActive){
    fill(0, 255, 0, 80);
  } else {
    fill(180, 180, 180, 60);
  }
  rect(consentX, consentY, consentW, consentH);

  fill(255);
  textSize(14);
  String msg = "Step inside the zone to participate\n" +
               "Consent: " + (consent?"ON":"OFF") +
               (consentArmed?"":" (paused by facilitator)");
  text(msg, 16, height - 40);
}

void drawConsentRestingOverlay(){
  if (!consentEnabled || consentState != 0) return;

  String msg = "No one has opted in yet.\nSystem resting until someone steps into the zone.";

  float boxW = width * 0.7;
  float boxH = 70;
  float boxX = (width - boxW) / 2.0;
  float boxY = height - boxH - 24;

  // Gentle pulse so the overlay feels alive but not shouty.
  float pulse = 0.5 + 0.5 * sin(frameCount * 0.05);
  int bgAlpha = int(100 + 80 * pulse);
  int textAlpha = int(200 + 40 * pulse);

  noStroke();
  fill(0, 0, 0, bgAlpha);
  rect(boxX, boxY, boxW, boxH, 8);

  fill(255, 255, 255, textAlpha);
  textAlign(CENTER, CENTER);
  textSize(16);
  text(msg, boxX + boxW / 2.0, boxY + boxH / 2.0);
}

void sendConsentIfChanged(){
  int consentInt = consent?1:0;
  if (consentInt != lastConsentSent || lastConsentCountSent != lastBroadcastConsentCount){
    sendOSC("/pd/consent", consentInt);
    // Also share how many centroids are in-bounds so downstream systems can
    // interpret group size or ratios if they care.
    sendOSC("/pd/consent_count", lastConsentCountSent);
    lastConsentSent = consentInt;
    lastBroadcastConsentCount = lastConsentCountSent;
  }
}
