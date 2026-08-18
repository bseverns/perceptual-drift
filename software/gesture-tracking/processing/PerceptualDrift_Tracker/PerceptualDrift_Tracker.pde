// -----------------------------------------------------------------------------
// PerceptualDrift_Tracker — Processing sketch
// Webcam-based coarse gesture → OSC
// Sends: /pd/alt, /pd/lat, /pd/yaw, /pd/crowd, /pd/consent,
//        /pd/consent_count
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

// -----------------------------------------------------------------------------
// TUNABLE PARAMETERS — tweak up here so the rest of the sketch stays readable.
// -----------------------------------------------------------------------------
int threshold = 35;          // frame-diff brightness cutoff; lower = more sensitive
int blurRadius = 0;          // optional blur applied to the working frame (0 = off)
int sampleStride = 8;        // pixel sampling step when estimating the centroid
int minBlobSamples = 12;     // reject isolated foreground/noise in the sampled mask
int consentEnterFrames = 3;  // consecutive fresh frames required to opt in
int cameraTimeoutMs = 500;   // force safe output after this long without a fresh frame

// Set either value to pin an overhead camera. Index wins when both are set.
// Leave the index at -1 and substring empty to use the best non-built-in camera.
int preferredCameraIndex = -1;
String preferredCameraContains = "";

// Consent zone in screen coordinates (top-down camera view)
// Rectangular by default; tweak ratios to reshape without hunting through code.
float consentXRatio = 0.25;
float consentYRatio = 0.25;
float consentWRatio = 0.5;
float consentHRatio = 0.5;

// Opt-in is frame-debounced; opt-out uses wall-clock time so camera FPS cannot
// stretch the grace period.
int consentHoldMs = 350;

// -----------------------------------------------------------------------------
// END TUNABLES
// -----------------------------------------------------------------------------

Capture cam;
OscP5 osc;
NetAddress dst;

float alt=0, lat=0, yaw=0, crowd=0;
float liveAlt=0, liveLat=0, liveYaw=0, liveCrowd=0; // preview keeps updating while OSC is safely neutral
boolean consent=false;
boolean consentArmed=false; // fail closed; facilitator must press space to arm
boolean consentZoneActive=false;
int consentState = 0; // 0 = no one in zone, 1 = at least one centroid in bounds
int lastConsentState = 0;
int lastConsentChangeFrame = 0;
PImage prev;
PImage baseline;
PImage presenceDiff;
boolean calibrationMode = false; // toggled with 'c'; sends neutral OSC while live previews keep updating
boolean havePrevFrame = false;
boolean haveBaseline = false;
boolean backgroundRefreshRequested = false;
int lastFreshCameraFrameMillis = 0;
int lastSafetyBroadcastMillis = -1000;
boolean cameraStalled = false;

float consentX, consentY, consentW, consentH;
int consentReleaseDeadlineMillis = 0;
int consentCandidateFrames = 0;

ArrayList<PVector> centroids = new ArrayList<PVector>();
ArrayList<Integer> centroidWeights = new ArrayList<Integer>();
boolean[] sampleActive;
boolean[] sampleVisited;
int[] floodQueue;
int sampleCols;
int sampleRows;
int lastConsentCountSent = -1;

void setup(){
  size(960,540);
  pixelDensity(1);
  updateConsentZoneDimensions();
  osc = new OscP5(this, 0);
  dst = new NetAddress("127.0.0.1", 9000);
  String[] cams = Capture.list();
  println("Processing video camera scan:");
  for (int i = 0; i < cams.length; i++){
    println("  [" + i + "] " + cams[i]);
  }
  if (cams.length == 0){
    println("No cameras found. Connect a camera or use the synthetic tracker path.");
    println("On macOS, also check System Settings -> Privacy & Security -> Camera and allow Processing.");
    return;
  }
  String selectedCamera = pickPreferredCamera(cams);
  println("Using camera: " + selectedCamera);
  cam = new Capture(this, selectedCamera);
  cam.start();
  prev = createImage(width, height, RGB);
  baseline = createImage(width, height, RGB);
  presenceDiff = createImage(width, height, RGB);
  sampleCols = (width + sampleStride - 1) / sampleStride;
  sampleRows = (height + sampleStride - 1) / sampleStride;
  int sampleCount = sampleCols * sampleRows;
  sampleActive = new boolean[sampleCount];
  sampleVisited = new boolean[sampleCount];
  floodQueue = new int[sampleCount];
  lastFreshCameraFrameMillis = millis();
}

String pickPreferredCamera(String[] cams){
  if (preferredCameraIndex >= 0 && preferredCameraIndex < cams.length){
    return cams[preferredCameraIndex];
  }
  String requested = preferredCameraContains.trim().toLowerCase();
  if (requested.length() > 0){
    for (int i = 0; i < cams.length; i++){
      if (cams[i].toLowerCase().indexOf(requested) != -1) return cams[i];
    }
    println("Configured camera substring not found: " + preferredCameraContains);
  }
  int bestIndex = 0;
  int bestScore = -9999;
  for (int i = 0; i < cams.length; i++){
    String entry = cams[i].toLowerCase();
    int score = 0;
    if (entry.indexOf("external") != -1) score += 100;
    if (entry.indexOf("usb") != -1) score += 80;
    if (entry.indexOf("facetime") != -1) score -= 100;
    if (entry.indexOf("built") != -1) score -= 80;
    if (entry.indexOf("default") != -1) score += 10;
    if (entry.indexOf("640x480") != -1) score += 20;
    if (entry.indexOf("1280x720") != -1) score += 15;
    if (entry.indexOf("fps=30") != -1) score += 5;
    if (entry.indexOf("320x240") != -1) score -= 10;
    if (score > bestScore){
      bestScore = score;
      bestIndex = i;
    }
  }
  return cams[bestIndex];
}

void draw(){
  if (cam == null){
    background(0);
    handleCameraStall();
    drawStartupMessage("No camera available. Safe output active; restart after connecting one.");
    return;
  }
  boolean freshFrame = cam.available();
  if (!freshFrame){
    if (refreshConsentDeadline(millis())){
      lastConsentCountSent = 0;
      sendConsentState();
    }
    if (millis() - lastFreshCameraFrameMillis >= cameraTimeoutMs){
      background(0);
      handleCameraStall();
      drawStartupMessage("Camera stream stalled. Safe output active; waiting for fresh frames...");
    }
    return;
  }
  cam.read();
  lastFreshCameraFrameMillis = millis();
  if (cameraStalled){
    cameraStalled = false;
    havePrevFrame = false;
    println("Camera stream resumed; re-priming frame differencing.");
  }
  background(0);
  if (cam.width <= 0 || cam.height <= 0){
    drawStartupMessage("Waiting for camera stream...");
    handleCameraStall();
    return;
  }
  PImage working = cam.get();
  if (working == null){
    drawStartupMessage("Waiting for first camera frame...");
    return;
  }
  working.resize(width, height);
  if (blurRadius > 0){
    // Soften sensor noise when tuning threshold; this is intentionally optional.
    working.filter(BLUR, blurRadius);
  }
  image(working, 0,0, width, height);
  if (!haveBaseline || backgroundRefreshRequested){
    captureBackground(working, backgroundRefreshRequested? "manual refresh" : "startup");
    drawStartupMessage("Background captured. Press 'b' to recalibrate when the space is empty.");
    return;
  }

  if (!havePrevFrame){
    prev.copy(working, 0,0, working.width, working.height, 0,0, width, height);
    havePrevFrame = true;
    drawStartupMessage("Camera online. Priming motion baseline...");
    return;
  }

  // Hybrid tracker:
  // - baseline diff (current vs startup/background frame) => presence + centroids
  // - frame diff (current vs previous frame) => motion intensity / crowd value
  working.loadPixels();
  prev.loadPixels();
  baseline.loadPixels();
  presenceDiff.loadPixels();
  int pixelCount = min(
    working.pixels.length,
    min(
      prev.pixels.length,
      min(baseline.pixels.length, presenceDiff.pixels.length)
    )
  );
  int motionCount=0;
  int roiPixelCount=0;
  for (int i=0; i<pixelCount; i++){
    int pixelX = i % width;
    int pixelY = i / width;
    boolean inConsentROI = pointInConsentZone(pixelX, pixelY);
    if (inConsentROI) roiPixelCount++;
    color c1 = working.pixels[i];
    color cPrev = prev.pixels[i];
    color cBase = baseline.pixels[i];
    float presenceDelta = abs(brightness(c1) - brightness(cBase));
    float motionDelta = abs(brightness(c1) - brightness(cPrev));
    if (presenceDelta > threshold){
      presenceDiff.pixels[i] = color(255);
    } else {
      presenceDiff.pixels[i] = color(0);
    }
    // Only explicit participants may shape control intent. The full-frame
    // presence mask remains visible for calibration, but never enters the
    // lateral/altitude/yaw/crowd calculations outside the marked ROI.
    if (inConsentROI && motionDelta > threshold){
      motionCount++;
    }
  }
  presenceDiff.updatePixels();
  tint(255, 100);
  image(presenceDiff, 0,0);
  noTint();

  // Connected components on a sampled presence mask reject isolated noise and
  // preserve one centroid per participant-sized blob.
  findPresenceBlobs();
  float cx=0, cy=0;
  int activeSamples = 0;
  for (int i = 0; i < centroids.size(); i++){
    int weight = centroidWeights.get(i);
    cx += centroids.get(i).x * weight;
    cy += centroids.get(i).y * weight;
    activeSamples += weight;
  }
  float normX = 0;
  float normY = 0;
  if (activeSamples > 0){
    cx /= activeSamples;
    cy /= activeSamples;
    float avgNormX = (cx - consentX) / max(1.0, consentW);
    float avgNormY = (cy - consentY) / max(1.0, consentH);
    normX = map(avgNormX, 0, 1, -1, 1);
    normY = map(avgNormY, 0, 1, -1, 1);
  }

  liveLat = constrain(normX, -1, 1);
  liveAlt = constrain(-normY, -1, 1);
  liveYaw = constrain(liveLat*0.2, -1, 1);
  liveCrowd = constrain(motionCount / float(max(1, roiPixelCount)) * 5.0, 0, 1);

  if (!calibrationMode){
    lat = liveLat;
    alt = liveAlt;
    yaw = liveYaw;
    crowd = liveCrowd;
  }

  // consent zoning — count bodies inside the opt-in rectangle and debounce
  int consentCount = centroids.size();
  int consentInt = consentCount>0?1:0;
  if (consentInt != consentState){
    lastConsentState = consentState;
    consentState = consentInt;
    lastConsentChangeFrame = frameCount;
  }
  updateConsentState(consentCount);

  if (!calibrationMode){
    sendOSC("/pd/lat", lat);
    sendOSC("/pd/alt", alt);
    sendOSC("/pd/yaw", yaw);
    sendOSC("/pd/crowd", crowd);
    sendConsentState();
  } else {
    sendSafeOSCFrame();
  }

  // HUD overlay helps with calibration / explaining to the crew what the drone
  // currently "feels" from the crowd.  Green bar = consent granted.
  drawConsentZoneOverlay();
  noStroke(); fill(consent? color(0,255,0): color(255,0,0));
  rect(0, height-10, width*(consent?1:0.25), 10);
  drawHUD();

  if (calibrationMode){
    drawCalibrationOverlay();
  }

  // Consent overlay uses the final gated state, so entry/exit debounce,
  // calibration, and facilitator pauses cannot contradict the OSC output.
  drawConsentRestingOverlay();

  prev.copy(working, 0,0, working.width, working.height, 0,0, width, height);
}

void findPresenceBlobs(){
  centroids.clear();
  centroidWeights.clear();
  for (int row = 0; row < sampleRows; row++){
    int y = row * sampleStride;
    for (int col = 0; col < sampleCols; col++){
      int x = col * sampleStride;
      int sampleIndex = row * sampleCols + col;
      sampleVisited[sampleIndex] = false;
      sampleActive[sampleIndex] = x < width && y < height &&
        pointInConsentZone(x, y) &&
        brightness(presenceDiff.pixels[y * width + x]) > 128;
    }
  }

  for (int start = 0; start < sampleActive.length; start++){
    if (!sampleActive[start] || sampleVisited[start]) continue;
    int head = 0;
    int tail = 0;
    floodQueue[tail++] = start;
    sampleVisited[start] = true;
    float sumX = 0;
    float sumY = 0;
    int blobSamples = 0;

    while (head < tail){
      int current = floodQueue[head++];
      int row = current / sampleCols;
      int col = current % sampleCols;
      sumX += col * sampleStride;
      sumY += row * sampleStride;
      blobSamples++;

      for (int rowOffset = -1; rowOffset <= 1; rowOffset++){
        for (int colOffset = -1; colOffset <= 1; colOffset++){
          if (rowOffset == 0 && colOffset == 0) continue;
          int neighborRow = row + rowOffset;
          int neighborCol = col + colOffset;
          if (neighborRow < 0 || neighborRow >= sampleRows ||
              neighborCol < 0 || neighborCol >= sampleCols) continue;
          int neighbor = neighborRow * sampleCols + neighborCol;
          if (sampleActive[neighbor] && !sampleVisited[neighbor]){
            sampleVisited[neighbor] = true;
            floodQueue[tail++] = neighbor;
          }
        }
      }
    }

    if (blobSamples >= minBlobSamples){
      centroids.add(new PVector(sumX / blobSamples, sumY / blobSamples));
      centroidWeights.add(blobSamples);
    }
  }
}

void drawStartupMessage(String msg){
  noStroke();
  fill(0, 0, 0, 180);
  rect(0, 0, width, height);
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(20);
  text(msg, width / 2.0, height / 2.0);
}

void captureBackground(PImage frame, String reason){
  baseline.copy(frame, 0,0, frame.width, frame.height, 0,0, width, height);
  prev.copy(frame, 0,0, frame.width, frame.height, 0,0, width, height);
  haveBaseline = true;
  havePrevFrame = true;
  backgroundRefreshRequested = false;
  centroids.clear();
  centroidWeights.clear();
  liveAlt = 0;
  liveLat = 0;
  liveYaw = 0;
  liveCrowd = 0;
  alt = 0;
  lat = 0;
  yaw = 0;
  crowd = 0;
  consentState = 0;
  consentArmed = false;
  consentZoneActive = false;
  consentReleaseDeadlineMillis = 0;
  consentCandidateFrames = 0;
  updateConsentState(0);
  println("Captured background baseline (" + reason + ").");
}

// consent toggle — space bar flips on/off.  Consider mapping this to a foot
// switch or physical button in the gallery so the facilitator controls arming.
// If you need to remap the arming key, this is the block to edit.
// New calibration helpers:
//   c   -> send safe neutral OSC while showing live tuning previews
//   b   -> capture a new empty-room background baseline
//   [/] -> threshold down/up
//   -/= -> blur radius down/up
//   j/l/i/k/u/o/n/m -> reposition/resize consent zone
//   s   -> print a config snippet you can paste into the TUNABLES block
void keyReleased(){
  if (key == ' '){
    if (calibrationMode || cameraStalled){
      consentArmed = false;
      println("Cannot arm while calibration or camera-safe mode is active.");
    } else {
      consentArmed = !consentArmed;
    }
    // Recompute state so the HUD and OSC reflect the arming gate immediately.
    refreshConsentGate();
  } else if (key == 'c' || key == 'C'){
    calibrationMode = !calibrationMode;
    if (calibrationMode){
      consentArmed = false;
      // Hold the last values for the HUD while OSC publishes a neutral heartbeat.
      lat = liveLat;
      alt = liveAlt;
      yaw = liveYaw;
      crowd = liveCrowd;
    }
    refreshConsentGate();
    if (calibrationMode) sendSafeOSCFrame();
  } else if (key == 'b' || key == 'B'){
    backgroundRefreshRequested = true;
  } else if (key == '['){
    threshold = max(0, threshold-1);
  } else if (key == ']'){
    threshold = min(255, threshold+1);
  } else if (key == '-'){ // blur down
    blurRadius = max(0, blurRadius-1);
  } else if (key == '='){ // blur up (same as plus without shift)
    blurRadius = min(12, blurRadius+1);
  } else if (key == 'j' || key == 'J'){
    consentXRatio = constrain(consentXRatio-0.01, 0, 1);
    updateConsentZoneDimensions();
  } else if (key == 'l' || key == 'L'){
    consentXRatio = constrain(consentXRatio+0.01, 0, 1);
    updateConsentZoneDimensions();
  } else if (key == 'i' || key == 'I'){
    consentYRatio = constrain(consentYRatio-0.01, 0, 1);
    updateConsentZoneDimensions();
  } else if (key == 'k' || key == 'K'){
    consentYRatio = constrain(consentYRatio+0.01, 0, 1);
    updateConsentZoneDimensions();
  } else if (key == 'u' || key == 'U'){
    consentWRatio = constrain(consentWRatio-0.01, 0.05, 1);
    updateConsentZoneDimensions();
  } else if (key == 'o' || key == 'O'){
    consentWRatio = constrain(consentWRatio+0.01, 0.05, 1);
    updateConsentZoneDimensions();
  } else if (key == 'n' || key == 'N'){
    consentHRatio = constrain(consentHRatio-0.01, 0.05, 1);
    updateConsentZoneDimensions();
  } else if (key == 'm' || key == 'M'){
    consentHRatio = constrain(consentHRatio+0.01, 0.05, 1);
    updateConsentZoneDimensions();
  } else if (key == 's' || key == 'S'){
    dumpConfigSnippet();
  }
}

void sendOSC(String addr, float v){
  OscMessage m = new OscMessage(addr);
  m.add(v);
  osc.send(m, dst);
}

void sendSafeOSCFrame(){
  sendOSC("/pd/lat", 0);
  sendOSC("/pd/alt", 0);
  sendOSC("/pd/yaw", 0);
  sendOSC("/pd/crowd", 0);
  sendOSC("/pd/consent", 0);
  sendOSC("/pd/consent_count", 0);
}

void handleCameraStall(){
  cameraStalled = true;
  consentArmed = false;
  centroids.clear();
  centroidWeights.clear();
  liveAlt = liveLat = liveYaw = liveCrowd = 0;
  alt = lat = yaw = crowd = 0;
  consentState = 0;
  lastConsentCountSent = 0;
  consentCandidateFrames = 0;
  consentReleaseDeadlineMillis = 0;
  consentZoneActive = false;
  refreshConsentGate();
  if (millis() - lastSafetyBroadcastMillis >= 250){
    sendSafeOSCFrame();
    lastSafetyBroadcastMillis = millis();
  }
}

void updateConsentZoneDimensions(){
  consentWRatio = constrain(consentWRatio, 0.05, 1.0);
  consentHRatio = constrain(consentHRatio, 0.05, 1.0);
  consentXRatio = constrain(consentXRatio, 0, 1.0-consentWRatio);
  consentYRatio = constrain(consentYRatio, 0, 1.0-consentHRatio);
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
  int now = millis();
  if (consentCount > 0){
    consentCandidateFrames = min(consentEnterFrames, consentCandidateFrames + 1);
    if (consentCandidateFrames >= consentEnterFrames){
      consentReleaseDeadlineMillis = now + max(1, consentHoldMs);
    }
  } else {
    consentCandidateFrames = 0;
  }

  refreshConsentDeadline(now);
  lastConsentCountSent = consentCount;
}

boolean refreshConsentDeadline(int now){
  boolean wasActive = consentZoneActive;
  // Subtraction keeps this comparison safe when millis() wraps, provided the
  // configured hold remains below half the integer range.
  consentZoneActive = consentReleaseDeadlineMillis != 0 &&
                      consentReleaseDeadlineMillis - now > 0;
  if (wasActive && !consentZoneActive){
    consentCandidateFrames = 0;
  }
  refreshConsentGate();
  return wasActive && !consentZoneActive;
}

void refreshConsentGate(){
  boolean newConsent = consentArmed && !calibrationMode && !cameraStalled && consentZoneActive;
  if (newConsent != consent){
    consent = newConsent;
  }
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
  textAlign(LEFT, TOP);
  textSize(14);
  String msg = "Step inside the zone to participate\n" +
               "Consent: " + (consent?"ON":"OFF") +
               (consentArmed?"":" (paused by facilitator)");
  text(msg, 16, height - 40);
  if (calibrationMode){
    fill(255, 180, 0);
    text("Calibration safe-output mode ON", consentX + 8, consentY + 18);
  }
}

void drawConsentRestingOverlay(){
  if (consent) return;

  String msg;
  if (calibrationMode){
    msg = "Calibration mode.\nSafe neutral output is active.";
  } else if (!consentArmed){
    msg = "Participation paused by facilitator.\nSafe neutral output is active.";
  } else if (lastConsentCountSent > 0){
    msg = "Presence detected. Hold position briefly to opt in.\nSystem remains resting until detection is stable.";
  } else {
    msg = "No one has opted in yet.\nSystem resting until someone steps into the zone.";
  }

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

// HUD: numeric readout of both frozen (OSC) and live values so you can see how
// tweaking the filters affects the math.  Stays on-screen during normal ops and
// calibration alike.
void drawHUD(){
  int hudW = 320;
  int hudH = 150;
  int hudX = 10;
  int hudY = 10;

  noStroke();
  fill(0, 0, 0, 160);
  rect(hudX, hudY, hudW, hudH, 8);

  fill(255);
  textAlign(LEFT, TOP);
  textSize(14);
  String frozenLabel = calibrationMode? "(safe neutral OSC)" : "(live OSC)";
  text("OSC → " + frozenLabel, hudX + 10, hudY + 8);
  text(String.format("lat %.2f  alt %.2f  yaw %.2f  crowd %.2f", lat, alt, yaw, crowd), hudX + 10, hudY + 28);
  text("Live (preview only)", hudX + 10, hudY + 52);
  text(String.format("lat %.2f  alt %.2f  yaw %.2f  crowd %.2f", liveLat, liveAlt, liveYaw, liveCrowd), hudX + 10, hudY + 72);

  String consentMsg = "Consent: " + (consent?"1 (armed)":"0") + (consentArmed?"":" (paused)");
  text(consentMsg, hudX + 10, hudY + 96);
  String baselineMsg = "Presence baseline: " + (haveBaseline?"ready":"capturing") + (backgroundRefreshRequested?" (refresh queued)":"");
  text(baselineMsg, hudX + 10, hudY + 118);
}

void drawCalibrationOverlay(){
  int overlayW = 420;
  int overlayH = 170;
  int overlayX = width - overlayW - 12;
  int overlayY = 12;

  noStroke();
  fill(20, 0, 0, 170);
  rect(overlayX, overlayY, overlayW, overlayH, 10);

  fill(255, 220, 160);
  textAlign(LEFT, TOP);
  textSize(14);
  text("Calibration mode (neutral OSC heartbeat active).", overlayX + 12, overlayY + 10);
  text("b = capture empty-room baseline  •  [ / ] threshold ±1  •  -/= blur ±1", overlayX + 12, overlayY + 32);
  text("Move ROI: j/l ←→   i/k ↑↓   Resize: u/o width  n/m height  •  s = dump config", overlayX + 12, overlayY + 52);

  String snippet = buildConfigSnippet();
  text("Config snippet (copy/paste):", overlayX + 12, overlayY + 76);
  text(snippet, overlayX + 12, overlayY + 96);
}

String buildConfigSnippet(){
  String format = "threshold=%d, blur=%d, stride=%d, minBlob=%d, " +
                  "enterFrames=%d, holdMs=%d, consent=(x:%.2f y:%.2f w:%.2f h:%.2f)";
  return String.format(format,
                      threshold, blurRadius, sampleStride, minBlobSamples,
                      consentEnterFrames, consentHoldMs,
                      consentXRatio, consentYRatio, consentWRatio, consentHRatio);
}

void dumpConfigSnippet(){
  String snippet = buildConfigSnippet();
  println("--- PerceptualDrift Tracker config block ---");
  println(snippet);
  println("Copy into the TUNABLE PARAMETERS block up top.");
}

void sendConsentState(){
  int consentInt = consent?1:0;
  // Consent is a heartbeat, not an edge-only event: a dropped UDP packet must
  // never leave a receiver indefinitely armed with an old state.
  sendOSC("/pd/consent", consentInt);
  sendOSC("/pd/consent_count", lastConsentCountSent);
}
