#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// -----------------------------------------------------------------------------
// Teensy LED companion for Perceptual Drift.
//
// Serial protocol (115200 baud, newline-terminated commands):
//   * "C r g b\n" sets the base RGB color (0–255 per channel).
//   * "I index\n" highlights one LED bank (index 0–15 maps into four corners).
//   * "D x\n" keeps the legacy global intensity dimmer (0–255) for smoke tests.
//   * "M a b c d\n" sets per-corner accent boosts (0–255 each) so the host can
//       translate drone motion into corner brightness without altering colors.
//
// The physical layout is four banks of four CRGB spots—one bank per cage
// corner.  Each bank is individually addressable so you can pulse corners as
// the drone leans or races around the space without changing the underlying
// palette.  All commands are kept intentionally tiny to stay friendly with
// Python/ROS bridges.
// -----------------------------------------------------------------------------

// Pin and pixel count: adjust to match your wiring harness.
constexpr uint8_t LED_PIN = 6;
constexpr uint8_t CORNER_COUNT = 4;
constexpr uint8_t SPOTS_PER_CORNER = 4;
constexpr uint8_t LED_COUNT = CORNER_COUNT * SPOTS_PER_CORNER;

// Corner accent boost applied when the drone leans toward a corner.  Kept
// modest to leave headroom for the global dimmer.
constexpr uint8_t HIGHLIGHT_BOOST = 120;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// State that the serial handlers mutate.  Defaults = gentle blue glow.
uint8_t baseR = 0;
uint8_t baseG = 0;
uint8_t baseB = 32;
uint8_t intensity = 128;
uint8_t cornerBoost[CORNER_COUNT] = {0, 0, 0, 0};

uint8_t cornerForIndex(uint8_t idx) {
  return idx / SPOTS_PER_CORNER;
}

void apply() {
  // Push the current color/intensity combo to every pixel.  You can replace
  // this with per-pixel effects if you want chase patterns.
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    const uint8_t corner = cornerForIndex(i);
    const uint16_t boosted = min<uint16_t>(255, intensity + cornerBoost[corner]);
    const uint8_t r = (baseR * boosted) / 255;
    const uint8_t g = (baseG * boosted) / 255;
    const uint8_t b = (baseB * boosted) / 255;
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void clearCornerBoosts() {
  for (uint8_t i = 0; i < CORNER_COUNT; i++) {
    cornerBoost[i] = 0;
  }
}

void handleCommand(const char *line) {
  if (line[0] == 'C') {
    int r, g, b;
    if (sscanf(line, "C %d %d %d", &r, &g, &b) == 3) {
      baseR = constrain(r, 0, 255);
      baseG = constrain(g, 0, 255);
      baseB = constrain(b, 0, 255);
      apply();
    }
  } else if (line[0] == 'D') {
    int x;
    if (sscanf(line, "D %d", &x) == 1) {
      intensity = constrain(x, 0, 255);
      apply();
    }
  } else if (line[0] == 'I') {
    int idx;
    if (sscanf(line, "I %d", &idx) == 1 && idx >= 0 && idx < LED_COUNT) {
      clearCornerBoosts();
      const uint8_t corner = cornerForIndex(static_cast<uint8_t>(idx));
      cornerBoost[corner] = HIGHLIGHT_BOOST;
      apply();
    }
  } else if (line[0] == 'M') {
    int a, b, c, d;
    if (sscanf(line, "M %d %d %d %d", &a, &b, &c, &d) == 4) {
      cornerBoost[0] = constrain(a, 0, 255);
      cornerBoost[1] = constrain(b, 0, 255);
      cornerBoost[2] = constrain(c, 0, 255);
      cornerBoost[3] = constrain(d, 0, 255);
      apply();
    }
  }
  // Unknown commands are intentionally ignored.
}

void setup() {
  Serial.begin(115200);  // Matches the Python bridge default.
  strip.begin();
  strip.show();  // Clear out RAM junk from startup.
  apply();
}

void loop() {
  static char line[64] = {0};
  static uint8_t linePos = 0;

  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (linePos > 0) {
        line[linePos] = '\0';
        handleCommand(line);
        linePos = 0;
        line[0] = '\0';
      }
    } else if (linePos < sizeof(line) - 1) {
      // Basic line buffer.  Good enough for short commands.  Swap for a ring
      // buffer if you start streaming complex CSV blobs.
      line[linePos++] = c;
    }
  }
  delay(5);  // Prevent serial starvation on older Teensy boards.
}
