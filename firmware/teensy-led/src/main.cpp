#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// -----------------------------------------------------------------------------
// Teensy LED companion for Perceptual Drift.
//
// Serial protocol (115200 baud, newline-terminated commands):
//   * "C r g b\n" sets the base RGB color (0–255 per channel).
//   * "I x\n" sets global intensity (0–255) acting like a dimmer.
// This intentionally mirrors the approach used by [Adafruit’s NeoPixel Uberguide]
// (https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-code) so you
// can cross reference animation ideas quickly.
//
// The control bridge or any laptop can blast commands over USB serial.  Because
// we separate "color" and "intensity" you can fade the strip without altering
// the palette selected by your show control.
// -----------------------------------------------------------------------------

// Pin and pixel count: adjust to match your wiring harness.
constexpr uint8_t LED_PIN = 6;
constexpr uint8_t LED_COUNT = 12;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// State that the serial handlers mutate.  Defaults = gentle blue glow.
uint8_t baseR = 0;
uint8_t baseG = 0;
uint8_t baseB = 32;
uint8_t intensity = 128;

void apply() {
  // Push the current color/intensity combo to every pixel.  You can replace
  // this with per-pixel effects if you want chase patterns.
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    uint8_t r = (baseR * intensity) / 255;
    uint8_t g = (baseG * intensity) / 255;
    uint8_t b = (baseB * intensity) / 255;
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void setup() {
  Serial.begin(115200);  // Matches the Python bridge default.
  strip.begin();
  strip.show();  // Clear out RAM junk from startup.
  apply();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (line.length() > 0) {
        if (line[0] == 'C') {
          int r, g, b;
          if (sscanf(line.c_str(), "C %d %d %d", &r, &g, &b) == 3) {
            baseR = constrain(r, 0, 255);
            baseG = constrain(g, 0, 255);
            baseB = constrain(b, 0, 255);
            apply();
          }
        } else if (line[0] == 'I') {
          int x;
          if (sscanf(line.c_str(), "I %d", &x) == 1) {
            intensity = constrain(x, 0, 255);
            apply();
          }
        }
      }
      line = "";
    } else {
      // Basic line buffer.  Good enough for short commands.  Swap for a ring
      // buffer if you start streaming complex CSV blobs.
      line += c;
    }
  }
  delay(5);  // Prevent serial starvation on older Teensy boards.
}
