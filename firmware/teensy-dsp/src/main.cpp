#include <Arduino.h>
#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <AudioDelayFeedback.h>

// -----------------------------------------------------------------------------
// Minimal Mozzi patch to prove the DSP lane works.
// The idea: drone installations get weird fast when you let the airframe mic or
// performer inputs feed back through delay lines.  This sketch is the seed you
// can graft onto.  Hook analog inputs or OSC serial commands in ``updateControl``
// to modulate delay time / feedback.
//
// Mozzi docs: https://sensorium.github.io/Mozzi/
// Example inspiration: [Look Mum No Computer’s drone choirs]
// (https://www.lookmumnocomputer.com/projects).
// -----------------------------------------------------------------------------

Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> osc(SIN2048_DATA);  // sine oscillator
AudioDelayFeedback<128> delay;  // 128 sample (~2.9 ms @ 44.1 kHz) buffer

void setup() {
  startMozzi();
  osc.setFreq(220);  // A3 droning away. Swap for sensors later.
}

void updateControl() {
  // Placeholder control loop.  Map OSC/analog into these for live modulation.
  delay.setDelayTime(64);        // Out of 128 samples total (~1.45 ms).
  delay.setFeedbackLevel(200);   // 0..255.  Keep <220 to avoid runaway squeals.
}

int updateAudio() {
  int sample = osc.next();
  return delay.next(sample);  // Feed the oscillator through the feedback delay.
}

void loop() {
  audioHook();  // Mozzi’s scheduler.  Keep this as the only thing in loop().
}
