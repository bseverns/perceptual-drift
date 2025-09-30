#include <Arduino.h>
#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <AudioDelayFeedback.h>

Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> osc(SIN2048_DATA);
AudioDelayFeedback<128> delay;

void setup(){
  startMozzi();
  osc.setFreq(220);
}

void updateControl(){
  // could map OSC or analog in to delay time, feedback
  delay.setDelayTime(64);
  delay.setFeedbackLevel(200);
}

int updateAudio(){
  int s = osc.next();
  return delay.next(s);
}

void loop(){
  audioHook();
}
