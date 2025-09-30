#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 12

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Simple serial protocol: "C r g b\n" sets color; "I x\n" sets intensity 0..255
uint8_t baseR=0, baseG=0, baseB=32;
uint8_t intensity=128;

void apply(){
  for (int i=0;i<LED_COUNT;i++){
    uint8_t r = (baseR * intensity) / 255;
    uint8_t g = (baseG * intensity) / 255;
    uint8_t b = (baseB * intensity) / 255;
    strip.setPixelColor(i, strip.Color(r,g,b));
  }
  strip.show();
}

void setup(){
  Serial.begin(115200);
  strip.begin();
  strip.show();
  apply();
}

void loop(){
  static String line;
  while (Serial.available()){
    char c = Serial.read();
    if (c=='\n'){
      if (line.length()>0){
        if (line[0]=='C'){
          int r,g,b;
          if (sscanf(line.c_str(), "C %d %d %d", &r,&g,&b)==3){
            baseR=r; baseG=g; baseB=b; apply();
          }
        } else if (line[0]=='I'){
          int x;
          if (sscanf(line.c_str(), "I %d", &x)==1){
            intensity = constrain(x,0,255); apply();
          }
        }
      }
      line="";
    } else {
      line+=c;
    }
  }
  delay(5);
}
