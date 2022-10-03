// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);

#include <FastLED.h>
#define LED_PIN 7
#define NUM_LEDS 30 

bool Green1 = false; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BE SOLID GREEN
bool Green2 = false; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BLINK GREEN
bool Blue = false; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BE SOLID BLUE
bool Red = false; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BLINK RED


CRGBArray<NUM_LEDS> leds;

struct LEDS {
  uint32_t m_color;
  int m_state;
};

void setup(){
FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS);
FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // Power Failsafe
// Clears LEDs when code is updated
FastLED.clear();
FastLED.show();
}

void loop(){


// Determining when 
if(...){
  Green1 = true;
}

if(...){
  Green2 = true; 
}

if(...){
  Blue = true;
}

if(...){
  Red = true;
}

  FastLED.setBrightness(0);
}

/// Sets all leds to solid green when 'x'
for(int i = 0; i < NUM_LEDS; i++){
  while(Green1 == true){
  leds[i] = CRGB(0, 255, 0);
  FastLED.show();
  }
}


// Turn lights Flashing Green
for(int i = 0; i < NUM_LEDS; i++){ // Flash on
  while(Green2 == true){
  leds[i] = CRGB(0, 255, 0);
  FastLED.setBrightness(2*i);
  FastLED.show();
  delay(100);
  }
}
for(int i = NUM_LEDS; i > 0; i--){ // Flash off
  while(Green2 == true){
  leds[i] = CRGB(0, 255, 0);
  FastLED.setBrightness(60-2*i);
  FastLED.show();
  delay(100);
  }
}

// Turn lights Flashing Red
for(int i = 0; i < NUM_LEDS; i++){ // Flash on
  while(Red == true){
  leds[i] = CRGB(255, 0, 0);
  FastLED.setBrightness(2*i);
  FastLED.show();
  delay(100);
  }
}
for(int i = NUM_LEDS; i > 0; i--){ // Flash off
  while(Red == true){
  leds[i] = CRGB(255, 0, 0);
  FastLED.setBrightness(60-2*i);
  FastLED.show();
  delay(100);
  }
}

// // Sets all leds to solid green when 'x'
// if(Green1 == true){ 
//   leds = CRGB::Green;
//   }




}