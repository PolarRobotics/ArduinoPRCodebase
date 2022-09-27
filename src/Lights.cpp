// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);

#include <FastLED.h>
#define LED_PIN 7
#define NUM_LEDS 30 

int Green1; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BE SOLID GREEN
int Green2; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BLINK GREEN

int Red1; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BE SOLID RED
int Red2; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BLINK RED

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



/// Sets all leds to flash green when 'x'
for(int i = 0; i < NUM_LEDS; i++){
  while(Green2 == true){
  leds[i] = CRGB(0, 255, 0);
  FastLED.setBrightness(6*i);
  FastLED.show();
  delay(100);
  }
}

for(int i = NUM_LEDS; i > 0; i--){
  while(Green2 == true){
  leds[i] = CRGB(0, 255, 0);
  FastLED.setBrightness(60-2*i);
  FastLED.show();
  delay(100);
  }
}

// Sets all leds to green when 'x'

if(Green1 == true){ 
  leds = CRGB::Green;
  }

if(Green2 == true){
 
  }
}




}