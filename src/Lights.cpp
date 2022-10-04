// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);

#include <FastLED.h>
#define LED_PIN 7
#define NUM_LEDS 30 

// bool Green1 = false; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BE SOLID GREEN
// bool Green2 = false; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BLINK GREEN
// bool Blue = false; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BE SOLID BLUE
// bool Red = false; // VARIABLE TO DETERMINE IF LIGHTS SHOULD BLINK RED

enum LEDState {
  PAIRING,      // flash Blue
  PAIRED,       // green then fade out
  OFFENSE,      // green
  DEFENSE,      // blue
  BALL_CARRIER, // flash red
};

LEDState currState;
CRGBArray<NUM_LEDS> leds;
uint8_t iteration = 255;

void setLEDStatus(LEDState status);
void updateLEDS();

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // Power Failsafe
  // Clears LEDs when code is updated
  FastLED.clear();
  FastLED.show();
}

void loop(){
  // Determining when 

  /// Sets all leds to solid green when 'x'
  // for(int i = 0; i < NUM_LEDS; i++){
  //   while(Green1 == true){
  //   leds[i] = CRGB(0, 255, 0);
  //   FastLED.show();
  // }
  setLEDStatus(PAIRED);
  updateLEDS();
  FastLED.delay(20);

  iteration += 15;
}

void setLEDStatus(LEDState status) {
  currState = status;
}

void updateLEDS() {
  switch(currState) {
    case PAIRING: {
      break;
    }
    case PAIRED: {
      leds = CRGB::Green;
      FastLED.setBrightness(iteration);
      break;
    }
    case OFFENSE: {
      leds = CRGB::Blue;
      break;
    }
    case DEFENSE: {
      leds = CRGB::Green;
      break;
    }
    case BALL_CARRIER: {
      if(iteration % 255/16 == 0) {
        leds = CRGB::Red;
        FastLED.delay(30);
      } else {
        leds = CRGB::Black;
      }
      break;
    }
    default: {
      leds = CRGB::Red;
      break;
    }
  }
  FastLED.show();
}



// // Turn lights Flashing Green
// for(int i = 0; i < NUM_LEDS; i++){ // Flash on
//   while(Green2 == true){
//   leds[i] = CRGB(0, 255, 0);
//   FastLED.setBrightness(2*i);
//   FastLED.show();
//   delay(100);
//   }
// }
// for(int i = NUM_LEDS; i > 0; i--){ // Flash off
//   while(Green2 == true){
//   leds[i] = CRGB(0, 255, 0);
//   FastLED.setBrightness(60-2*i);
//   FastLED.show();
//   delay(100);
//   }
// }

// // Turn lights Flashing Red
// for(int i = 0; i < NUM_LEDS; i++){ // Flash on
//   while(Red == true){
//   leds[i] = CRGB(255, 0, 0);
//   FastLED.setBrightness(2*i);
//   FastLED.show();
//   delay(100);
//   }
// }
// for(int i = NUM_LEDS; i > 0; i--){ // Flash off
//   while(Red == true){
//   leds[i] = CRGB(255, 0, 0);
//   FastLED.setBrightness(60-2*i);
//   FastLED.show();
//   delay(100);
//   }
// }

// // Sets all leds to solid green when 'x'
// if(Green1 == true){ 
//   leds = CRGB::Green;
//   }




