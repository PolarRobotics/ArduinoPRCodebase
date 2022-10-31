// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);

#include <FastLED.h>
#define LED_PIN 7
#define NUM_LEDS 30

class Lights {
private:
  uint8_t currState; //LEDState currState;
  CRGBArray <NUM_LEDS> leds;
  uint8_t iteration;
  // int i, updateCount;
public:
  // MUHAMMED ENUM PRAISE BE UPON HIM
  enum LEDState {
    PAIRING,      // nothing and then red
    PAIRED,       // green then fade out
    OFFENSE,      // blue
    DEFENSE,      // green
    BALL_CARRIER  // flash red
  };
  Lights();
  void setLEDStatus(LEDState status);
  void updateLEDS();
  void runLoop(int count);
};

// Function Definitions 
Lights::Lights() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // Power Failsafe
  // Clears LEDs when code is updated
  FastLED.clear();

  currState = PAIRING;
  updateLEDS();
  FastLED.show();
}

void Lights::setLEDStatus(LEDState status) {
  currState = status;
}

void Lights::updateLEDS() {
  switch(currState) {
    case PAIRING: {
      leds = CRGB::Red;
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




