#pragma once

// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);

#include <FastLED.h>
#define LED_PIN 7
#define NUM_LEDS 30
#define TIME_BETWEEN_TOGGLES 25

class Lights {
private:
    unsigned long lastToggleTime;
    uint8_t currState; // LEDState currState;
    CRGBArray<NUM_LEDS> leds;
    uint8_t iteration;
    bool m_isOffense;
    // int i, updateCount;
public:
    // MUHAMMED ENUM PRAISE BE UPON HIM
    enum LEDState {
        PAIRING,     // Yellow
        PAIRED,      // green then fade out
        OFFENSE,     // blue (also need green)
        DEFENSE,     // green
        BALL_CARRIER // turn red and then go back to offense state (testing digital pin signal)
    };
    Lights();
    void setupLEDS();
    void setLEDStatus(LEDState status);
    // void setLEDColor(uint8_t r, uint8_t g, )
    void updateLEDS();
    //   void runLoop(int count);
    void togglePosition();
};