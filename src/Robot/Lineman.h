#ifndef LINEMAN_H_
#define LINEMAN_H_

#include "Robot/Robot.h"

/**
 * @brief Lineman Subclass Header
 * @authors Max Phillips
 */

class Lineman: public Robot {
    private: 
        uint8_t test;
    public:
        // drive pins constants, stored in flash instead of SRAM
        const PROGMEM uint8_t lPin = 3; 
        const PROGMEM uint8_t rPin = 5;
        Lineman() { Serial.println(F("Creating lineman")); };

        // Override virtual functions
        void initialize() override;
        void action(PS5BT& PS5) override;
};

#endif