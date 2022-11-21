#pragma once

#ifndef KICKER_H_
#define KICKER_H_

#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"

/**
 * @brief Kicker header file
 * @authors Andrew Nelson 
 */

class Kicker: public Robot {
  private:
    bool m_enabled; // safety feature
    uint8_t m_kickerpin;
    Servo m_windupMotor;
  public:
    Kicker();
    void initialize() override;
    void action(PS5BT& PS5) override;
    void setup(uint8_t kicker_pin);
    void Test();
    void turnfwd();
    void turnrev();
    void stop();
};

#endif /* KICKER_H_ */