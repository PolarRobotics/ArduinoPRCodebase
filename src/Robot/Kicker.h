#pragma once

#ifndef KICKER_H_
#define KICKER_H_

#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"

#define WINDUP_PIN 7
#define WIND_STOP_SPEED 90
#define WIND_FWD_SPEED 70
#define WIND_REV_SPEED 110

/**
 * @brief Kicker header file
 * @authors Andrew Nelson 
 */

class Kicker: public Robot {
  private:
    bool kickerEnabled; // safety feature
    Servo windupMotor;
  public:
    Kicker();
    void initialize() override;
    void action(PS5BT& PS5) override;
    void test();
    void windFwd();
    void windRev();
    void windStop();
    void enable();
    void disable();
};

#endif /* KICKER_H_ */