#pragma once

#ifndef __OLDCENTER_H__
#define __OLDCENTER_H__

#include <Arduino.h>
#include <SPI.h>

#include <Servo.h>
#include "Robot.h"

#define ARM_PIN 6
#define CLAW_PIN 13

#define CLAW_SPEED_OPEN 96
#define CLAW_SPEED_CLOSE 84
#define CLAW_SPEED_STOP 93

#define ARM_SPEED_LOWER 98
#define ARM_SPEED_HIGHER 87
#define ARM_SPEED_STOP 93
#define ARM_SPEED_HOLD 90


enum ArmStatus {
  HIGHER, LOWER, STOP, HOLD
};

enum ClawStatus {
  OPEN, CLOSE, STOP
};

class Center: public Robot {
  private:
    // uint8_t clawPin, m_elevationpin;
    Servo clawMotor;
    Servo armMotor;

  public:
    Center(); 
    void clawControl(ClawStatus target);
    void armControl(ArmStatus target);
    void initialize() override;
    void action(PS5BT& PS5) override;
};

#endif