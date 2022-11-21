#pragma once

#ifndef __OLDCENTER_H__
#define __OLDCENTER_H__

#include <Arduino.h>
#include <SPI.h>

#include <Servo.h>
#include "Robot.h"

enum armStatus {
  Higher, Lower, Stop, Hold
};

enum clawStatus {
  Open, Close, clawStop
};

class Center: public Robot {
  private:
    // uint8_t clawPin, m_elevationpin;
    Servo clawmotor;
    Servo armmotor;

  public:
    Center(); 
    void setServos(Servo& armPin, Servo& clawPin);
    void clawControl(clawStatus reqstatus);
    void armControl(armStatus reqstatus);
    void initialize() override;
    void action(PS5BT& PS5) override;
};

#endif