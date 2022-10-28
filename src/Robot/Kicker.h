#ifndef KICKER_H_
#define KICKER_H_

#include <Arduino.h>
#include <PS5BT.h>
#include <PS3BT.h>
#include <SPI.h>
#include <Servo.h>

/**
 * @brief Kicker header file
 * @authors Andrew Nelson 
 */

class Kicker {
  private:
    uint8_t m_kickerpin;
    Servo kickerMotor;
  public:
    Kicker();
    void setup(uint8_t KickerPin);
    void Test();
    void Windup();
    void Release();
    void Stop();
};

#endif /* KICKER_H_ */