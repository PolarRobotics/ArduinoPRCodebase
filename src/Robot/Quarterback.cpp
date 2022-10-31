/*
       ___    _   _      _      ____    _____   _____   ____    ____       _       ____   _  __   ____         ___
      / _ \  | | | |    / \    |  _ \  |_   _| | ____| |  _ \  | __ )     / \     / ___| | |/ /  |___ \       / _ \
     | | | | | | | |   / _ \   | |_) |   | |   |  _|   | |_) | |  _ \    / _ \   | |     | ' /     __) |     | | | |
     | |_| | | |_| |  / ___ \  |  _ <    | |   | |___  |  _ <  | |_) |  / ___ \  | |___  | . \    / __/   _  | |_| |
      \___\_\ \___/  /_/   \_\ |_| \_\   |_|   |_____| |_| \_\ |____/  /_/   \_\  \____| |_|\_\  |_____| (_)  \___/
  */

/** QB code
 * ---Functions list---
 * Quaterback
 * fly_control
 * fly_open
 * fly_close
 * fly_stop
 * arm_control
 * arm_up
 * arm_down
 * arm_down
 * 
 * Rewrite code for quarterback using center as a template, ask upperclassmen
 * how to do stuff.
 * 
 * 
 * 
 **/

#include "PolarRobotics.h"
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <PS5BT.h>

enum arm_status 
{
  high, low, stop
};

enum fly_status
{
  high, stop
};

class Quarterback 
{
  private:
    uint8_t m_fly_pin, m_elevation_pin;
    arm_status m_currarm_status;
    fly_status m_currfly_status;
    Servo fly_motor;
    Servo elevation_motor;
    void fly_control();
    void elevation_control();

    public:
      Quarterback(uint8_t fly_pin, uint8_t evelation_pin);
      void high();
      void stop();
      void high();
      void low();
      void elevation_stop();
};

/**
 * Description: Public function that starts the arm and fly motors and sets their
 * starting status to stop.
 * Author @ h-charles
 * Date 9/26/2022
 **/
Quarterback::Quarterback(uint8_t fly_pin, uint8_t elevation_pin) 
{
  m_fly_pin = fly_pin;
  m_elevation_pin = elevation_pin;
  fly_motor.attach (fly_pin);
  elevation_motor.attach (elevation_pin);
  m_currarm_status = arm_status::stop;
  m_currfly_status = fly_status::stop;
}

/**
 * Description: public function that sets the fly status to high.
 * Author: @ h-charles
 * Date: 9/26/2022
 **/
void Quarterback::high()
{
  m_currfly_status = fly_status::high;
  Quarterback::fly_control();
}

