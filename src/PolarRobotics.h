#pragma once

#ifndef POLAR_ROBOTICS_H_
#define POLAR_ROBOTICS_H_

#include <Arduino.h>
#include <Servo.h>

#ifndef NUM_MOTORS
// the number of motors associated with driving, usually multiples of 2, default: 2
#define NUM_MOTORS 2 // 4 for mechanum wheels
#endif

// #TODO move this to the robot parent class
// #ifndef MECHANUM
// // determine if the motors are in a mechanum configuration or the standard config. default: false
// #define MECHANUM false
// #endif

// rate of change of power with respect to time when accelerating %power/10th of sec
#define ACCELERATION_RATE .0375 
// rate of deceleration/braking
#define BRAKE_PERCENTAGE -0.25 
// was 2000, for 2000ms. needs to be way faster
#define TIME_INCREMENT 25 
// DO NOT CHANGE THIS EVER!!!!!
#define PWM_CONVERSION_FACTOR 0.3543307087 

// Controller Defines
#define STICK_DEADZONE 8.0 / 127.0
#define THRESHOLD 0.00001

class Drive {
private:
  float stickForwardPower, stickTurnPower;

  // motor variables
  uint8_t motorPins[NUM_MOTORS];
  float motorPower[NUM_MOTORS];
  unsigned long lastRampTime[NUM_MOTORS];
  float currentPower[NUM_MOTORS];
  // float inputPower[NUM_MOTORS];
  // float rampedPower[NUM_MOTORS];
public:
  Drive();
  void setStickPwr(uint8_t leftY, uint8_t rightX);
  void generateTurnScalar(uint8_t mtr1, uint8_t mtr2);
  float ramp(float requestedPower, uint8_t mtr);
  float Convert2PWMVal(float rampPwr);
  float getMotorPwr(uint8_t mtr);
  void setMotors();
};

// All functions below this need to be moved to a parent Robot class or other name
// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);
struct LEDS {
  uint32_t m_color;
  int m_state;
};

enum BOT_STATE {
  PAIRING,
  CONNECTED,
  DISCONNECTED,
  OFFENSE,
  DEFENSE,
  TACKLED
};

// Robot Age Enum
// 0 for old robot, 1 for new robot
enum AGE {
  OLD,
  NEW
};

// Robot Type Enum
// 0 for lineman, 1 for reciever, 2 for center, 3 for quarterback, 4 for kicker
enum TYPE {
  lineman,
  receiver,
  center,
  quarterback,
  kicker
};

// QB Aim Enum
// Elevation enum and variables for linear actuators
enum ELEVATION {
  low,
  middle,
  high
};

#endif /* POLAR_ROBOTICS_H */
