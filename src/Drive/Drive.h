#pragma once

#ifndef DRIVE_H_
#define DRIVE_H_

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

  Servo M1, M2; //temporary solution, use vector for future
  //vector<Servo> Motors;
  // motor variables
  uint8_t motorPins[NUM_MOTORS];
  float motorPower[NUM_MOTORS];
  unsigned long lastRampTime[NUM_MOTORS];
  float currentPower[NUM_MOTORS];
  // float inputPower[NUM_MOTORS];
  // float rampedPower[NUM_MOTORS];
public:
  Drive(int leftmotorpin, int rightmotorpin); //constructor for two motors
  void setStickPwr(uint8_t leftY, uint8_t rightX);
  void generateTurnScalar();
  float ramp(float requestedPower, uint8_t mtr);
  float Convert2PWMVal(float rampPwr);
  float getMotorPwr(uint8_t mtr);
  void update();
};

// Robot Age Enum
// 0 for old robot, 1 for new robot
// remove later - deprecated
enum AGE {
  OLD,
  NEW
};

#endif /* DRIVE_H */
