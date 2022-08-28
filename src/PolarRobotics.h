#pragma once

#ifndef POLAR_ROBOTICS_H_
#define POLAR_ROBOTICS_H_

#include <Arduino.h>

#define NUM_MOTORS 2
const float ACCELERATION_RATE = .0375; // rate of change of power with respect to time when accelerating %power/10th of sec
const float BRAKE_PERCENTAGE = -0.25; // rate of deceleration/braking
const int timeIncrement = 25; // was 2000, for 2000ms. needs to be way faster

const float PWM_CONVERSION_FACTOR = 0.3543307087; // DO NOT CHANGE THIS EVER!!!!!

class Drive {
private:
  double CLOSE_ENOUGH; 
  unsigned long lastRampTime[NUM_MOTORS];
  float currentPower[NUM_MOTORS];
  // float inputPower[NUM_MOTORS];
  // float rampedPower[NUM_MOTORS];
public:
  Drive();
  float ramp(float requestedPower, byte mtr);
  float Convert2PWMVal(float rampPwr);
};

// float ramp(float requestedPower, int mtr);

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
