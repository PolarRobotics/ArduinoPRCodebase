#pragma once

#ifndef QUARTERBACK_H_
#define QUARTERBACK_H_

#include "Robot/Robot.h"
#include "Drive/Drive.h"
#include <Servo.h>

// Flywheel defines
#define FLYWHEEL_SPEED_FULL 120 // this should be between 90 and 140.
#define FLYWHEEL_STOP_SPEED 93

// Elevation (linear actuators) defines
#define SERVO_SPEED_UP 175
#define SERVO_SPEED_STOP 90 // this should always be 90.
#define SERVO_SPEED_DOWN 5
#define MAX_ELEVATION 100
#define ELEVATION_PERIOD 3750

// Conveyor defines
#define CONVEYOR_ON 140
#define CONVEYOR_OFF 93

// Enum for Increasing or Decreasing Flywheel Speed
enum speedStatus {
  increase, decrease
};

// Enum for whether to aim up or aim down
enum qbAim {
  aimUp, aimDown
};

/**
 * @brief Quarterback Subclass Header
 * @authors Rhys Davies
 */
class Quarterback : public Robot {
    private:
        uint8_t m_FlywheelPin;
        uint8_t m_conveyorPin;
        uint8_t m_ElevationPin;
        Servo FWMotor;
        Servo conveyorMotor;
        Servo elevationMotors;
        bool flywheelsOn, conveyorOn;
        bool aimingUp, aimingDown;
        bool raise, lower;
        uint8_t currentElevation, targetElevation;
        unsigned long lastElevationTime;
        unsigned long lastFlywheelToggleTime;
        float flywheelSpeedFactor;
    public:
        Quarterback();
        void attachMotors(uint8_t fwpin,
            uint8_t conveyorpin, uint8_t elevationpin);
        void toggleFlywheels();
        void aim(qbAim dir);
        void toggleConveyor();
        void changeFWSpeed(speedStatus speed);
        void updateAim();
        void initialize() override;
        void action() override;
};

#endif // QUARTERBACK_H_