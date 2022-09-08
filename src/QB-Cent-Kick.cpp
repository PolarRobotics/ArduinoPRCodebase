// #include "PolarRobotics.h"

/*
       ___    _   _      _      ____    _____   _____   ____    ____       _       ____   _  __   ____         ___
      / _ \  | | | |    / \    |  _ \  |_   _| | ____| |  _ \  | __ )     / \     / ___| | |/ /  |___ \       / _ \
     | | | | | | | |   / _ \   | |_) |   | |   |  _|   | |_) | |  _ \    / _ \   | |     | ' /     __) |     | | | |
     | |_| | | |_| |  / ___ \  |  _ <    | |   | |___  |  _ <  | |_) |  / ___ \  | |___  | . \    / __/   _  | |_| |
      \___\_\ \___/  /_/   \_\ |_| \_\   |_|   |_____| |_| \_\ |____/  /_/   \_\  \____| |_|\_\  |_____| (_)  \___/
  */

// /* * * * * * * * * * * * * * *
//  * Quarterback Internal Code *
//  * * * * * * * * * * * * * * */

// // Servo Speed Consts
// const int SERVO_SPEED_UP = 175; // this should be between 90 and 180.
// const int SERVO_SPEED_STOP = 90; // this should always be 90.
// const int SERVO_SPEED_DOWN = 5; // this should be between 0 and 90.
// int getSpeedUp() { return SERVO_SPEED_UP; }
// int getSpeedStop() { return SERVO_SPEED_STOP; }
// int getSpeedDown() { return SERVO_SPEED_DOWN; }

// // Flywheel Speed Consts
// const int FLYWHEEL_RIGHT_SPEED_FULL = 20; // this should be between 0 and 90.
// const int FLYWHEEL_LEFT_SPEED_FULL = 170; // this should be between 90 and 180.
// const int FLYWHEEL_STOP_SPEED = 90; // this should always be 90.
// // to set the flywheel to another speed, subtract (for right) or add (left) from/to FLYWHEEL_STOP_SPEED like so:
// // for example, if you wanted 10 power (about 14% of the range between off and full)
// // leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED + 10);

// // Elevation Int
// const int QB_ELEVATION_INTERVAL = 4000; // Constant for time to determine stages of elevation
// unsigned long getElevationInterval() { return QB_ELEVATION_INTERVAL; }

// // Elevation Time Benchmark (interval * stage)
// unsigned long benchmark = 0;
// unsigned long getAimBenchmark() { return benchmark; }

// /* * * * * * * * * * * * * * *
//  * Aiming / Elevation Control *
//  * * * * * * * * * * * * * * */

// // stage is the difference between the current position and target position, from a value of [-2, 2]
// void aim(int stage) {
//   benchmark = abs(stage) * QB_ELEVATION_INTERVAL;
//   if (stage > 0) { // target position above current position
//     getElevationMotor().write(SERVO_SPEED_UP);
//   } else if (stage < 0) { // target position below current position
//     getElevationMotor().write(SERVO_SPEED_DOWN);
//   }
// }

// void stopAiming() { // wonder what this does
//   // turn elevation motor off
//   getElevationMotor().write(SERVO_SPEED_STOP);
// }

// //debug
// void debug_showSelectionInfo(ELEVATION c, ELEVATION t) {
//   Serial.print("\n");
//   Serial.print("current: ");
//   Serial.println(c);
//   Serial.print("target: ");
//   Serial.println(t);
//   Serial.print("abs: ");
//   Serial.println(abs(c - t));
// }

// void debug_showBenchmark() {
//   Serial.print("\n");
//   Serial.print("benchmark: ");
//   Serial.println(benchmark);
//   Serial.print("\n");
// }

// void debug_showTime() {
//   Serial.print("\n");
//   Serial.print("millis(): ");
//   Serial.println(millis());
//   Serial.print("\n");
//   Serial.print("timestamp: ");
//   Serial.println(QBAimTimestamp);
// }

// /* * * * * * * * * *
//  * Flywheel Control *
//  * * * * * * * * * */

// /** startFlywheel */
// void startFlywheel(Servo rightFlywheelMotor, Servo leftFlywheelMotor) {
//   rightFlywheelMotor.write(FLYWHEEL_RIGHT_SPEED_FULL);
//   leftFlywheelMotor.write(FLYWHEEL_LEFT_SPEED_FULL);
// }

// /** stopFlywheel */
// void stopFlywheel(Servo rightFlywheelMotor, Servo leftFlywheelMotor) {
//   rightFlywheelMotor.write(FLYWHEEL_STOP_SPEED);
//   leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED);
// }

// /** passBall */
// // pass the ball to the kicker
// void passBall(Servo rightFlywheelMotor, Servo leftFlywheelMotor) {
//   //turn flywheels on to low: approx 10 power for a light boost
//   rightFlywheelMotor.write(FLYWHEEL_STOP_SPEED - 10);
//   leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED + 10);
//   //once firing mechanism is finished add that in and make it a macro?
// }

// /* * * * * * * * * * * * * *
//  * Firing Mechanism Control *
//  * * * * * * * * * * * * * */

// /** fireWeapon */
// void fireWeapon(Servo FireMotor, String requestedStatus) {
//   if (requestedStatus == "Fire") {
//     FireMotor.write(50);
//   } else if (requestedStatus == "Retract") {
//     FireMotor.write(130);
//   } else if (requestedStatus == "Stop") {
//     FireMotor.write(90);
//   }
// }


/*
       ____   _____   _   _   _____   _____   ____
      / ___| | ____| | \ | | |_   _| | ____| |  _ \
     | |     |  _|   |  \| |   | |   |  _|   | |_) |
     | |___  | |___  | |\  |   | |   | |___  |  _ <
      \____| |_____| |_| \_|   |_|   |_____| |_| \_\
  */


/** Center Code

    --- Functions List ---
    int centerUp(SabertoothSimplified Sabre, String requestedStatus)        - Raises the arm on the center up    | Returns the current Position of the Arm for the main code to handle if it wants | Up Arrow on D-Pad Activates Function
    int centerDown(SabertoothSimplified Sabre, String requestedStatus)      - Lowers the arm on the center       | Returns the current Position of the Arm for the main code to handle if it wants | Down Arrow on D-Pad Activates Function
    
    int centerOpen(SabertoothSimplified Sabre, String requestedStatus)      - Move the servo to the open Position   | Pressing X Opens the Servo
    int centerClose(SabertoothSimplified Sabre, String requestedStatus)     - Move the servo to the closed Position | Pressing Triangle Closes the Servo

*/

#include <Arduino.h>
// #include <SabertoothSimplified.h>

// SabertoothSimplified ST; // We'll name the Sabertooth object ST.

enum class Status {
  Higher, Lower, Stop
};

class Center {
  private:
    uint8_t m_clawpin, m_elevationpin;
    int m_elevation;
    // SabertoothSimplified m_Sabertooth;
    Status m_currstatus;

  public:
    Center(uint8_t ClawPin, uint8_t ElevationPin); 
    void open(Status requestedStatus);
    void close(Status requestedStatus);
    int up(Status requestedStatus);
    int down(Status requestedStatus);
}

Center::Center(SabertoothSimplified ST, uint8_t ClawPin, uint8_t ElevationPin) {
  m_clawpin = ClawPin;
  m_elevationpin = ElevationPin;
  m_Sabertooth = ST;
  m_currstatus = Status::Stop;
}

/** void open */
void Center::open() { //, Status requestedStatus
  
}

/** void close */
void Center::close() { //, Status requestedStatus
  
}

/** int up */
int Center::up(Status requestedStatus) {
  if (requestedStatus == Status::Higher) {
    m_Sabertooth.motor(1, 10);
  } else if (requestedStatus == Status::Stop) {
    m_Sabertooth.motor(1, 0);
  }
  return 1;
}

/** int down */
int Center::down(Status requestedStatus) {
  if (requestedStatus == Status::Higher) {
    m_Sabertooth.motor(1, -10);
  } else if (requestedStatus == Status::Stop) {
    m_Sabertooth.motor(1, 0);
  }
  return 2;
}


// original code
//Raise the Center Arm
    if (CenterArmMoveStatus == "" && (PS3.getButtonPress(UP)) && CenterArmStatus != 1) {
      //The user wants to raise the Center arm up
      CenterArmMoveStatus = "High";
      CenterArmMoveTimer = millis();
      CenterArmStatus = centerUp(ST, CenterArmMoveStatus);
    } else if (CenterArmMoveStatus == "High" && (millis() - CenterArmMoveTimer) > 5000) {
      CenterArmMoveStatus = "Stop";
      centerUp(ST, CenterArmMoveStatus);
      CenterArmMoveStatus = "";
    }

    //Lower the Center Arm
    if (CenterArmMoveStatus == "" && (PS3.getButtonPress(DOWN)) && CenterArmStatus != 2) {
      //The user wants to raise the Center arm up
      CenterArmMoveStatus = "Lower";
      CenterArmMoveTimer = millis();
      CenterArmStatus = centerDown(ST, CenterArmMoveStatus);
    } else if (CenterArmMoveStatus == "Lower" && (millis() - CenterArmMoveTimer) > 5000) {
      CenterArmMoveStatus = "Stop";
      centerUp(ST, CenterArmMoveStatus);
      CenterArmMoveStatus = "";
    }


// kicker

/**
//General Framework for the Kicker Code

//Manual Wind Up Arm
//Up On D-Pad
void manualWindUp() {
  
}

//Auto Wind Up
//Triangle on Controller
void autoWindUp() {

}

//Slow Release Arm
//Down on DPad
void slowRelease() {

}

//Fire
//X on Controller
void fireKicker() {

}
**/