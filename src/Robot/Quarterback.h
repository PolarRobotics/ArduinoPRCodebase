#include <Robot/Robot.h>
#include <Drive/Drive.h>
#include <Servo.h>

// Flywheel defines
#define FLYWHEEL_RIGHT_SPEED_FULL 20 // this should be between 0 and 90.
#define FLYWHEEL_LEFT_SPEED_FULL 170 // this should be between 90 and 180.
#define FLYWHEEL_STOP_SPEED 90
#define FLYWHEEL_PERIOD 50

// Elevation (linear actuators) defines
#define SERVO_SPEED_UP 175
#define SERVO_SPEED_STOP 90 // this should always be 90.
#define SERVO_SPEED_DOWN 5
#define MAX_ELEVATION 100
#define ELEVATION_PERIOD 25

// Conveyor defines
#define CONVEYOR_ON 145
#define CONVEYOR_OFF 30 

/**
 * @brief Quarterback Subclass Header
 * @authors Rhys Davies
 */
class Quarterback { //: public Robot
    private: 
        uint8_t m_leftFlywheelPin;
        uint8_t m_rightFlywheelPin;
        uint8_t m_conveyorPin;
        uint8_t m_ElevationPin;
        Servo leftFWMotor;
        Servo rightFWMotor;
        Servo conveyorMotor;
        Servo elevationMotors;
        bool flywheelsOn, conveyorOn;
        bool raise, lower;
        uint8_t currentElevation, targetElevation;
        unsigned long lastElevationTime;
        unsigned long lastFlywheelToggleTime;
    public:
        Quarterback();
        void attachMotors(uint8_t rightfwpin, uint8_t leftfwpin, 
            uint8_t conveyorpin, uint8_t elevationpin);
        void toggleFlywheels();
        void aimUp();
        void aimDown();
        void stopAiming();
        void toggleConveyor();
};

Quarterback::Quarterback() {
    flywheelsOn = true;
    conveyorOn = true;
}

void Quarterback::attachMotors(uint8_t rightfwpin, uint8_t leftfwpin, 
            uint8_t conveyorpin, uint8_t elevationpin) {
    m_leftFlywheelPin = rightfwpin;
    m_rightFlywheelPin = leftfwpin;
    m_conveyorPin = conveyorpin;
    m_ElevationPin = elevationpin;
    leftFWMotor.attach(m_leftFlywheelPin);
    rightFWMotor.attach(m_rightFlywheelPin);
    conveyorMotor.attach(m_conveyorPin);
    elevationMotors.attach(m_ElevationPin);
}

// stage is the difference between the current position and target position, from a value of [-2, 2]
// void Quarterback::aim(int stage) {
//     benchmark = abs(stage) * QB_ELEVATION_INTERVAL;
//     if (stage > 0) { // target position above current position
//       elevationMotors.write(SERVO_SPEED_UP);
//     } else if (stage < 0) { // target position below current position
//       elevationMotors.write(SERVO_SPEED_DOWN);
//     }
// }

void Quarterback::toggleFlywheels() {
    if (millis() - lastFlywheelToggleTime >= FLYWHEEL_PERIOD) {
        if (flywheelsOn) {
            // turn on the flywheels
            rightFWMotor.write(FLYWHEEL_RIGHT_SPEED_FULL);
            leftFWMotor.write(FLYWHEEL_LEFT_SPEED_FULL);
        }
        else {
            // turn off the flywheels
            rightFWMotor.write(FLYWHEEL_STOP_SPEED);
            leftFWMotor.write(FLYWHEEL_STOP_SPEED);
        }
        // toggle the flywheel status
        flywheelsOn = !flywheelsOn;
    }
}

// Aiming related functions

void Quarterback::aimUp() {
    // move elevation motors up
    // wait, find the delta t for the time
    // stop elevation motors
    // stage--
    if(currentElevation < MAX_ELEVATION) {
        if(millis() - lastElevationTime >= ELEVATION_PERIOD) {
            // if the motors need to move up
            elevationMotors.write(SERVO_SPEED_UP);
            lastElevationTime = millis();
            currentElevation++;
        }
    }
    else {
        //stop the motors if the elevation is at max
        elevationMotors.write(SERVO_SPEED_STOP);
    }

}

void Quarterback::aimDown() {
    if(currentElevation > 0) {
        if(millis() - lastElevationTime >= ELEVATION_PERIOD) {
            // if the motors need to move down
            elevationMotors.write(SERVO_SPEED_DOWN);
            lastElevationTime = millis();
            currentElevation--;
        }
    }
    else {
        //stop the motors if the elevation is at min
        elevationMotors.write(SERVO_SPEED_STOP);
    }
}

void Quarterback::stopAiming() { 
    // turn elevation motor off
    elevationMotors.write(SERVO_SPEED_STOP);
}

void Quarterback::toggleConveyor() {
    if (conveyorOn) {
        // turn on the conveyor
        conveyorMotor.write(CONVEYOR_ON);
    }
    else {
        // turn off the conveyor
        conveyorMotor.write(CONVEYOR_OFF);
    }
    // toggle the flywheel status
    conveyorOn = !conveyorOn;
}

// void Quarterback::passBall() {
//     //turn flywheels on to low: approx 10 power for a light boost
//     rightFlywheelMotor.write(FLYWHEEL_STOP_SPEED - 10);
//     leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED + 10);
//     //once firing mechanism is finished add that in and make it a macro?
// }


// void Quarterback::update() {

// }

// void Quarterback::fireWeapon(String requestedStatus) {
//   if (requestedStatus == "Fire") {
//     FireMotor.write(50);
//   } else if (requestedStatus == "Retract") {
//     FireMotor.write(130);
//   } else if (requestedStatus == "Stop") {
//     FireMotor.write(90);
//   }
// }


/*
    Notes:

    IDEA:
    can have an action where if you hold down on a button
    and move the left hat you can adjust the elevation of the QB
    this can allow for better control of the height

*/

/*  OLD MAIN CODE:

    if (PS3.getButtonClick(TRIANGLE) && targetElevation + 1 < 3) {
      targetElevation = targetElevation + 1;
    } else if (PS3.getButtonClick(CROSS) && targetElevation - 1 >= 0) {
      targetElevation = targetElevation - 1;
    } else if (PS3.getButtonClick(DOWN)) {
      targetElevation = 0;
    }

    int maxCounter = 13000;

    if ((counter > maxCounter || counter < 0) && (aimingup == true || aimingdown == true)) {
      aimingup = false;
      aimingdown = false;
      ElevationMotor.write(getSpeedStop());
      if (counter == -1) {
        counter = 0;
      } else {
        counter = maxCounter;
      }
      Serial.println("im stuck");
    } else if (targetElevation > currentElevation) {
      ElevationMotor.write(getSpeedUp());
      currentElevation = targetElevation;
      aimingup = true;
      aimingdown = false;
    } else if (targetElevation < currentElevation) {
      ElevationMotor.write(getSpeedDown());
      currentElevation = targetElevation;
      aimingdown = true;
      aimingup = false;
    } else if (aimingup == true) {
      counter = counter + 1;
    } else if (aimingdown == true) {
      counter = counter - 1;
    }

    if (targetElevation == 1 && counter == maxCounter/2) {
      aimingup = false;
      aimingdown = false;
      ElevationMotor.write(getSpeedStop());
    }

    if (PS3.getButtonClick(LEFT)) {
      ElevationMotor.write(getSpeedDown());
      delay(8000);
      ElevationMotor.write(getSpeedStop());
      counter = 0;
      currentElevation = 0;
      targetElevation = 0;
    }

    if (PS3.getButtonPress(CIRCLE)) {
      conveyor.write(145);
    } else {
      conveyor.write(30);
    }

    // flywheels.write(60);
    
    if (PS3.getButtonClick(SQUARE)) {
      flywheelstate = flywheelstate + 1;
      if (flywheelstate == 1) {
        flywheels.write(100);
        //flywheelstatis = true;
        //Serial.print("ran line 1");
        //Serial.println("  ");
      } else if (flywheelstate == 2) {
        flywheels.write(145);
        //flywheelstatis = true;
        //Serial.print("ran line 1");
        //Serial.println("  ");
      } else if (flywheelstate==3){
        flywheels.write(93);
        //flywheelstatis = false;
        flywheelstate = 0;
        //Serial.print("ran line 2");
        //Serial.println("  ");
      }
        //Serial.print("ran line 3");
        //Serial.println("  ");
    }


*/
