/*
       ____   _____   _   _   _____   _____   ____
      / ___| | ____| | \ | | |_   _| | ____| |  _ \
     | |     |  _|   |  \| |   | |   |  _|   | |_) |
     | |___  | |___  | |\  |   | |   | |___  |  _ <
      \____| |_____| |_| \_|   |_|   |_____| |_| \_\
  
*/

/** Center Code
    --- Functions List ---
    Center
    clawControl - Based on inputs from main, opens, closes, or stops the claw. 
    armControl - Based on inputs from main, raises, lowers, or stops the arm. 
*/
#include "Robot/Center.h"

/**
 * Description: Public function that starts the arm and claw motors and sets their starting status to stop. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
**/
Center::Center() {
    this->setType(TYPE::center);
    this->setDrive(new Drive(MOTORS::small));

    armMotor.attach(ARM_PIN);
    armControl(ArmStatus::STOP);
    
    clawMotor.attach(CLAW_PIN);
    clawControl(ClawStatus::STOP);
}

void Center::initialize() {
    Serial.println(F("Creating Center"));
}

void Center::action(PS5BT& PS5) {
    Serial.println(F("Actual Center Action Executed"));
    if (PS5.getAnalogButton(R2)) {
      armControl(ArmStatus::HIGHER);
    } else if (PS5.getAnalogButton(L2)) {
      armControl(ArmStatus::LOWER);
    } else if (PS5.getButtonPress(TRIANGLE)) {
      armControl(ArmStatus::HOLD);
    } else {
      armControl(ArmStatus::STOP);
    }
    
    if (PS5.getButtonPress(UP)) {
      clawControl(ClawStatus::OPEN);
    } else if (PS5.getButtonPress(DOWN)) {
      clawControl(ClawStatus::CLOSE);
    } else {
      clawControl(ClawStatus::STOP);
    }
}

/**
 * Description: Public helper function that checks the claw status and updates the claw motor accordingly. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
 **/
void Center::clawControl(ClawStatus target) {
  switch (target) {
    case ClawStatus::OPEN:
        clawMotor.write(CLAW_SPEED_OPEN);
        break;
    case ClawStatus::CLOSE:
        clawMotor.write(CLAW_SPEED_CLOSE);
        break;
    case ClawStatus::STOP:
        clawMotor.write(CLAW_SPEED_STOP);
  } 
}

/**
 * Description: Public helper function that checks the arm status and updates the arm motor accordingly. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
 **/
void Center::armControl(ArmStatus target) {
  switch (target) {
    case ArmStatus::LOWER:
        armMotor.write(ARM_SPEED_LOWER);
        break;
    case ArmStatus::HIGHER:
        armMotor.write(ARM_SPEED_HIGHER);
        break;
    case ArmStatus::HOLD:
        armMotor.write(ARM_SPEED_HOLD);
        break;
    case ArmStatus::STOP:
        armMotor.write(ARM_SPEED_STOP);
  }

}