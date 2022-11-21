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
    clawControl - Based on what is inputed in main, opens, closes, or stops the claw. 
    armControl - Based on what is inputed in main, raises, lowers, or stops the arm. 
*/
#include "Robot/Center.h"

/**
 * Description: Public function that starts the arm and claw motors and sets their starting status to stop. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
**/
Center::Center() {
    setDrive(new Drive());
}

void Center::initialize() {
    // this->setDrive(new Drive(3,5));
    Serial.println(F("Creating Center"));
}

void Center::action(PS5BT& PS5) {
    Serial.println(F("Actual Center Action Executed"));
    if (PS5.getAnalogButton(R2)) {
      armControl(armStatus::Higher);
    } else if (PS5.getAnalogButton(L2)) {
      armControl(armStatus::Lower);
    } else if (PS5.getButtonPress(TRIANGLE)) {
      armControl(armStatus::Hold);
    } else {
      armControl(armStatus::Stop);
    }
    
    if (PS5.getButtonPress(UP)) {
      clawControl(clawStatus::Open);
    } else if (PS5.getButtonPress(DOWN)) {
      clawControl(clawStatus::Close);
    } else {
      clawControl(clawStatus::clawStop);
    }
}

void Center::setServos(Servo& armPin, Servo& clawPin) {
  armmotor=armPin;
  clawmotor=clawPin;
}

/**
 * Description: Public helper function that checks the claw status and updates the claw motor accordingly. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
 **/
void Center::clawControl(clawStatus reqstatus) {
  if(reqstatus == clawStatus::Open) {
    clawmotor.write(96);
  } else if(reqstatus == clawStatus::Close) {
    clawmotor.write(84);
  } else if(reqstatus == clawStatus::clawStop) {
    clawmotor.write(93);
  }
}

/**
 * Description: Public helper function that checks the arm status and updates the arm motor accordingly. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
 **/
void Center::armControl(armStatus reqstatus) {
  if (reqstatus == armStatus::Lower) {
    armmotor.write(98);
  } else if (reqstatus == armStatus::Higher) {
    armmotor.write(87);
  } else if (reqstatus == armStatus::Stop) {
    armmotor.write(93);
  } else if (reqstatus == armStatus::Hold) {
    armmotor.write(90);
  }

}