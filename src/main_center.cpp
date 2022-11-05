#include <Arduino.h>
#include <SPI.h> //Built in
#include <EEPROM.h> //Built in
#include <PS5BT.h>
#include <TaskScheduler.h>

// Custom Polar Robotics Libraries:
// #include "PolarRobotics.h"
#include <Robot/Robot.h>
#include <Drive/Drive.h>
#include <Robot/Edited_Center.h>

// USB, Bluetooth, and Controller variable initialization
// The USB Host shield uses pins 9 through 13, so don't use those pins
USB Usb; 
BTD Btd(&Usb);    
PS5BT PS5(&Btd);

// Robot and Drivebase 
Robot robot;
#define lPin 3
#define rPin 5
Servo leftMotor;
Servo rightMotor;
uint8_t motorType;
Drive DriveMotors;

// Center Robot
Center CenterBot;   
Servo centerArm;
Servo centerClaw;
#define armPin 6
#define clawPin 13


/*
   ____    _____   _____   _   _   ____
  / ___|  | ____| |_   _| | | | | |  _ \
  \___ \  |  _|     | |   | | | | | |_) |
   ___) | | |___    | |   | |_| | |  __/
  |____/  |_____|   |_|    \___/  |_|

*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print(F("\r\nStarting..."));

  // Get the bot drive type and then attach drive motors and set drive type
  motorType = EEPROM.read(0);
  DriveMotors.setMotorType((MOTORS) motorType);
  leftMotor.attach(lPin);
  rightMotor.attach(rPin);
  DriveMotors.setServos(leftMotor, rightMotor);

  // Attach the center motors and set ps5 controller for center
  centerArm.attach(armPin);
  centerClaw.attach(clawPin);
  CenterBot.setServos(centerArm, centerClaw);
  PS5.leftTrigger.setTriggerForce(0, 255);
  PS5.rightTrigger.setTriggerForce(0, 255);
  
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nReconnecting..."));
    while (Usb.Init() == -1) { // wait until the controller connects
      delay(5); 
    }
  }

  Serial.print(F("\r\nConnected"));

  delay(1000);
}

/*
   __  __      _      ___   _   _     _        ___     ___    ____
  |  \/  |    / \    |_ _| | \ | |   | |      / _ \   / _ \  |  _ \
  | |\/| |   / _ \    | |  |  \| |   | |     | | | | | | | | | |_) |
  | |  | |  / ___ \   | |  | |\  |   | |___  | |_| | | |_| | |  __/
  |_|  |_| /_/   \_\ |___| |_| \_|   |_____|  \___/   \___/  |_|

*/
void loop() {
 
  // clean up the usb registers, allows for new commands to be executed
  Usb.Task();

  // The main looping code, controls driving and any actions during a game
  if ((millis() - PS5.getLastMessageTime()) < 300) { // checks if PS5 is connected, had response within 300 ms
    DriveMotors.setStickPwr(PS5.getAnalogHat(LeftHatY), PS5.getAnalogHat(RightHatX));

    // determine BSN percentage (boost, slow, or normal)
    if (PS5.getButtonPress(R1)) {
      DriveMotors.setBSN(Drive::boost);
    } else if (PS5.getButtonPress(L1)) {
      DriveMotors.setBSN(Drive::slow);
    } else {
      DriveMotors.setBSN(Drive::normal);
    }

    // Update the motors based on the inputs from the controller  
    DriveMotors.update();

    // Check the what to do with the arm
    if (PS5.getAnalogButton(R2)) {
        CenterBot.armControl(armStatus::Higher);
    } else if (PS5.getAnalogButton(L2)) {
        CenterBot.armControl(armStatus::Lower);
    } else if (PS5.getButtonPress(SQUARE)) {
        CenterBot.armControl(armStatus::Hold);
    } else {
        CenterBot.armControl(armStatus::Stop);
    }
    
    // Check to do with the claw
    if (PS5.getButtonPress(TRIANGLE)) {
        CenterBot.clawControl(clawStatus::Open);
    } else if (PS5.getButtonPress(CROSS)) {
        CenterBot.clawControl(clawStatus::Close);
    } else {
        CenterBot.clawControl(clawStatus::clawStop);
    }





  } else { // no response from PS5 controller within last 300 ms, so stop
    // Emergency stop if the controller disconnects
    DriveMotors.emergencyStop();
  }


}