#include <Arduino.h>
#include <SPI.h> //Built in
#include <EEPROM.h> //Built in
#include <PS5BT.h>

// Custom Polar Robotics Libraries:
// #include "PolarRobotics.h"
// #include <Robot/Robot.h>
#include <Drive/Drive.h>
// #include <Robot/Lights.h>

// USB, Bluetooth, and Controller variable initialization
// The USB Host shield uses pins 9 through 13, so don't use those pins
USB Usb; 
BTD Btd(&Usb);    
PS5BT PS5(&Btd);

// Robot and Drivebase 
// Robot robot;
#define lPin 3
#define rPin 5
Servo leftMotor;
Servo rightMotor;
uint8_t motorType;
Drive DriveMotors;

// Lights robotLED;
// unsigned long CURRENTTIME;

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
//   DriveMotors.attach();
  motorType = EEPROM.read(0);
//   EEPROM.write(4, EEPROM.read(4) + 1);
//   Serial.print(F("\r\nIteration:"));
//   Serial.print(EEPROM.read(4));
  DriveMotors.setMotorType((MOTORS) motorType);
  leftMotor.attach(lPin);
  rightMotor.attach(rPin);
  DriveMotors.setServos(leftMotor, rightMotor);
  
  // Set initial LED color state
//   robotLED.setupLEDS();
//   robotLED.setLEDStatus(Lights::PAIRING);

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nReconnecting..."));
    while (Usb.Init() == -1) { // wait until the controller connects
      delay(5); 
    }
  } else{
    // robotLED.setLEDStatus(Lights::DEFENSE);
  }

  Serial.print(F("\r\nConnected"));
  analogWrite(lPin, 0);
  analogWrite(rPin, 0);

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
  if ((millis() - PS5.getLastMessageTime()) < 100 && PS5.connected()) { // checks if PS5 is connected, had response within 300 ms
    DriveMotors.setStickPwr(PS5.getAnalogHat(LeftHatY), PS5.getAnalogHat(RightHatX));

    // determine BSN percentage (boost, slow, or normal)
    if (PS5.isTouching()){
      DriveMotors.emergencyStop();
      DriveMotors.setBSN(Drive::brake);
    } else if (PS5.getButtonPress(R1)) {
      DriveMotors.setBSN(Drive::boost);
    } else if (PS5.getButtonPress(L1)) {
      DriveMotors.setBSN(Drive::slow);
    } else {
      DriveMotors.setBSN(Drive::normal);

    }

    // if(PS5.getButtonPress(UP)){
    //   robotLED.togglePosition();
    // }
    
    // if (millis() - CURRENTTIME >= 200) {
    //     CURRENTTIME = millis();
    //     robotLED.togglePosition();
    // }

    // Update the motors based on the inputs from the controller
    if(PS5.getAnalogButton(L2)) {
      DriveMotors.drift();
    } else {
      DriveMotors.update();
    }
    
  } else { // no response from PS5 controller within last 300 ms, so stop
    // Emergency stop if the controller disconnects
    DriveMotors.emergencyStop();
  }
//   DriveMotors.printDebugInfo();

}
