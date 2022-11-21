#include <Arduino.h>
#include <SPI.h> // Built in
#include <EEPROM.h> // Built in
#include <PS5BT.h>

// Custom Polar Robotics Libraries:
#include "PolarRobotics.h"
#include "Drive/Drive.h"
#include "Robot/Robot.h"
#include "Robot/Lineman.h"

// Special Robot Libraries
// This build flag is set by the selected environment (see platformio.ini)
#if INCLUDE_SPECIAL == 1
#include "Robot/Quarterback.h"
#else
#include "Robot/Dummy.h"
#endif

// USB, Bluetooth, and Controller variable initialization
// The USB Host shield uses pins 9 through 13, so don't use those pins
USB Usb;
BTD Btd(&Usb);
PS5BT PS5(&Btd);

// Robot and Drivebase
// TODO: in the future, retrieve drive pins from specific drive subclass
uint8_t lPin = 3; // default pin for left motor is 3
uint8_t rPin = 5; // default pin for right motor is 5
Robot* robot;
Drive* drive;
Servo leftMotor;
Servo rightMotor;
MOTORS motorType;

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
  Serial.begin(115200);
  Serial.print(F("\r\nStarting..."));

  /* * * * * * * * * * * * * *
  * Robot Type Determination *
  * * * * * * * * * * * * * */

  TYPE type = (TYPE) EEPROM.read(1);
  Serial.print(F("EEPROM 1: "));
  Serial.println(static_cast<String>(type));

  switch (type) {
    case quarterback:
      Serial.println(F("Robot Type: Quarterback"));
      robot = new Quarterback();
      break;
    case lineman:
    default:
      // Assume lineman
      Serial.println(F("Robot Type: Lineman"));
      robot = new Lineman();
  }

  robot->initialize();
  drive = robot->getDrive(); // grab pointer to drive for easier use

  // * Motors
  motorType = (MOTORS) EEPROM.read(0);
  drive->setMotorType(motorType);
  // ! Drive motor servos must be attached in main.cpp, otherwise we get delays
  leftMotor.attach(lPin);
  rightMotor.attach(rPin);
  drive->setServos(leftMotor, rightMotor);

  // Set initial LED color state
  // robotLED.setupLEDS();
  // robotLED.setLEDStatus(Lights::PAIRING);

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nReconnecting..."));
    while (Usb.Init() == -1) { // wait until the controller connects
      delay(5);
    }
  } else{
    // robotLED.setLEDStatus(Lights::DEFENSE);
  }

  // Serial.print(F("\r\nConnected"));

}

/*
   __  __      _      ___   _   _     _        ___     ___    ____
  |  \/  |    / \    |_ _| | \ | |   | |      / _ \   / _ \  |  _ \
  | |\/| |   / _ \    | |  |  \| |   | |     | | | | | | | | | |_) |
  | |  | |  / ___ \   | |  | |\  |   | |___  | |_| | | |_| | |  __/
  |_|  |_| /_/   \_\ |___| |_| \_|   |_____|  \___/   \___/  |_|

*/

// The main looping code, controls driving and any actions during a game
void loop() {

  // clean up the usb registers, allows for new commands to be executed
  // Usb.Task();

  

  // checks if PS5 is connected, had response within 300 ms
  if ((millis() - PS5.getLastMessageTime()) < 100 && PS5.connected()) { 
    drive->setStickPwr(PS5.getAnalogHat(LeftHatY), PS5.getAnalogHat(RightHatX));

    // determine BSN percentage (boost, slow, or normal)
    if (PS5.isTouching()){
      drive->emergencyStop();
      drive->setBSN(Drive::brake);
    } else if (PS5.getButtonPress(R1)) {
      drive->setBSN(Drive::boost);
    } else if (PS5.getButtonPress(L1)) {
      drive->setBSN(Drive::slow);
    } else {
      drive->setBSN(Drive::normal);
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
      drive->drift();
    } else {
      drive->update();
    }

    // Special Robot Action
    robot->action();

  } else { // no response from PS5 controller within last 300 ms, so stop
    // Emergency stop if the controller disconnects
    drive->emergencyStop();
  }

  // DriveMotors.printDebugInfo();
  // robotLED.updateLEDS();

}
