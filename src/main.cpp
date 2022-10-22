#include <Arduino.h>
#include <SPI.h> //Built in
#include <EEPROM.h> //Built in
#include <PS5BT.h>
#include <TaskScheduler.h>
// Custom Polar Robotics Libraries:
// #include "PolarRobotics.h"
#include "Drive/Drive.h"


// the USB Host shield uses pins 9 through 13, so dont use these pins
USB Usb;            // There is a USB port
BTD Btd(&Usb);      // The Location of the Bluetooth port
PS5BT PS5(&Btd);
bool usbConnected = false;
#define lPin 3
#define rPin 5
Servo leftMotor;
Servo rightMotor;
int robotAge;
Drive DriveMotors;


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
  // DriveMotors.attach();
  robotAge = EEPROM.read(0);
  leftMotor.attach(lPin);
  rightMotor.attach(rPin);
  DriveMotors.setServos(leftMotor, rightMotor);
  
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
  } else { // no response from PS5 controller within last 300 ms, so stop
    // Emergency stop if the controller disconnects
    DriveMotors.emergencyStop();
  }
}
