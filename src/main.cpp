#include <Arduino.h>
#include <SPI.h> //Built in
// #include <EEPROM.h> //Built in
#include <PS5BT.h>
#include <TaskScheduler.h>
// Custom Polar Robotics Libraries:
// #include "PolarRobotics.h"
#include "Drive/Drive.h"

#define buttonPin 8

// The variables for PS5 and pair button
bool debounce = false;
bool usbConnected = false;

// the USB Host shield uses pins 9 through 13, so dont use these pins
USB Usb;            // There is a USB port
BTD Btd(&Usb);      // The Location of the Bluetooth port
PS5BT PS5(&Btd);

unsigned long start = 0, end = 0, delta = 0; 

Drive DriveMotors(3, 5);

// Tasks + Scheduler
void t1Callback(); // First instance of function to be ran
Scheduler taskRunner; // Name the scheduler
Task task1(500, TASK_FOREVER, &t1Callback); // time in milliseconds, amount of iterations, function to run

void t1Callback() {
  // your code to refresh here
  if (PS5.connected()) {
    usbConnected = true;  // The USB receiver is still receiving information
  } else {
    usbConnected = false; // Makes sure the function doesn't run again
    DriveMotors.emergencyStop(); // stop the motors
  }
}

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
  
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nReconnecting..."));
    while (Usb.Init() == -1) { // wait until the controller connects
      delay(5); 
    }
  }

  Serial.print(F("\r\nConnected"));

  // Enable the Tasks
  taskRunner.init();
  taskRunner.addTask(task1); // Add task to scheduler

  task1.enable();

  pinMode(buttonPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
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
  // this loop took 4 to 5 ms to run through on 10-10-2022
  taskRunner.execute();

  Usb.Task();

  // The main looping code, controls driving and any actions during a game
  if (usbConnected) {
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
  } else {
    // Emergency stop if the controller disconnects
    DriveMotors.emergencyStop();
  }
}
