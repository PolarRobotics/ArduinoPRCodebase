#include <Arduino.h>
#include <SPI.h> //Built in
#include <EEPROM.h> //Built in
#include <PS5BT.h>
// #include "PolarRobotics.h"
#include "Drive/Drive.h"

#define buttonPin 4

// The variables for PS5 and pair button
bool debounce = false;
USB Usb;            // There is a USB port
BTD Btd(&Usb);      // The Location of the Bluetooth port
PS5BT PS5(&Btd);

Drive DriveMotors(3, 5);

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
  
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }

  pinMode(buttonPin, INPUT_PULLUP);
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
  Usb.Task();
  // The main looping code, controls driving and any actions during a game
  // put your main code here, to run repeatedly:

  if (PS5.connected()) {
    DriveMotors.setStickPwr(PS5.getAnalogHat(LeftHatY), PS5.getAnalogHat(RightHatX));

    // determine BSN percentage (boost, slow, or normal)
    if (PS5.getButtonPress(R1)) {
      DriveMotors.setBSN(Drive::boost);
    } else if (PS5.getButtonPress(L1)) {
      DriveMotors.setBSN(Drive::slow);
    } else {
      DriveMotors.setBSN(Drive::normal);
    }

    DriveMotors.update();
  } else {
    DriveMotors.emergencyStop();
  }
  // If the button is pressed and it is not debounced then go into statement
  if (digitalRead(buttonPin) == 1 && !debounce) {
    Serial.println("Pairing...");
    debounce = true;             
    PS5.disconnect();            // Disconnect the current PS5 controller
    delete [] &PS5;              // Deletes the memory allocation for the PS5 controller so a new one can be created with same name
    PS5 = PS5BT(&Btd, 1);        // Re-initalizes the PS5 object
    do {                         // Delay any other code from running until the PS5 controller is connected
      delay(10);
    } while (!PS5.connected());
    if(PS5.connected()) {        // Reset the debounce when it finally connects so button can be pressed and it can run again
      debounce = false;
    }
  }

}
