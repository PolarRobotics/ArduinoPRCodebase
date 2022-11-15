#include <Arduino.h>
#include <SPI.h> //Built in
#include <EEPROM.h> //Built in
#include <PS5BT.h>

// Custom Polar Robotics Libraries:
#include <PolarRobotics.h>

// #include <Robot/Robot.h>
#include <Robot/Quarterback.h>
#include <Robot/Center.h>
#include <Robot/Kicker.h>

#include <Drive/Drive.h>

// USB, Bluetooth, and Controller variable initialization
// The USB Host shield uses pins 9 through 13, so don't use those pins
USB Usb; 
BTD Btd(&Usb);    
PS5BT PS5(&Btd);

// Robot specific variables and objects
// Robot robot;

uint8_t botType;
// Lineman lineman;
// Receiver receiver;

// Center specific variables  
Center centerbot;
Servo centerArm;
Servo centerClaw;
#define armPin 6
#define clawPin 13

// Quarterback specific variables
Quarterback quarterbackbot;
#define FLYWHEEL_PIN 7
#define ELEVATION_MOTORS_PIN 4
#define CONVEYOR_MOTOR_PIN 6 

// Kicker specific variables
Kicker kickerbot;
#define windupPin 7

// Drive specific variables and objects
#define lPin 3
#define rPin 5
Servo leftMotor;
Servo rightMotor;
uint8_t motorType;

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
  motorType = EEPROM.read(0);
  DriveMotors.setMotorType((MOTORS) motorType);
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

  // check the EEPROM to get the bot type
  // 0 = Lineman
  // 1 = Recever
  // 2 = Center
  // 3 = Quarter Back
  // 4 = Kicker
  botType = EEPROM.read(1);
  if (botType == lineman || botType == receiver) {
    // the bot is a lineman or a receiver

  }
  else if (botType == center) {
    // the bot is a center (old center)
    centerArm.attach(armPin);
    centerClaw.attach(clawPin);
    centerbot.setServos(centerArm, centerClaw);
    PS5.leftTrigger.setTriggerForce(0, 255);
    PS5.rightTrigger.setTriggerForce(0, 255);
  }
  else if (botType == quarterback) {
    // the bot is a quarterback
    quarterbackbot.attachMotors(FLYWHEEL_PIN, CONVEYOR_MOTOR_PIN, ELEVATION_MOTORS_PIN);
  }
  else if (botType == kicker) {
    // the bot is a kicker 
    DriveMotors.setMotorType(MOTORS::big);
    kickerbot.setup(windupPin);
  }
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

    if (botType == lineman || botType == receiver) {
      

    }
    else if (botType == center) {
      if (PS5.getAnalogButton(R2)) {
        centerbot.armControl(armStatus::Higher);
      } else if (PS5.getAnalogButton(L2)) {
        centerbot.armControl(armStatus::Lower);
      } else if (PS5.getButtonPress(TRIANGLE)) {
        centerbot.armControl(armStatus::Hold);
      } else {
        centerbot.armControl(armStatus::Stop);
      }
      
      if (PS5.getButtonPress(UP)) {
        centerbot.clawControl(clawStatus::Open);
      } else if (PS5.getButtonPress(DOWN)) {
        centerbot.clawControl(clawStatus::Close);
      } else {
        centerbot.clawControl(clawStatus::clawStop);
      }
    }
    else if (botType == quarterback) {
      // Update the bools within the class to see if the user wants to go up or down
      if (PS5.getButtonClick(UP))
        quarterbackbot.aim(qbAim::aimUp);
      else if (PS5.getButtonClick(DOWN))
        quarterbackbot.aim(qbAim::aimDown);
      
      // Update the aim on quarterback to see if we need to stop or not
      quarterbackbot.updateAim();

      // Toogle the Conveyor and Flywheels
      if (PS5.getButtonClick(SQUARE))
        quarterbackbot.toggleConveyor();
      else if (PS5.getButtonClick(CIRCLE))
        {quarterbackbot.toggleFlywheels();}
      
      // Change the flywheel speed
      if(PS5.getButtonClick(TRIANGLE))
        quarterbackbot.changeFWSpeed(speedStatus::increase);
      else if (PS5.getButtonClick(CROSS))
        quarterbackbot.changeFWSpeed(speedStatus::decrease);
    }
    else if (botType == kicker) {
      // Control the motor on the kicker
      if (PS5.getButtonPress(TRIANGLE))
        kickerbot.turnfwd();
      else if (PS5.getButtonPress(CROSS))
        kickerbot.turnrev();
      else
        kickerbot.stop();
    }


  } else { // no response from PS5 controller within last 300 ms, so stop
    // Emergency stop if the controller disconnects
    DriveMotors.emergencyStop();
  }
}
