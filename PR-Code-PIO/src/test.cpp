#include <SPI.h>
#include <PS3BT.h>
#include <SabertoothSimplified.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <EEPROM.h>
#include <Servo.h>
#include <TaskScheduler.h>
#include "PolarRobotics.h"


USB Usb;            // There is a USB port
BTD Btd(&Usb);      // The Location of the Bluetooth port
PS3BT PS3(&Btd);    // There is a PS3 Controller at the location of the Bluetooth port
Servo servoObj;   // Initialize a servo object for the connected servo

Servo M1; //initialize Motor 1 as a servo
Servo M2; //initialize Motor 2 as a servo

SabertoothSimplified ST; // We'll name the Sabertooth object ST.

// Pin and Servo for quaterback flywheels
int rightFlywheelPin = 18;
int leftFlywheelPin = 19;
Servo rightFlywheelMotor;
Servo leftFlywheelMotor;

// Pin and Servo for quarterback firing mechanism
int FirePin = 16;
Servo FireMotor;

// Pin and Servo for quarterback aiming mechanism
int ElevationPin = 14;
Servo ElevationMotor;

// Tasks + Scheduler
void t1Callback(); // First instance of function to be ran
Scheduler taskRunner; // Name the scheduler
Task task1(500, TASK_FOREVER, &t1Callback); // time in milliseconds, amount of iterations, function to run

Servo getElevationMotor() { return ElevationMotor; }

// For how to configure the Sabertooth, see the DIP Switch Wizard for
//   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
// Be sure to select Simplified Serial Mode for use with this library.
// This sample uses a baud rate of 9600.
//
// Connections to make:
//   Arduino TX->1  ->  Sabertooth S1
//   Arduino GND    ->  Sabertooth 0V
//   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
//
// If you want to use a pin other than TX->1, see the SoftwareSerial example.


/* * * * * * * * * * * * * * *
 * Linemen/General Variables *
 * * * * * * * * * * * * * * */

//Position Values of respective joysticks as returned thru USB connection
int leftJStickY = 0;
int rightJStickX = 0;

// Variable motor power adjuster later in code based on joystick pos and bot type
int rightMotorValue = 0;
int leftMotorValue = 0;

// Helps us create a time interval so we are not checking on connection status and updating every ms.
// Helps performance as well as eliminates bot glitches from spotty connection
int lastChecked = 0;

// Works with lastChecked to make sure the controller is still connected (Just an added safety feature)
boolean usbConnected = false;

// Depends on the type of bot, makes sure the bigger bots get less power and don't rip themselves apart
double percentPower;

// Direction that the robot is currently moving
int dir = 0; //-1 back, 0 stop, 1 forward

// Direction the robot was last moving
int preDir = 0;

// How much time has elapsed since ramping up or down started
int rampETime = 0;

// Not specific to left or right, just a general power state of the motors between none and full power
double motorPower = 0; //0.4 for new robots, 1.0 for old robots

// Read on program start, tells the robot how to act and what class of robot it is
TYPE robotType = lineman;

// Also read on program start, tells which power system is being used (12v old, 24v new) changing motor powers etc.
AGE robotAge = OLD;

// How often we want the robot to refresh it's status and connections
static const unsigned long Ref_Int = 250; // ms

// When was the last time we refreshed, resets whenever robot refreshes
static unsigned long lastRefreshTime = 0;

// What power level the ramp functions of the robot starts with
double const initPercentPower = .1;

// The max power level the robot can have on ramping start
double const endPercentPower = 1;

int motortype;
float PWMConvertionFactor;
int oldMotorValueR;
int oldMotorValueL;
float PWMMotorValueR;
float PWMMotorValueL;

/* * * * * * * * * * * * * * * * *
 * Quarterback Specific Variables *
 * * * * * * * * * * * * * * * * */

// A timer used to run the motor for a set ammount of time (IMPLEMENT ENCODERS IN V3)
unsigned long QBAimTimestamp = millis();

// Boolean, true if currently aiming/moving the linear actuators, false otherwise
boolean aiming = false;

// Variables of custom enum for keeping track of elevation of linear actuators/aimers
ELEVATION currentElevation = low;
ELEVATION targetElevation = low;
ELEVATION getCurrentElevation() { return currentElevation; }

// Boolean keeping track of if the flywheels are on
boolean flywheelOn = false;

// Stores a timestamp that increases by BASE_TOGGLE_CD every time the flywheel is toggled
unsigned long flywheelToggleCooldown = millis();
const unsigned long BASE_TOGGLE_CD = 500; //500ms flywheel toggle cooldown


/* * * * * * * * * * * * * * *
   Center Specific Variables
 * * * * * * * * * * * * * * */

// Integer Position of the arm
int CenterArmStatus = 0; // 1 = Up, 2 = Down

// String Value used when actually moving arm to see where we are at
String CenterArmMoveStatus = "";

// A timer used to run the motor for a set ammount of time (IMPLEMENT ENCODERS IN V3)
unsigned long CenterArmMoveTimer = millis();

// The pin number for the PWM input of the servo
int servoInputPin = 9;

// Center Servo Angle?
int servoAngle = 0;

/*
   ____    _____   _____   _   _   ____
  / ___|  | ____| |_   _| | | | | |  _ \
  \___ \  |  _|     | |   | | | | | |_) |
   ___) | | |___    | |   | |_| | |  __/
  |____/  |_____|   |_|    \___/  |_|
*/


/* Setup Function

   Define the function that will repeat on the task scheduler

   Start Serial Port
   Connect to Sabertooth
   Connect USB and make sure it stays connected
   Read the robot type from the EEPROM on initialization
   Read the robot age from the EEPROM
   Print out any errors and continue to main loop function
*/

void t1Callback() {
  // your code to refresh here
  if (PS3.PS3Connected) {
    usbConnected = true;  // The USB receiver is still receiving information
  } else {
    usbConnected = false; // Makes sure the function doesn't run again
    ST.motor(1, 0);       // Stops the Motors
    ST.motor(2, 0);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.

  M1.attach(5);
  M2.attach(3);

  if (Usb.Init() == -1) {
    while (Usb.Init() == -1); //Wait until reconnect
  }

  /* * * * * * * * * * * * * *
   * Robot Type Determination *
   * * * * * * * * * * * * * */

  robotType = EEPROM.read(1);
  //Serial.print(robotType);
  if (robotType != lineman && robotType != receiver && robotType != center && robotType != quarterback && robotType != kicker)
  {
    Serial.print("Error on robot type read, assumed lineman role");
    robotType = lineman;
  } else if (robotType == center) {
    //We need to create the servo connection
    servoObj.attach(servoInputPin);  //The pin number to attach


    /*TESTING THE SERVO TO MAKE SURE I UNDERSTAND HOW IT WORKS!!!**/
    for (servoAngle = 0; servoAngle < 180; servoAngle += 1)   // command to move from 0 degrees to 180 degrees
    {
      servoObj.write(servoAngle);                 //command to rotate the servo to the specified angle
      delay(15);
    }

    delay(1000);

    for (servoAngle = 180; servoAngle >= 1; servoAngle -= 5) // command to move from 180 degrees to 0 degrees
    {
      servoObj.write(servoAngle);              //command to rotate the servo to the specified angle
      delay(5);
    }

    delay(1000);


  } else if (robotType == quarterback) {     // Quarterback Setup
    rightFlywheelMotor.attach(rightFlywheelPin);
    leftFlywheelMotor.attach(leftFlywheelPin);
    FireMotor.attach(FirePin);
    ElevationMotor.attach(ElevationPin);
  }

  /* * * * * * * * * * * * * *
   * Robot Age Determination *
   * * * * * * * * * * * * * */

  robotAge = EEPROM.read(0);
  //Serial.print(robotAge);
  if (robotAge == OLD)
  {
    // If it's an old (12V) robot, maximize drive motor power
    motorPower = 1.0;
  }
  else if (robotAge == NEW)
  {
    // If it's a new (24V) robot, tone down the base drive motor power so they don't do seven wheelies
    motorPower = 0.25;
  }
  else
  {
    Serial.print ("Read error on age of robot, assumed new robot");
    motorPower = 0.35;
  }
  Serial.println(motorPower);
  Serial.print("type");
  Serial.println(robotType);
  Serial.print("age");
  Serial.println(robotAge);

  // Enable the Tasks
  taskRunner.init();
  taskRunner.addTask(task1); // Add task to scheduler

  task1.enable();
}

/*
   __  __      _      ___   _   _     _        ___     ___    ____
  |  \/  |    / \    |_ _| | \ | |   | |      / _ \   / _ \  |  _ \
  | |\/| |   / _ \    | |  |  \| |   | |     | | | | | | | | | |_) |
  | |  | |  / ___ \   | |  | |\  |   | |___  | |_| | | |_| | |  __/
  |_|  |_| /_/   \_\ |___| |_| \_|   |_____|  \___/   \___/  |_|
*/
void loop() {
  // The main looping code, controls driving and any actions during a game
  // put your main code here, to run repeatedly:
  taskRunner.execute();

  Usb.Task();

  if (usbConnected) {
    // Get controller values
    // Motor 1 = Right
    // Motor 2 = Left
    leftJStickY = PS3.getAnalogHat(LeftHatY);
    rightJStickX = -127 + PS3.getAnalogHat(RightHatX); //This helps zeros the input value

    // Drive Straight
    rightMotorValue = leftJStickY;  //left joystick controllers the right and left moter
    rightMotorValue -= 127;         // Bumps the value into the correct range for the motor to read the input
    if (rightMotorValue > 127) {
      rightMotorValue = 127;
    } else if (rightMotorValue < -127) {
      rightMotorValue = -127;
    } else if (rightMotorValue > -5 && rightMotorValue < 5) { //To help make sure the robot doesn't move when the joystick is at rest
      rightMotorValue = 0;
    }
    leftMotorValue = rightMotorValue;

    //Turn
    if (rightMotorValue < 0) {
      dir = 1;
      //We are driving forwards so calculate the turns that way
      if (rightJStickX < -5) {
        //Turn Left while going forwards
        leftMotorValue -= rightJStickX;
      } else if (rightJStickX > 5) {
        //Turn right while going forwards
        rightMotorValue += rightJStickX - 1;
      }
    } else if (rightMotorValue > 0) {
      dir = -1;
      //We are driving backwards so calculate the turns that way
      if (rightJStickX < -5) {
        //Turn Left while going backwards
        leftMotorValue += rightJStickX - 1;
      } else if (rightJStickX > 5) {
        //Turn Right while going backwards
        rightMotorValue -= rightJStickX;
      }
    } else {
      dir = 0;
      //Sitting Still and turning so both motors can work together more effectively
      if (rightJStickX < -5) {
        //Turn Left
        leftMotorValue -= rightJStickX;
        rightMotorValue += rightJStickX;
        percentPower = 1;
      } else if (rightJStickX > 5) {
        rightMotorValue += rightJStickX - 1;
        leftMotorValue -= rightJStickX + 1;
        percentPower = 1;
      }
    }
    //Serial.println(rightMotorValue);
    if ( dir != 0) {
      if (rampETime > 20) {
        if (percentPower + .1 < endPercentPower) {
          percentPower += .1;
          rampETime = 0;

        }


      }

      rampETime = rampETime + 1;
    } else if (rightJStickX < -5 || rightJStickX > 5) {

    } else {

      percentPower = initPercentPower;
      rampETime = 0;
    }

    //Serial.print(rightMotorValue);
    //Serial.print("  ");
    //Serial.println(leftMotorValue);

    // Boost Option
    if (PS3.getButtonPress(R1)) { //Boost
      //Serial.println("Button Clicked");
      oldMotorValueR = percentPower * rightMotorValue * motorPower * 3;
      oldMotorValueL = percentPower * leftMotorValue * motorPower * 3;
    } else if (PS3.getButtonPress(L1)) { //Boost
      //Serial.println("Button Clicked");
      oldMotorValueR = percentPower * rightMotorValue * motorPower * .3;
      oldMotorValueL = percentPower * leftMotorValue * motorPower * .3;
    } else {
      //Serial.println("Button Not Clicked");
      oldMotorValueR = percentPower * rightMotorValue * motorPower;
      oldMotorValueL = percentPower * leftMotorValue * motorPower;
    }

    //Serial.print(oldMotorValueR);
    //Serial.print("  ");
    //Serial.println(oldMotorValueL);

    //PWM convertion

    PWMConvertionFactor = 0.3543307087;

    //right convertion
    if (oldMotorValueR < 0) {
      PWMMotorValueR = 90 + PWMConvertionFactor * oldMotorValueR;
    } else if (oldMotorValueR > 0 ) {
      PWMMotorValueR = 100 + PWMConvertionFactor * oldMotorValueR;
    } else {
      PWMMotorValueR = 93;
    }

    //left convertion
    if (oldMotorValueL < 0) {
      PWMMotorValueL = 90 + PWMConvertionFactor * oldMotorValueL;
    } else if (oldMotorValueL > 0 ) {
      PWMMotorValueL = 100 + PWMConvertionFactor * oldMotorValueL;
    } else {
      PWMMotorValueL = 93;
    }

    //Serial.print(PWMMotorValueR);
    //Serial.print("  ");
    //Serial.println(PWMMotorValueL);

    //Run Motors
    M1.write(PWMMotorValueR);
    M2.write(PWMMotorValueL);
    //Serial.println(rampETime);

    preDir = dir; // store last movement direction
    //Serial.println(motorValue);
  } else {
    //If controller is disconnected stop motors (Safety)
    ST.motor(1, 0);
    ST.motor(2, 0);
  }


  /*
      ___    _   _      _      ____    _____   _____   ____    ____       _       ____   _  __   ____         ___
     / _ \  | | | |    / \    |  _ \  |_   _| | ____| |  _ \  | __ )     / \     / ___| | |/ /  |___ \       / _ \
    | | | | | | | |   / _ \   | |_) |   | |   |  _|   | |_) | |  _ \    / _ \   | |     | ' /     __) |     | | | |
    | |_| | | |_| |  / ___ \  |  _ <    | |   | |___  |  _ <  | |_) |  / ___ \  | |___  | . \    / __/   _  | |_| |
    \__\_\   \___/  /_/   \_\ |_| \_\   |_|   |_____| |_| \_\ |____/  /_/   \_\  \____| |_|\_\  |_____| (_)  \___/
  */

//   //Step 1: Verify that this robot actually is the QB
//   if (robotType == quarterback) {

//     //Aim Control
//     if (PS3.getButtonClick(CIRCLE)) { // MANUAL RESET
//       ElevationMotor.write(getSpeedDown());
//       currentElevation = low;
//       targetElevation = low;
//       delay(11000);
//       ElevationMotor.write(getSpeedStop());
//     } else if (!aiming) {
//       if (PS3.getButtonClick(UP)) {
//         targetElevation = high;
//         //debug_showSelectionInfo(currentElevation, targetElevation);
//       } else if (PS3.getButtonClick(LEFT) || PS3.getButtonClick(RIGHT)) {
//         targetElevation = middle;
//         //debug_showSelectionInfo(currentElevation, targetElevation);
//       } else if (PS3.getButtonClick(DOWN)) {
//         targetElevation = low;
//         //debug_showSelectionInfo(currentElevation, targetElevation);
//       }
//       if (targetElevation != currentElevation) {
//         QBAimTimestamp = millis();
//         aim(targetElevation - currentElevation);
//         aiming = true;
//       }
//     } else if (aiming) {
//       //debug_showTime();
//       if((millis() - QBAimTimestamp) > getAimBenchmark()) {
//         currentElevation = targetElevation;
//         stopAiming();
//         aiming = false;
//       }
//     }


//     //Flywheel Control Toggle
//     if (flywheelToggleCooldown <= millis()) {
//       if (!flywheelOn) {
//         if (PS3.getButtonClick(X)) { //flywheels on fast
//           startFlywheel(rightFlywheelMotor, leftFlywheelMotor);
//           flywheelOn = true;
//           flywheelToggleCooldown = millis() + BASE_TOGGLE_CD;
//         } else if (PS3.getButtonClick(SQUARE)) { //flywheels on slow for pass
//           passBall(rightFlywheelMotor, leftFlywheelMotor);
//           flywheelOn = true;
//           flywheelToggleCooldown = millis() + BASE_TOGGLE_CD;
//         }
//       } else if (flywheelOn) {
//         if (PS3.getButtonClick(X) || PS3.getButtonClick(SQUARE)) {
//           stopFlywheel(rightFlywheelMotor, leftFlywheelMotor);
//           flywheelOn = false;
//         }
//       }
//     }

//     //Firing the Weapon
//     if (PS3.getAnalogButton(R2) || PS3.getAnalogButton(L2)) {
//       //They want to move the motor
//       if (PS3.getAnalogButton(R2)) {
//         fireWeapon(FireMotor, "Fire");
//       } else if (PS3.getAnalogButton(L2)) {
//         fireWeapon(FireMotor, "Retract");
//       }
//     } else {
//       fireWeapon(FireMotor, "Stop");
//     }
//   }

//   /*
//        ____   _____   _   _   _____   _____   ____
//      / ___| | ____| | \ | | |_   _| | ____| |  _ \
//     | |     |  _|   |  \| |   | |   |  _|   | |_) |
//     | |___  | |___  | |\  |   | |   | |___  |  _ <
//      \____| |_____| |_| \_|   |_|   |_____| |_| \_\
//   */

//   //Version: 2.0
//   //Step 1: Verify that this robot actually is the Center
//   if (robotType == center) {

//     //Raise the Center Arm
//     if (CenterArmMoveStatus == "" && (PS3.getButtonPress(UP)) && CenterArmStatus != 1) {
//       //The user wants to raise the Center arm up
//       CenterArmMoveStatus = "High";
//       CenterArmMoveTimer = millis();
//       CenterArmStatus = centerUp(ST, CenterArmMoveStatus);
//     } else if (CenterArmMoveStatus == "High" && (millis() - CenterArmMoveTimer) > 5000) {
//       CenterArmMoveStatus = "Stop";
//       centerUp(ST, CenterArmMoveStatus);
//       CenterArmMoveStatus = "";
//     }

//     //Lower the Center Arm
//     if (CenterArmMoveStatus == "" && (PS3.getButtonPress(DOWN)) && CenterArmStatus != 2) {
//       //The user wants to raise the Center arm up
//       CenterArmMoveStatus = "Lower";
//       CenterArmMoveTimer = millis();
//       CenterArmStatus = centerDown(ST, CenterArmMoveStatus);
//     } else if (CenterArmMoveStatus == "Lower" && (millis() - CenterArmMoveTimer) > 5000) {
//       CenterArmMoveStatus = "Stop";
//       centerUp(ST, CenterArmMoveStatus);
//       CenterArmMoveStatus = "";
//     }


//   }

}