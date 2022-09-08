
#include <Arduino.h>
#include <SPI.h> //Built in
#include <EEPROM.h> //Built in
#include <Servo.h> //Built in
#include <SabertoothSimplified.h> //included in PolarRobotics/lib
#include <Sabertooth.h>
#include <PS3BT.h>
#include <SoftwareSerial.h> 
#include <TaskScheduler.h>
#include "PolarRobotics.h"

Drive Robot;
USB Usb;            // There is a USB port
BTD Btd(&Usb);      // The Location of the Bluetooth port
PS3BT PS3(&Btd);    // There is a PS3 Controller at the location of the Bluetooth port
Servo servoObj;   // Initialize a servo object for the connected servo

Servo M1; //initialize Motor 1 as a servo
Servo M2; //initialize Motor 2 as a servo

Servo flywheels;
Servo conveyor;

SabertoothSimplified ST; // We'll name the Sabertooth object ST.

// Pin and Servo for quarterback flywheels
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

// Drive V2 Constants
const float  // should be a value from (-1, 1) but close to zero
             BOOST_PCT = 1.0, // this is 1.0, the maximum power possible to the motors.
             NORMAL_PCT = 0.4, // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
             SLOW_PCT = 0.15; // should be a value less than NORMAL_PCT, to slow down for precision maneuvering
// const double THRESHOLD = pow(10, -5);

// DO NOT CHANGE THIS CONSTANT UNLESS YOU KNOW WHAT YOU ARE DOING
/*
This one is a bit complicated. It affects a large part of the drive code.
THIS VALUE MUST BE BETWEEN 0 and 1.

1. It factors into the calculation of turnScalar, thus it directly impacts
   how responsive the turning joystick is with increasing forward power.

2. It is the maximum value of powerDelta,
   i.e. the maximum value by which the target powers can exceed 1.

3. It is the forward power below which no forward velocity is considered when turning at maximum velocity.
   i.e. it is a deadzone of sorts.
*/
const double TURN_SCALAR_CONSTANT = 3.0 / 4.0;

// Drive V2 Calculation Variables
float stickForwardPower = 0, stickTurnPower = 0, bsnPct = 0, targetPowerLeft = 0, targetPowerRight = 0, turnScalar = 0, powerDelta = 0;
float scaledBoostedPowerLeft = 0, scaledBoostedPowerRight = 0, rampedPowerLeft = 0, rampedPowerRight = 0, PWMMotorValueR = 0, PWMMotorValueL = 0;

//Position Values of respective joysticks as returned thru USB connection
int leftJStickY = 0, rightJStickX = 0;

// Variable motor power adjuster later in code based on joystick pos and bot type
int rightMotorValue = 0, leftMotorValue = 0;

// Helps us create a time interval so we are not checking on connection status and updating every ms.
// Helps performance as well as eliminates bot glitches from spotty connection
int lastChecked = 0;

// Works with lastChecked to make sure the controller is still connected (Just an added safety feature)
bool usbConnected = false;

// Depends on the type of bot, makes sure the bigger bots get less power and don't rip themselves apart
double percentPower;

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

/* * * * * * * * * * * * * * * * *
 * Quarterback Specific Variables *
 * * * * * * * * * * * * * * * * */

// A timer used to run the motor for a set ammount of time (IMPLEMENT ENCODERS IN V3)
unsigned long QBAimTimestamp = millis();

// Boolean, true if currently aiming/moving the linear actuators, false otherwise
boolean aimingup = false;
boolean aimingdown = false;
boolean flywheelstatis = false;
int flywheelstate = 0;
// Variables of custom enum for keeping track of elevation of linear actuators/aimers
int targetElevation = 0;
int currentElevation = 0;
int counter = 0;

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
  Serial.begin(115200);
  //SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.

  M1.attach(5);
  M2.attach(3);

  flywheels.attach(4);
  conveyor.attach(2);
  
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
    // command to move from 0 degrees to 180 degrees
    for (servoAngle = 0; servoAngle < 180; servoAngle += 1) {
      servoObj.write(servoAngle);                 //command to rotate the servo to the specified angle
      delay(15);
    }

    delay(1000);

    // command to move from 180 degrees to 0 degrees
    for (servoAngle = 180; servoAngle >= 1; servoAngle -= 5) {
      servoObj.write(servoAngle);              //command to rotate the servo to the specified angle
      delay(5);
    }

    delay(1000);

  } else if (robotType == quarterback) {     // Quarterback Setup
    rightFlywheelMotor.attach(rightFlywheelPin);
    leftFlywheelMotor.attach(leftFlywheelPin);
    FireMotor.attach(FirePin);
    ElevationMotor.attach(ElevationPin);
    conveyor.write(30);
    M1.write(93);
    M2.write(93);
  }

  /* * * * * * * * * * * * * *
   * Robot Age Determination *
   * * * * * * * * * * * * * */
  robotAge = EEPROM.read(0);
  //Serial.print(robotAge);
  if (robotAge == OLD) {
    // If it's an old (12V) robot, maximize drive motor power
    motorPower = 1.0;
  }
  else if (robotAge == NEW) {
    // If it's a new (24V) robot, tone down the base drive motor power so they don't do seven wheelies
    motorPower = 0.5; 
  }
  else {
    Serial.print ("Read error on age of robot, assumed new robot");
    motorPower = 0.35;
  }
  /*
  Serial.println(motorPower);
  Serial.print("type");
  Serial.println(robotType);
  Serial.print("age");
  Serial.println(robotAge);*/

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
    Robot.setStickPwr(PS3.getAnalogHat(LeftHatY), PS3.getAnalogHat(RightHatX));
    
    // if the inputs are zero, just stop and don't do any unnecessary calculations
    // helps prevent divide by zero errors
    if (stickForwardPower == 0 && stickTurnPower == 0) {
      rampedPowerLeft = Robot.ramp(0, 0);
      rampedPowerRight = Robot.ramp(0, 1);
    } else {
      float fwdSign;
      if (stickForwardPower == 0) { // this may not be necessary, was a failed hotfix
        fwdSign = 1.0;
      } else {
        fwdSign = copysign(1.0, stickForwardPower);
      }
      
      // determine BSN percentage (boost, slow, or normal)
      if (PS3.getButtonPress(R1)) {
        bsnPct = BOOST_PCT; // Boost, 1.0, max power
      } else if (PS3.getButtonPress(L1)) {
        bsnPct = SLOW_PCT; //Slow
      } else {
        bsnPct = NORMAL_PCT; // Normal Speed
      }

      // generate turn scalar
      float intermediateTurnScalar = (1 + TURN_SCALAR_CONSTANT) - fabs(stickForwardPower);
      turnScalar = constrain(intermediateTurnScalar, -1.0, 1.0) * fwdSign;

      // generate target powers from bsn scalar, manual stick magnitude and turn
      /*
        The value of stickTurnPower is positive if turning right and negative if turning left.
        std::copysign takes the sign of the forward value so that the addition/subtraction is correct for the direction.
        If turning right, the left wheels need to travel further than the right wheels, so we add to left and subtract from right.
        Vice versa for left. This could also be done with an if/else statement but it's more efficient to get the sign and multiply that way.
      */
      targetPowerLeft = (stickForwardPower + (stickTurnPower * turnScalar * fwdSign));
      targetPowerRight = (stickForwardPower - (stickTurnPower * turnScalar * fwdSign));

      // Serial.print("TargetLeft: ");
      // Serial.print(targetPowerLeft);
      // Serial.print("  TargetRight: ");
      // Serial.println(targetPowerRight);

      // if the magnitude of either target power exceeds 1, calculate the difference between it and 1.
      if ((fabs(targetPowerLeft) - 1) > THRESHOLD) {
        powerDelta = 1 - targetPowerLeft;
      } else if ((fabs(targetPowerRight) - 1) > THRESHOLD) {
        powerDelta = 1 - targetPowerRight;
      } else {
        powerDelta = 0;
      }

      // this section keeps the ratio between powers the same when scaling down
      if (stickTurnPower > (0 + THRESHOLD)) {
        scaledBoostedPowerLeft = targetPowerLeft + (fwdSign * powerDelta);
        scaledBoostedPowerRight = targetPowerRight + (fwdSign * powerDelta * (targetPowerRight / targetPowerLeft));
      } else if (stickTurnPower < (0 - THRESHOLD)) {
        scaledBoostedPowerLeft = targetPowerLeft + (fwdSign * powerDelta * (targetPowerLeft / targetPowerRight));
        scaledBoostedPowerRight = targetPowerRight + (fwdSign * powerDelta);
      } else {
        scaledBoostedPowerLeft = targetPowerLeft;
        scaledBoostedPowerRight = targetPowerRight;
      }
    
      // Serial.print("  CBP_Left: ");
      // Serial.print(scaledBoostedPowerLeft);
      // Serial.print("  CBP_Right: ");
      // Serial.print(scaledBoostedPowerRight);

      // bsnPct is determined above and changes based on manual control.
      scaledBoostedPowerLeft *= bsnPct;
      scaledBoostedPowerRight *= bsnPct;

      Serial.print("  BSN_Scaled_Left: ");
      Serial.print(scaledBoostedPowerLeft);
      Serial.print("  BSN_Scaled_Right: ");
      Serial.println(scaledBoostedPowerRight);

      // call ramp function from polar robotics library and...
      // scale new drive values up to old drive values.
      // pass 0 to ramp for left and 1 for right
      rampedPowerLeft = Robot.ramp(scaledBoostedPowerLeft, 0);
      rampedPowerRight = Robot.ramp(scaledBoostedPowerRight, 1);

      // Serial.print("  RampedScaled_Power_Left: ");
      // Serial.print(rampedPowerLeft);

      // Serial.print("  RampedScaled_Power_Right: ");
      // Serial.print(rampedPowerRight);

      // if (abs(rampedPowerLeft) < THRESHOLD) { rampedPowerLeft = 0.0; }
      // if (abs(rampedPowerRight) < THRESHOLD) { rampedPowerRight = 0.0; }
    }

    // Serial.print("BSN_Left: ");
    // Serial.print(rampedPowerLeft);
    // Serial.print("  BSN_Right: ");
    // Serial.print(rampedPowerRight);

    //PWM conversions
    //right conversion
    PWMMotorValueR = Robot.Convert2PWMVal(rampedPowerRight);
    //left conversion
    PWMMotorValueL = Robot.Convert2PWMVal(rampedPowerLeft);

    // Serial.print("  PWM_Left: ");
    // Serial.print(PWMMotorValueL);
    // Serial.print("  PWM_Right: ");
    // Serial.println(PWMMotorValueR);

    //Run Motors
    M1.write(PWMMotorValueR);
    M2.write(PWMMotorValueL);
  } else {
    //If controller is disconnected stop motors (Safety)
    M1.write(93);
    M2.write(93);
  }

  /*
       ___    _   _      _      ____    _____   _____   ____    ____       _       ____   _  __   ____         ___
      / _ \  | | | |    / \    |  _ \  |_   _| | ____| |  _ \  | __ )     / \     / ___| | |/ /  |___ \       / _ \
     | | | | | | | |   / _ \   | |_) |   | |   |  _|   | |_) | |  _ \    / _ \   | |     | ' /     __) |     | | | |
     | |_| | | |_| |  / ___ \  |  _ <    | |   | |___  |  _ <  | |_) |  / ___ \  | |___  | . \    / __/   _  | |_| |
      \___\_\ \___/  /_/   \_\ |_| \_\   |_|   |_____| |_| \_\ |____/  /_/   \_\  \____| |_|\_\  |_____| (_)  \___/
  */

  //Step 1: Verify that this robot actually is the QB

  if (robotType == quarterback) {    
    // if (PS3.getButtonClick(TRIANGLE) && targetElevation + 1 < 3) {
    //   targetElevation = targetElevation + 1;
    // } else if (PS3.getButtonClick(CROSS) && targetElevation - 1 >= 0) {
    //   targetElevation = targetElevation - 1;
    // } else if (PS3.getButtonClick(DOWN)) {
    //   targetElevation = 0;
    // }

    // int maxCounter = 13000;

    // if ((counter > maxCounter || counter < 0) && (aimingup == true || aimingdown == true)) {
    //   aimingup = false;
    //   aimingdown = false;
    //   ElevationMotor.write(getSpeedStop());
    //   if (counter == -1) {
    //     counter = 0;
    //   } else {
    //     counter = maxCounter;
    //   }
    //   Serial.println("im stuck");
    // } else if (targetElevation > currentElevation) {
    //   ElevationMotor.write(getSpeedUp());
    //   currentElevation = targetElevation;
    //   aimingup = true;
    //   aimingdown = false;
    // } else if (targetElevation < currentElevation) {
    //   ElevationMotor.write(getSpeedDown());
    //   currentElevation = targetElevation;
    //   aimingdown = true;
    //   aimingup = false;
    // } else if (aimingup == true) {
    //   counter = counter + 1;
    // } else if (aimingdown == true) {
    //   counter = counter - 1;
    // }

    // if (targetElevation == 1 && counter == maxCounter/2) {
    //   aimingup = false;
    //   aimingdown = false;
    //   ElevationMotor.write(getSpeedStop());
    // }

    // if (PS3.getButtonClick(LEFT)) {
    //   ElevationMotor.write(getSpeedDown());
    //   delay(8000);
    //   ElevationMotor.write(getSpeedStop());
    //   counter = 0;
    //   currentElevation = 0;
    //   targetElevation = 0;
    // }

    // if (PS3.getButtonPress(CIRCLE)) {
    //   conveyor.write(145);
    // } else {
    //   conveyor.write(30);
    // }

    // // flywheels.write(60);
    
    // if (PS3.getButtonClick(SQUARE)) {
    //   flywheelstate = flywheelstate + 1;
    //   if (flywheelstate == 1) {
    //     flywheels.write(100);
    //     //flywheelstatis = true;
    //     //Serial.print("ran line 1");
    //     //Serial.println("  ");
    //   } else if (flywheelstate == 2) {
    //     flywheels.write(145);
    //     //flywheelstatis = true;
    //     //Serial.print("ran line 1");
    //     //Serial.println("  ");
    //   } else if (flywheelstate==3){
    //     flywheels.write(93);
    //     //flywheelstatis = false;
    //     flywheelstate = 0;
    //     //Serial.print("ran line 2");
    //     //Serial.println("  ");
    //   }
    //     //Serial.print("ran line 3");
    //     //Serial.println("  ");
    // }
  }

  /*
       ____   _____   _   _   _____   _____   ____
      / ___| | ____| | \ | | |_   _| | ____| |  _ \
     | |     |  _|   |  \| |   | |   |  _|   | |_) |
     | |___  | |___  | |\  |   | |   | |___  |  _ <
      \____| |_____| |_| \_|   |_|   |_____| |_| \_\
  */

  //Version: 2.0
  //Step 1: Verify that this robot actually is the Center
  if (robotType == center) {
    // //Raise the Center Arm
    // if (CenterArmMoveStatus == "" && (PS3.getButtonPress(UP)) && CenterArmStatus != 1) {
    //   //The user wants to raise the Center arm up
    //   CenterArmMoveStatus = "High";
    //   CenterArmMoveTimer = millis();
    //   CenterArmStatus = centerUp(ST, CenterArmMoveStatus);
    // } else if (CenterArmMoveStatus == "High" && (millis() - CenterArmMoveTimer) > 5000) {
    //   CenterArmMoveStatus = "Stop";
    //   centerUp(ST, CenterArmMoveStatus);
    //   CenterArmMoveStatus = "";
    // }

    // //Lower the Center Arm
    // if (CenterArmMoveStatus == "" && (PS3.getButtonPress(DOWN)) && CenterArmStatus != 2) {
    //   //The user wants to raise the Center arm up
    //   CenterArmMoveStatus = "Lower";
    //   CenterArmMoveTimer = millis();
    //   CenterArmStatus = centerDown(ST, CenterArmMoveStatus);
    // } else if (CenterArmMoveStatus == "Lower" && (millis() - CenterArmMoveTimer) > 5000) {
    //   CenterArmMoveStatus = "Stop";
    //   centerUp(ST, CenterArmMoveStatus);
    //   CenterArmMoveStatus = "";
    // }
  }
}
