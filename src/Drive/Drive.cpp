
#include "Drive/Drive.h"
#include <Arduino.h>
#include <Servo.h> //Built in
// #include <Servo_Hardware_PWM.h>

/** 
 * @brief Drive Class, base class for specialized drive classes, this configuration is intended for the standard linemen.
 * this class takes the the the stick input, scales the turning value for each motor and ramps that value over time,
 * then sets the ramped value to the motors
 * @authors Rhys Davies (@rdavies02), Max Phillips (@RyzenFromFire) 
 * 
 * @class
 *    2 motor configuration shown below  
 * 
 *               ^
 *               | Fwd
 *       _________________        
 *      |        _        |       
 *      |       |O|       |       O: represents the Omniwheel, a wheel that can turn on 2 axis       
 *      |       |_|       |       L: represents the left Wheel, powered by the left motor via a chain
 *      |  _           _  |            - the left motor would turn ccw to move the bot forward
 *      | |L|         |R| |       R: represents the right Wheel, powered by the right motor via a chain
 *      | |_|         |_| |            - the right motor would turn cw to move the bot forward
 *      |_________________|
 * 
 * @todo
 *  - add a turning radius parameter, needed for the kicker
 *  - add mechanium driving code, for the new center, needed next semester (Spring 2023)
 * 
 * Default configuration:
 * @param leftmotorpin the arduino pin needed for the left motor, needed for servo
 * @param rightmotorpin the arduino pin needed for the right motor, needed for servo
*/
Drive::Drive(int leftmotorpin, int rightmotorpin) {
    // M1.attach(leftmotorpin, 1000, 2000);
    // M2.attach(rightmotorpin, 1000, 2000);
    motorPins[0] = leftmotorpin;
    motorPins[1] = rightmotorpin;
    pinMode(leftmotorpin, OUTPUT);
    pinMode(rightmotorpin, OUTPUT);
}

/**
 * setStickPwr takes the stick values passed in and normalizes them to values between -1 and 1
 * and sets this value to the private variables stickFwdRev and stickTurn respectively
 * @author Rhys Davies
 * Created: 9-12-2022
 *   
 * @param leftY the forward backward value from the left stick (0 to 255)
 * @param rightX the left right value from the right stick (0 to 255)
*/
void Drive::setStickPwr(uint8_t leftY, uint8_t rightX) {
    // left stick all the way foreward is 0, backward is 255 
    stickForwardRev = (0 - (leftY / 127.5 - 1)); // +: forward, -: backward. needs to be negated so that forward is forward and v.v. subtracting 1 bumps into correct range
    stickTurn = (rightX / 127.5 - 1); // +: right turn, -: left turn. subtracting 1 bumps into correct range

    // stick deadzones
    // set to zero (no input) if within the set deadzone
    if (fabs(stickForwardRev) < STICK_DEADZONE) {
      stickForwardRev = 0;
    }
    if (fabs(stickTurn) < STICK_DEADZONE) {
      stickTurn = 0;
    }
    if (stickForwardRev > 1) stickForwardRev = 1;
    if (stickForwardRev < -1) stickForwardRev = -1;
    if (stickTurn > 1) stickTurn = 1;
    if (stickTurn < -1) stickTurn = -1;
}

/**
 * @brief setBSN sets the internal variable to the requested percent power, this is what the motor power gets multiplied by, 
 * this is where the boost, slow and normal scalars get passed in 
 * @author Rhys Davies
 * Created: 9-12-2022
 * 
 * @param bsn input speed choice Drive::Boost, Drive::Slow, Drive::Normal
*/
void Drive::setBSN(SPEED bsn) {
    // set the scalar to zero if the requested value is greater than 1, this is not entirely necessary, but is a safety
    // BSNscalar = (powerMultiplier > 1) ? 0 : powerMultiplier;
    switch (bsn) {
        case boost: {
            BSNscalar = BOOST_PCT;
            break;
        }
        case normal: {
            BSNscalar = NORMAL_PCT; 
            break;
        }
        case slow: {
            BSNscalar = SLOW_PCT;
            break;
        }
    }
}


/** 
 * generateTurnScalar takes the input stick power and scales the max turning power allowed with the forward power input
 * @authors Grant Brautigam, Rhys Davies
 * Created: 9-12-2022
*/
void Drive::generateMotionValues() {
    // bool fwdPositive = (stickForwardRev > 0);
    bool trnPositive = (stickTurn > 0);

    // dont move the motors if there is no input
    if(stickForwardRev == 0 && stickTurn == 0) 
        motorPower[0] = 0, motorPower[1] = 0;

    // move both motors forward if the left stick is between 0 and 1 AND the right stick is 0
    else if(stickForwardRev >= -1 && stickForwardRev <= 1 && stickTurn == 0) { 
        motorPower[0] = BSNscalar * stickForwardRev; 
        motorPower[1] = BSNscalar * stickForwardRev;
    }
    // move both motors in reverse if the left stick is between -1 and 0 AND the right stick is 0
    // else if(stickForwardRev >= -1 && stickForwardRev < 0 && stickTurn == 0) {
    //     motorPower[0] = -BSNscalar;
    //     motorPower[1] = -BSNscalar;

    // }

    // tankmode turn right if the left stick is 0 AND the right stick is between 0 and 1
    else if(stickForwardRev == 0 && stickTurn <= 1 && stickTurn > 0 && lastTurnPwr == 0) {
        motorPower[0] = BSNscalar * abs(stickTurn);
        motorPower[1] = -BSNscalar * abs(stickTurn);
    }
        
    // tankmode turn left if the left stick is 0 AND the right stick is between -1 and 0
    else if(stickForwardRev == 0 && stickTurn >= -1 && stickTurn < 0 && lastTurnPwr == 0) {
        motorPower[0] = -BSNscalar * abs(stickTurn);
        motorPower[1] = BSNscalar * abs(stickTurn);

    }

    /*
    if the sticks are not in any of the edge cases tested for above (when both sticks are not 0),
    a value must be calculated to determine how to scale the motor that is doing the turning.
    i.e.: if the user moves the left stick all the way forward (stickFwdRev = 1), and they are attempting
    to turn right. The left motor should get set to 1 and the right motor should get set to 
    some value less than 1, this value is determined by the function calcTurningMotorValue
    */
    else {
        if(trnPositive) { // turn Right
            //shorthand if else: variable = (condition) ? expressionTrue : expressionFalse;
            motorPower[0] = stickForwardRev * BSNscalar;// set the left motor
            motorPower[1] = calcTurningMotorValue(stickForwardRev, lastRampPower[0]); // set the right motor
        } else if(!trnPositive) { // turn Left
            motorPower[0] = calcTurningMotorValue(stickForwardRev, lastRampPower[1]); // set the left motor
            motorPower[1] = stickForwardRev * BSNscalar; // set the right motor
        }
    }
}


/**
 * @brief getTurningMotorValue generates a value to be set to the turning motor, the motor that corresponds to the direction of travel
 * @authors Grant Brautigam, Rhys Davies
 * Created: 9-12-2022
 * 
 * Mathematical model:
 *  TurningMotor = TurnStickNumber(1-offset)(CurrentPwrFwd)^2+(1-TurnStickNumber)*CurrentPwrFwd
 *   *Note: CurrentPwrFwd is the current power, not the power from the stick
 * 
 * @param sticktrn the absoulte value of the current turning stick input
 * @param prevpwr the motor value from the previous loop
 * @return float - the value to get set to the turning motor (the result of the function mention above)
 */
float Drive::calcTurningMotorValue(float sticktrn, float prevpwr) {
    float temp = abs(sticktrn) * (1 - OFFSET) * pow(prevpwr, 2) + (1-abs(stickTurn)) * abs(prevpwr);
    temp = copysign(temp, prevpwr);
    lastTurnPwr = temp;
    return temp;
}


/**
 * @brief ramp
 * @authors Max Phillips, Grant Brautigam
 * Created: early 2022
 * 
 * @param requestedPower 
 * @param mtr pass 0 for left and 1 for right, used to help ease with storing values for multiple motors 
 * @return float 
 */
float Drive::ramp(float requestedPower, uint8_t mtr) {
    // Serial.print("  lastRampTime ");
    // Serial.print(lastRampTime[mtr]);
    // Serial.print("  requestedPower ");
    // Serial.print(requestedPower);
    // Serial.print("  current ");
    // Serial.print(currentPower[mtr]);
    // Serial.print("  requestedPower - currentPower ");
    // Serial.println(requestedPower - currentPower[mtr], 10);
    if (millis() - lastRampTime[mtr] >= TIME_INCREMENT) {
        if (abs(requestedPower) < THRESHOLD) { // if the input is effectively zero
        // Experimental Braking Code
            if (abs(currentPower[mtr]) < 0.1) { // if the current power is very small just set it to zero
                currentPower[mtr] = 0;
            }
            else {
                currentPower[mtr] *= BRAKE_PERCENTAGE;
            }
            //currentPower[mtr] = 0;
            lastRampTime[mtr] = millis();
        }
        else if (abs(requestedPower - currentPower[mtr]) < ACCELERATION_RATE) { // if the input is effectively at the current power
            return requestedPower;
        }
        else if (requestedPower > currentPower[mtr]) { // if we need to increase speed
            currentPower[mtr] = currentPower[mtr] + ACCELERATION_RATE;
            lastRampTime[mtr] = millis();
        }
        else if (requestedPower < currentPower[mtr]) { // if we need to decrease speed
            currentPower[mtr] = currentPower[mtr] - ACCELERATION_RATE;
            lastRampTime[mtr] = millis();
        }
    }
    return currentPower[mtr];
}


/** 
 * normalizes the signed power value from the ramp function to an unsigned value that the servo function can take
 * @authors Rhys Davies, Grant Brautigam, Alex Brown  
 * Updated: 9-13-2022
 *
 * @param rampPwr the value to be normalized. Hopefully a value between [-1, 1]
 * @return normalized PWM value (between 1000 to 2000)
*/
float Drive::Convert2PWMVal(float rampPwr) {
    // Original Function: to be removed later.
    // float temp_PWMVal;
    // if (rampPwr < 0) {
    //     // temp_PWMVal = map(rampPwr, 0, 1, 96, 120);
    //     temp_PWMVal = 90 + PWM_CONVERSION_FACTOR * 127 * rampPwr; // maybe we should use the map function instead of this???
    // } else if (rampPwr > 0 ) {
    //     // temp_PWMVal = map(rampPwr, -1, 0, 40, 90);
    //     temp_PWMVal = 100 + PWM_CONVERSION_FACTOR * 127 * rampPwr;
    // } else {
    //     temp_PWMVal = 93;
    // }
    // return temp_PWMVal;
    

    //original variable's range = [-1,1]
    //converted variable's range = [1000, 2000]
    // multiply the ramp power by 1000 (shift the decimal place over a bit) 
    // to bring the number into a range that map can use
    // return map(rampPwr * 1000, -1000, 1000, 1000, 2000);
    return (-rampPwr + 1) * 500 + 1000;
}

void setMtr(float pwr, byte mtr);

/**
 * returns the stored motor value in the class
 * @param mtr the motor number to get, an array index, so 0 -> mtr 1, etc...
 * @return returns the stored motor power for a given motor
*/
float Drive::getMotorPwr(uint8_t mtr) {
    return motorPower[mtr];
}

void Drive::emergencyStop() {
    M1.writeMicroseconds(1500);
    M2.writeMicroseconds(1500);
    // while(1);
}

/**
 * @brief updates the motors after calling all the functions to generate 
 * turning and scaling motor values, the intention of this is so the 
 * programmer doesnt have to call all the functions, this just handles it,
 * reducing clutter in the main file.
 * DO NOT CALL THIS FUNCTION UNTIL setStickPwr and setBSN have been called before update
 * @author Rhys Davies
 * Created: 9-12-2022
*/
void Drive::update() {    
    // Serial.print("Left Input: ");
    // Serial.print(stickForwardRev);
    // Serial.print("  Right: ");
    // Serial.print(stickTurn);

    // Generate turning motion
    generateMotionValues();
    
    // Serial.print("  |  Turn: ");
    // Serial.print(lastTurnPwr);

    // Serial.print("  |  Left ReqPwr: ");
    // Serial.print(motorPower[0]);
    // Serial.print("  Right ReqPwr: ");
    // Serial.print(motorPower[1]);

    // set the motors to zero if the motor power exceeds the allowable range
    // if (abs(rampPwr + 0.5) > BOOST_PCT) {
    //     return 0;
    // }

    // get the ramp value
    motorPower[0] = ramp(motorPower[0], 0);
    motorPower[1] = ramp(motorPower[1], 1);


    // Set the ramp value to a function, needed for generateMotionValues
    lastRampPower[0] = motorPower[0];
    lastRampPower[1] = motorPower[1];

    // Serial.print("  Left Motor: ");
    // Serial.print(motorPower[0]);
    // Serial.print("  Right: ");
    // Serial.println(motorPower[1]);
    // Write to the motors
    // M1.writeMicroseconds(Convert2PWMVal(motorPower[0]));
    // M2.writeMicroseconds(Convert2PWMVal(motorPower[1]));

    // Serial.print("  |  Left Motor: ");
    // Serial.print(Convert2PWMVal(-motorPower[0]));
    // Serial.print("  Right: ");
    // Serial.println(Convert2PWMVal(motorPower[1]));

    //  M1.writeMicroseconds(Convert2PWMVal(-motorPower[0]));
    //  M2.writeMicroseconds(Convert2PWMVal(motorPower[1]));
    digitalWrite(motorPins[0], HIGH);
    delayMicroseconds(Convert2PWMVal(-motorPower[0])-40 );
    digitalWrite(motorPins[0], LOW);
    delayMicroseconds(2000 - Convert2PWMVal(-motorPower[0])-40 ); //-170
}

//Old functions

// if the magnitude of either target power exceeds 1, calculate the difference between it and 1.
//   if ((fabs(targetPowerLeft) - 1) > THRESHOLD) {
//     powerDelta = 1 - targetPowerLeft;
//   } else if ((fabs(targetPowerRight) - 1) > THRESHOLD) {
//     powerDelta = 1 - targetPowerRight;
//   } else {
//     powerDelta = 0;
//   }


// this section keeps the ratio between powers the same when scaling down
//   if (stickTurnPower > (0 + THRESHOLD)) {
//     scaledBoostedPowerLeft = targetPowerLeft + (fwdSign * powerDelta);
//     scaledBoostedPowerRight = targetPowerRight + (fwdSign * powerDelta * (targetPowerRight / targetPowerLeft));
//   } else if (stickTurnPower < (0 - THRESHOLD)) {
//     scaledBoostedPowerLeft = targetPowerLeft + (fwdSign * powerDelta * (targetPowerLeft / targetPowerRight));
//     scaledBoostedPowerRight = targetPowerRight + (fwdSign * powerDelta);
//   } else {
//     scaledBoostedPowerLeft = targetPowerLeft;
//     scaledBoostedPowerRight = targetPowerRight;
//   }
