
#include "Drive/Drive.h"
#include <Arduino.h>
#include <Servo.h> //Built in


/** Drive constructor for the drive class
 *  Implements robot drive functions.
 *   Base class for specialized drivebases.
 *
 *    Consolidates code associated with driving the robots
 *
 *    @todo this class should store all variables and constants related to the drive code,
 *    other than controller stuff
*/
Drive::Drive(int leftmotorpin, int rightmotorpin) {
    M1.attach(leftmotorpin);
    M2.attach(rightmotorpin);
}


void Drive::setStickPwr(uint8_t leftY, uint8_t rightX) {
    stickForwardRev = (0 - (leftY / 127.0 - 1)); // +: forward, -: backward. needs to be negated so that forward is forward and v.v. subtracting 1 bumps into correct range
    stickTurn = (rightX / 127.0 - 1); // +: right turn, -: left turn. subtracting 1 bumps into correct range

    // stick deadzones
    // set to zero (no input) if within the set deadzone
    if (fabs(stickForwardRev) < STICK_DEADZONE) {
      stickForwardRev = 0;
    }
    if (fabs(stickTurn) < STICK_DEADZONE) {
      stickTurn = 0;
    }
}


/**  generateTurnScalar takes the input stick power and scales the max turning power allowed with the forward power input
 *    @authors Grant Brautigam, Rhys Davies
 *    Created: 9-12-2022
 *
*/
void Drive::generateMotionValues() {
    // bool fwdPositive = (stickForwardRev > 0);
    bool trnPositive = (stickTurn > 0);
    
    // dont move the motors if there is no input
    if(stickForwardRev == 0 && stickTurn == 0) 
        motorPower[0] = 0, motorPower[1] = 0;

    // move both motors forward if the left stick is between 0 and 1 AND the right stick is 0
    else if(stickForwardRev <= 1 && stickForwardRev > 0 && stickTurn == 0) 
        motorPower[0] = 1, motorPower[1] = 1;
    
    // move both motors in reverse if the left stick is between -1 and 0 AND the right stick is 0
    else if(stickForwardRev >= -1 && stickForwardRev < 0 && stickTurn == 0)
        motorPower[0] = -1, motorPower[1] = -1;

    // tankmode turn right if the left stick is 0 AND the right stick is between 0 and 1
    else if(stickForwardRev == 0 && stickTurn <= 1 && stickTurn > 0)
        motorPower[0] = 1, motorPower[1] = -1;

    // tankmode turn left if the left stick is 0 AND the right stick is between -1 and 0
    else if(stickForwardRev == 0 && stickTurn >= -1 && stickTurn < 0)
        motorPower[0] = -1, motorPower[1] = 1;

    /*
    if the sticks are not in any of the edge cases tested for above (when both sticks are not 0),
    a value must be calculated to determine how much to turn the motor that is doing the turning.
    i.e.: if the user moves the left stick all the way forward (stickFwdRev = 1), and they are attempting
    to turn right. The left motor should get set to 1 and the right motor should get set to 
    some value less than 1
    */
    else {
        if(trnPositive) { // turn Right
            //shorthand if else: variable = (condition) ? expressionTrue : expressionFalse;
            motorPower[0] = stickForwardRev;// set the left motor
            motorPower[1] = copysign(calcTurningMotorValue(abs(stickTurn), lastRampPower[1]), stickForwardRev); // set the right motor
        } else if(!trnPositive) { // turn Left
            motorPower[0] = copysign(calcTurningMotorValue(abs(stickTurn), lastRampPower[0]), stickForwardRev); // set the left motor
            motorPower[1] = stickForwardRev; // set the right motor
        }
    }
}


/**
 * @brief getTurningMotorValue generates a value to be set to the turning motor, the motor that corresponds to the direction of travel,
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
    return pow(sticktrn * (1 - OFFSET) * prevpwr, 2) + (1-stickTurn) * prevpwr;
}


/**
 * @brief ramp
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


/** float Convert2PWMVal normalizes the signed power value from the ramp function to an unsigned value that the servo function can take
 *    @authors Grant Brautigam, Alex Brown  
 *   Updated: 9-13-2022
 *
 *    @param rampPwr the value to be normalized. Hopefully a value between [-1, 1]
 *    @return normalized PWM value (between 1000 to 2000)
*/
float Drive::Convert2PWMVal(float rampPwr) {
    // Original Function: to be removed later.
    /*
    float temp_PWMVal;
    if (rampPwr < 0) {
        // temp_PWMVal = map(rampPwr, 0, 1, 96, 120);
        temp_PWMVal = 90 + PWM_CONVERSION_FACTOR * 127 * rampPwr; // maybe we should use the map function instead of this???
    } else if (rampPwr > 0 ) {
        // temp_PWMVal = map(rampPwr, -1, 0, 40, 90);
        temp_PWMVal = 100 + PWM_CONVERSION_FACTOR * 127 * rampPwr;
    } else {
        temp_PWMVal = 93;
    }
    return temp_PWMVal;
    */

    //original variable's range = [-1,1]
    //converted variable's range = [1000, 2000]
    return map(rampPwr, -1, 1, 1000, 2000);
}

/** float getMotorPwr returns the stored motor value in the class
 * @param mtr the motor number to get, an array index, so 0 -> mtr 1, etc...
 * @return returns the stored motor power for a given motor
*/
float Drive::getMotorPwr(uint8_t mtr) {
    return motorPower[mtr];
}

void Drive::update() {
    for (int i = 0; i < NUM_MOTORS; i++) {

    }
    
    generateTurnScalar()
    
    motorPower[0] = 
    motorPower[1] =

    motorPower[0] = ramp()
    motorPower[1] = ramp()

    lastRampPower[0] = motorPower[0];
    lastRampPower[1] = motorPower[1];

    M1.writeMicroseconds(Convert2PWMVal(motorPower[0]));
    M2.writeMicroseconds(Convert2PWMVal(motorPower[1]));
}
