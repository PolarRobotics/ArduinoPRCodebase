
#include "Drive/Drive.h"
#include <Arduino.h>
#include <Servo.h> //Built in

/*

Class Drive:
Implements robot drive functions.
Base class for specialized drivebases.

*/


/* Drive constructor for the drive class
    Consolidates code associated with driving the robots

    @todo this class should store all variables and constants related to the drive code,
    other than controller stuff
*/
Drive::Drive(int leftmotorpin, int rightmotorpin) {
    M1.attach(leftmotorpin);
    M2.attach(rightmotorpin);
}

void Drive::setStickPwr(uint8_t leftY, uint8_t rightX) {
    stickForwardPower = (0 - (leftY / 127.0 - 1)); // +: forward, -: backward. needs to be negated so that forward is forward and v.v. subtracting 1 bumps into correct range
    stickTurnPower = (rightX / 127.0 - 1); // +: right turn, -: left turn. subtracting 1 bumps into correct range

    // stick deadzones
    // set to zero (no input) if within the set deadzone
    if (fabs(stickForwardPower) < STICK_DEADZONE) {
      stickForwardPower = 0;
    }
    if (fabs(stickTurnPower) < STICK_DEADZONE) {
      stickTurnPower = 0;
    }
}

// mtr: pass 0 for left and 1 for right
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

/*  generateTurnScalar takes the input stick power and scales the max turning power allowed with the forward power input


*/

void Drive::generateTurnScalar() {
    if(stickForwardPower > 0 && stickTurnPower > 0) {

    }


}

/* float Convert2PWMVal normalizes the signed power value from the ramp function to an unsigned value that the servo function can take

    input: a float containing a value with a range of [-1, 1]
    
    output: a float containing a value between 1000 to 2000
    
    @param rampPwr : float the value to be normalized
    @return normalized PWM value
*/
float Drive::Convert2PWMVal(float rampPwr) {
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
}
