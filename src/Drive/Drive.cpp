
#include "Drive/Drive.h"
#include <Arduino.h>
#include <Servo.h> //Built in


/* Drive constructor for the drive class
    **NOTE: Only for two motors, this is a temporary solution, 
    future iterations should use vecotor
    
    Intended to help consolidate code associated 
    with diving the robots

    @todo this class should store all variables and constants related to the drive code, 
    other than controller stuff

    @future manages driving of mechanum wheels as well as normal wheels 
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

    input: not entirely sure what the values are
    
    output: for the 24v motors, the sabertooth takes values roughly between 45-120
        where 45 would be the slowest speed in one direction and 90 would be the max speed in that same direction
        values between 91 and 95 are in the dead zone, no motor spin.
        values between 96 and 120 are the values for the other direction, 96 being the max and 120 being the min
    
    @param rampPwr : float the value to be normalized
    @return normalized PWM value
*/
float Drive::Convert2PWMVal(float rampPwr) {
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
}

/** float getMotorPwr returns the stored motor value in the class
 * @param mtr the motor number to get, an array index, so 0 -> mtr 1, etc...
 * @return returns the stored motor power for a given motor        
*/
float Drive::getMotorPwr(uint8_t mtr) {
    return motorPower[mtr];
}

void Drive::setMotors() {
    for (int i = 0; i < NUM_MOTORS; i++)

}

