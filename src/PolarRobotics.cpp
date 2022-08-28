
#include "PolarRobotics.h"
#include <Arduino.h>
#include <Servo.h> //Built in


/* Drive constructor for the drive class
    @todo add the ability to pass the servo objects in, this class 
    should store all variables and constants related to the drive code

    @future need to rewrite the servo class to work correctly with the sabertooth
*/
Drive::Drive() {
    // !!! DO NOT CHANGE THIS VALUE EVER
    CLOSE_ENOUGH = pow(10, -5);
    // lastRampTime = {0, 0};
    // currentPower = {0, 0};
    // inputPower = {0, 0};
    // rampedPower = {0, 0};
}

// mtr: pass 0 for left and 1 for right
float Drive::ramp(float requestedPower, byte mtr) {
    // Serial.print("millis() ");
    // Serial.print(millis());
    // Serial.print("  lastRampTime ");
    // Serial.print(lastRampTime[mtr]);
    // Serial.print("  requestedPower ");
    // Serial.print(requestedPower);
    // Serial.print("  current ");
    // Serial.print(currentPower[mtr]);
    // Serial.print("  requestedPower - currentPower ");
    // Serial.println(requestedPower - currentPower[mtr], 10);
    if (millis() - lastRampTime[mtr] >= timeIncrement) {
        if (abs(requestedPower) < CLOSE_ENOUGH) { // if the input is effectively zero
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


/*
    class Drive: a class intended to help consolidate code associated 
    with the two motors responsible for driving a robot

*/

// class Drive {
//     private:
//     uint8_t m_Lmotorpin, m_Rmotorpin;
//     bool tankmode;
//     uint8_t m_maxpower;
//     //Servo m_L, m_R;

//     public:
//         Drive(uint8_t Lmotorpin, uint8_t RmotorPin);
//         void forward();
//         void reverse();
//         void left();
//         void right();
// }

// Drive::Drive(uint8_t Lmotorpin, uint8_t RmotorPin) {
//     m_Lmotorpin = Lmotorpin, m_Rmotorpin = RmotorPin;
//     m_L.attach(m_Lmotorpin);
//     m_R.attach(m_Rmotorpin);
// }

// void Drive::forward() {

// }
    
// void Drive::reverse() {

// }

// void Drive::left() {
//     if (tankmode) {

//     }
// }

// void Drive::right() {

// }