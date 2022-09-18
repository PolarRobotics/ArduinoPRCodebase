#include <Arduino.h>
#include <PolarRobotics.h>
#include <Robot/Robot.h>
#include <Robot/Lineman.h>
#include <Robot/Quarterback.h>
#include <EEPROM.h>
#include <SPI.h>
#include <string.h>

// Variables

// Using dynamic storage duration for long term usage
// This is a pointer/reference
Robot* robot;

void setup() {
    Serial.begin(115200);

    /* * * * * * * * * * * * * *
    * Robot Type Determination *
    * * * * * * * * * * * * * */

    uint8_t type = EEPROM.read(1);
    Serial.println("EEPROM: " + static_cast<String>(type));
    TYPE tempType = (TYPE) type;
    
    switch (tempType) {
        case quarterback:
            Serial.println("Robot Type: Quarterback");
            robot = new Quarterback();
            break;
        case lineman:
        default:
            // Assume lineman
            Serial.println("Robot Type: Lineman");
            robot = new Lineman();
    }
}

void loop() {
    robot->getDrive()->iterate(); // Does the driving
    robot->action(); // Performs any special actions
}