#include <Robot/Quarterback.h>

void Quarterback::initialize() {
    // this->setDrive(new Drive(3,5));
    Serial.println(F("Creating QB"));
}

void Quarterback::action() {
    Serial.println(F("Actual QB Action Executed"));
}