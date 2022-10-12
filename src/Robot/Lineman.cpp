#include <Robot/Lineman.h>

void Lineman::initialize() {
    this->setDrive(new Drive(3,5));
}

void Lineman::action() {
    Serial.println("lineman is a go");
}