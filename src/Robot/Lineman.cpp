#include <Robot/Lineman.h>

// Lineman::~Lineman() {}

void Lineman::initialize() {
    // this->setDrive(new Drive(3,5));
    Serial.println("test");
}

void Lineman::action() {
    Serial.println("lineman is a go");
}