#include "Robot/Lineman.h"

// Lineman::~Lineman() {}

void Lineman::initialize() {
    // this->setDrive(new Drive(3,5));
    Serial.println(F("Lineman initialize"));
}

void Lineman::action() {
    Serial.println(F("Lineman action executed"));
}