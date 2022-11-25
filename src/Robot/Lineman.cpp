#include "Robot/Lineman.h"

Lineman::Lineman() { 
    Serial.println(F("Creating lineman"));
    this->setType(TYPE::lineman);
    this->setDrive(new Drive(this->lPin, this->rPin));
};

void Lineman::initialize() {
    // this->setDrive(new Drive(3,5));
    Serial.println(F("Lineman initialize"));
}

void Lineman::action(PS5BT& PS5) {
    Serial.println(F("Lineman action executed"));
}