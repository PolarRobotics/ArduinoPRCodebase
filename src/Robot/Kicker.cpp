#include "Kicker.h"

Kicker::Kicker() {
    kickerEnabled = false;
    setDrive(new Drive(MOTORS::big)); // TODO: update to use new derived class
    windupMotor.attach(WINDUP_PIN);
}

void Kicker::enable() { kickerEnabled = true; }

void Kicker::disable() { kickerEnabled = false; }

void Kicker::initialize() {
    Serial.println(F("Creating Kicker"));
}

void Kicker::action(PS5BT& PS5) {
    Serial.println(F("Actual Kicker Action Executed"));
    
    // Control the motor on the kicker
    if (PS5.getButtonPress(TRIANGLE))
        windFwd();
    else if (PS5.getButtonPress(CROSS))
        windRev();
    else
        windStop();
}

void Kicker::windFwd() {
    if (kickerEnabled)
        windupMotor.write(WIND_FWD_SPEED);
}

void Kicker::windRev() {
    if (kickerEnabled)
        windupMotor.write(WIND_REV_SPEED);
}

void Kicker::windStop() {
    if (kickerEnabled)
        windupMotor.write(WIND_STOP_SPEED);
}

// TODO: remove if not needed
void Kicker::test() {
    if (kickerEnabled) {
        windupMotor.write(50); //clockwise
        delay(3000);
        windupMotor.write(90); //stop
        delay(1000);
        windupMotor.write(130); //counter-clockwise
        delay(3000);
        windupMotor.write(90); //stop
    }
}