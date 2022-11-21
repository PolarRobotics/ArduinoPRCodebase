#include "Kicker.h"

void Kicker::initialize() {
    Serial.println(F("Creating Kicker"));
}

void Kicker::action(PS5BT& PS5) {
    Serial.println(F("Actual Kicker Action Executed"));
    // Control the motor on the kicker
    if (PS5.getButtonPress(TRIANGLE))
        turnfwd();
    else if (PS5.getButtonPress(CROSS))
        turnrev();
    else
        stop();
}

void Kicker::setup(uint8_t kicker_pin) {
    m_enabled = true;
    m_kickerpin = kicker_pin;
    m_windupMotor.attach(kicker_pin); //! were we having issues with this causing delay?
}

void Kicker::Test() {
    if (m_enabled) {
    m_windupMotor.write(50); //clockwise
    delay(3000);
    m_windupMotor.write(90); //stop
    delay(1000);
    m_windupMotor.write(130); //counter-clockwise
    delay(3000);
    m_windupMotor.write(90); //stop
    }
}

void Kicker::turnfwd() {
    if (m_enabled)
    m_windupMotor.write(70);
}

void Kicker::turnrev() {
    if (m_enabled)
    m_windupMotor.write(110);
}

void Kicker::stop() {
    if (m_enabled)
    m_windupMotor.write(90);
}