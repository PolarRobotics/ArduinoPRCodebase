#include <Kicker.h>

Kicker::Kicker() {
}

void Kicker::setup(uint8_t KickerPin) {
  m_kickerpin = KickerPin;
  kickerMotor.attach(KickerPin);
}

void Kicker::Test() {
  kickerMotor.write(50); //clockwise
  delay(3000);
  kickerMotor.write(90); //stop
  delay(1000);
  kickerMotor.write(130); //counter-clockwise
  delay(3000);
  kickerMotor.write(90); //stop
}

void Kicker::Windup() {
  kickerMotor.write(50);
  delay(3000);
}

void Kicker::Release() {
  kickerMotor.write(130);
  delay(3000);
}

void Kicker::Stop() {
  kickerMotor.write(90);
}