#include <Arduino.h>
#include <SPI.h>
#include <PS5BT.h>

#include <Robot/Kicker.h>

int pin = 9;
Kicker foot;
USB Usb;
BTD Btd(&Usb);
PS5BT PS5(&Btd);
//PS3BT PS3(&Btd); // for testing

void setup() {
  // put your setup code here, to run once:
  foot.setup(pin);
  if (Usb.Init() == -1) {
    while (Usb.Init() == -1); // Wait until reconnect
  }
  foot.Test();
}

void loop() {
  Usb.Task();
  // put your main code here, to run repeatedly:
  /*if (PS5.getButtonClick(TRIANGLE)) {
    foot->Test();
  }
  else*/ if (PS5.getButtonClick(SQUARE)) {
    foot.Windup();
  }
  else if (PS5.getButtonClick(CIRCLE)) {
    foot.Release();
  }
  else {
    foot.Stop();
  }
  delay(50);
  
}