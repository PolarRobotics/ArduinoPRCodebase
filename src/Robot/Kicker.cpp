// kicker

/**
//General Framework for the Kicker Code

//Manual Wind Up Arm
//Up On D-Pad
void manualWindUp(Servo KickerArm, String requestedStatus) {
  if (requestedStatus == "Wind") {
    KickerArm.write(50);
  }
  else if (requestedStatus == "Stop") {
    KickerArm.write(90);
  }
}

//Auto Wind Up
//Triangle on Controller
void autoWindUp() {

}

//Slow Release Arm
//Down on DPad
void slowRelease(Servo KickerArm, String requestedStatus) {
  if (requestedStatus == "Release") {
    KickerArm.write(130);
  }
  else if (requestedStatus == "Stop") {
    KickerArm.write(90);
  }
}

//Fire
//X on Controller
void fireKicker() {

}


//Step 1: Verify that this robot is actually the kicker
if (robotType == kicker) {
    if (PS3.getAnalogButton(R2)) {
        KickerMotor.write(75);
    }
    if (PS3.getAnalogButton(L2)) {
        KickerMotor.write(107);
    }
    if (!PS3.getAnalogButton(R2) && !PS3.getAnalogButton(l2)) {
        KickerMotor.write(90);
    }
}
**/
