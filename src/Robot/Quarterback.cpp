#include "Robot/Quarterback.h"

Quarterback::Quarterback() {
    this->setType(TYPE::quarterback);
    this->setDrive(new Drive(MOTORS::small));

    // Attach the motors to their respective pins
    flywheelMotors.attach(FLYWHEEL_PIN);
    conveyorMotors.attach(CONVEYOR_PIN);
    elevationMotors.attach(ELEVATION_PIN);

    // Set initial rest state, all mechanisms off
    flywheelsOn = false;
    flywheelMotors.write(FLYWHEEL_STOP_SPEED);

    conveyorOn = false;
    conveyorMotors.write(CONVEYOR_OFF);

    // lower linear actuators to lowest elevation
    aimingUp = false;
    aimingDown = false;
    currentElevation = 0;
    elevationMotors.write(SERVO_SPEED_DOWN);
    delay(8000);
    elevationMotors.write(SERVO_SPEED_STOP);
}

void Quarterback::initialize() {
    Serial.println(F("Creating QB"));
}

void Quarterback::action(PS5BT& PS5) {
    Serial.println(F("Actual QB Action Executed"));
    // Update the bools within the class to see if the user wants to go up or down
    if (PS5.getButtonClick(UP))
      aim(qbAim::aimUp);
    else if (PS5.getButtonClick(DOWN))
      aim(qbAim::aimDown);
    
    // Update the aim on quarterback to see if we need to stop or not
    updateAim();

    // Toogle the Conveyor and Flywheels
    if (PS5.getButtonClick(SQUARE))
      toggleConveyor();
    else if (PS5.getButtonClick(CIRCLE))
      {toggleFlywheels();}
    
    // Change the flywheel speed
    if(PS5.getButtonClick(TRIANGLE))
      changeFlywheelSpeed(speedStatus::increase);
    else if (PS5.getButtonClick(CROSS))
      changeFlywheelSpeed(speedStatus::decrease);
}

void Quarterback::toggleFlywheels() {
    // Toggle the flywheels and use the speed factor to know what speed
    if (!flywheelsOn){
      flywheelMotors.write(FLYWHEEL_SPEED_BASE + flywheelSpeedFactor);
    } else {
      flywheelMotors.write(FLYWHEEL_STOP_SPEED);
    }
    // Toggle the bool so we know if its on or not
    flywheelsOn = !flywheelsOn;
}

// Aiming related functions

void Quarterback::aim(qbAim dir) {
  // Check which direction the user wants and turn the other direction off
  switch(dir) {
    case aimUp: aimingUp = true; aimingDown = false; break;
    case aimDown: aimingDown = true; aimingUp = false; break;
  }
}

void Quarterback::updateAim() {
    // If it is currently moving then dont go into the loop until it is done moving
    if (millis() - lastElevationTime >= ELEVATION_PERIOD) {
      if (aimingUp && currentElevation < MAX_ELEVATION) {
        // Set the elevation of the top to 50% higher
        elevationMotors.write(SERVO_SPEED_UP);
        currentElevation += 50;
        aimingUp = false;
        lastElevationTime = millis();

      } else if (aimingDown && currentElevation > 0) {
        // Set the elevation of the bottom to 50% lower
        elevationMotors.write(SERVO_SPEED_DOWN);
        currentElevation -= 50;
        aimingDown = false;
        lastElevationTime = millis();

      } else {
        // Stop the linear actuators if it is not supposed to move up or down anymore
        elevationMotors.write(SERVO_SPEED_STOP);
      }
    }
}

void Quarterback::toggleConveyor() {
    // Toggle the conveyor between on or off
    if (!conveyorOn){
      conveyorMotors.write(CONVEYOR_ON);
    } else {
      conveyorMotors.write(CONVEYOR_OFF);
    }
    // Toggle the bool so we know which mode it is in
    conveyorOn = !conveyorOn;
}

void Quarterback::changeFlywheelSpeed(speedStatus speed) {
  // Change the speed factor based on whether the user wants to increase or decrease
  switch(speed) {
    case increase: flywheelSpeedFactor += 5; break;
    case decrease: flywheelSpeedFactor -= 5; break;
  }
  // Cap it so they only have two levels to speed up and two levels to slow down
  flywheelSpeedFactor = constrain(flywheelSpeedFactor, -15, 15);

  // Update the motors if they are spinning for the new speed
  if (flywheelsOn){
      flywheelMotors.write(FLYWHEEL_SPEED_BASE + flywheelSpeedFactor);
    } else {
      flywheelMotors.write(FLYWHEEL_STOP_SPEED);
    }

}

// void Quarterback::passBall() {
//     //turn flywheels on to low: approx 10 power for a light boost
//     rightFlywheelMotor.write(FLYWHEEL_STOP_SPEED - 10);
//     leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED + 10);
//     //once firing mechanism is finished add that in and make it a macro?
// }

/*
    Notes:

    IDEA:
    can have an action where if you hold down on a button
    and move the left hat you can adjust the elevation of the QB
    this can allow for better control of the height

*/
