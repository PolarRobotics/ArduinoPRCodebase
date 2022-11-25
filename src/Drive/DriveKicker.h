#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "PolarRobotics.h"
#include "Drive/Drive.h"

class DriveKicker : public Drive {
    public:
        DriveKicker(int l, int r);
};