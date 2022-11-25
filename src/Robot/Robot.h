#pragma once

#ifndef ROBOT_H_
#define ROBOT_H_

#include <Arduino.h>
#include <PS5BT.h>
#include "PolarRobotics.h"
#include "Drive/Drive.h"

/**
 * @brief Robot Base Class Header File
 * @authors Max Phillips
 */

class Robot {
    private:
        Drive* drive;
        TYPE type;
    protected:
        // ! setType methods should only be called in the constructor of the derived class!
        void setType(TYPE t) { type = t; };
        void setType(uint8_t t) { type = static_cast<TYPE>(t); };
    public:
        Drive* getDrive() { return drive; };
        void setDrive(Drive* d) { drive = d; };
        TYPE getType() { return type; };


        // Virtual function that effectively acts like a constructor
        // TODO: we should be able to avoid using this eventually, and instead use the constructor properly
        // "virtual" keyword required to enable runtime polymorphism (i.e. actually use overrides)
        virtual void initialize() = 0;

        // Virtual function to perform any loop actions for special robots
        virtual void action(PS5BT& PS5) = 0;
        
};

#endif /* ROBOT_H_ */