#ifndef DUMMY_H_
#define DUMMY_H_

#include "Robot/Robot.h"

/**
 * @brief Dummy Class Header to avoid vtable undefined errors
 * @authors Max Phillips
 */

// One of these empty dummy classes will be needed for each
class Quarterback: public Robot {
    public:
        Quarterback() {};
        void initialize() {}
        void action() { Serial.println(F("Dummy QB Action Executed")); }
};


#endif