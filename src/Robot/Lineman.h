#include <Robot/Robot.h>

/**
 * @brief Lineman Subclass Header
 * @authors Max Phillips
 */

class Lineman : public Robot {
    private: 
        int test;
    public:
        Lineman() {
            this->setDrive(new Drive(3,5));
        }
        void action();
};