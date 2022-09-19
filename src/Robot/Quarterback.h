#include <Robot/Robot.h>
#include <Drive/DriveQuick.h>

/**
 * @brief Quarterback Subclass Header
 * @authors Max Phillips
 */
class Quarterback: public Robot {
    private: 
        int test;
    public:
        Quarterback() {}
        void initialize();
        void action();
};