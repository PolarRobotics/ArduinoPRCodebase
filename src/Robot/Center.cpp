/*
       ____   _____   _   _   _____   _____   ____
      / ___| | ____| | \ | | |_   _| | ____| |  _ \
     | |     |  _|   |  \| |   | |   |  _|   | |_) |
     | |___  | |___  | |\  |   | |   | |___  |  _ <
      \____| |_____| |_| \_|   |_|   |_____| |_| \_\
  */


/** Center Code

    --- Functions List ---
    int centerUp(SabertoothSimplified Sabre, String requestedStatus)        - Raises the arm on the center up    | Returns the current Position of the Arm for the main code to handle if it wants | Up Arrow on D-Pad Activates Function
    int centerDown(SabertoothSimplified Sabre, String requestedStatus)      - Lowers the arm on the center       | Returns the current Position of the Arm for the main code to handle if it wants | Down Arrow on D-Pad Activates Function
    
    int centerOpen(SabertoothSimplified Sabre, String requestedStatus)      - Move the servo to the open Position   | Pressing X Opens the Servo
    int centerClose(SabertoothSimplified Sabre, String requestedStatus)     - Move the servo to the closed Position | Pressing Triangle Closes the Servo

*/

#include <Arduino.h>

enum class Status {
  Higher, Lower, Stop
};

class Center {
  private:
    uint8_t m_clawpin, m_elevationpin;
    int m_elevation;
    // SabertoothSimplified m_Sabertooth;
    Status m_currstatus;

  public:
    Center(uint8_t ClawPin, uint8_t ElevationPin); 
    void open(Status requestedStatus);
    void close(Status requestedStatus);
    int up(Status requestedStatus);
    int down(Status requestedStatus);
}

Center::Center(SabertoothSimplified ST, uint8_t ClawPin, uint8_t ElevationPin) {
  m_clawpin = ClawPin;
  m_elevationpin = ElevationPin;
  m_Sabertooth = ST;
  m_currstatus = Status::Stop;
}

/** void open */
void Center::open() { //, Status requestedStatus
  
}

/** void close */
void Center::close() { //, Status requestedStatus
  
}

/** int up */
int Center::up(Status requestedStatus) {
  if (requestedStatus == Status::Higher) {
    m_Sabertooth.motor(1, 10);
  } else if (requestedStatus == Status::Stop) {
    m_Sabertooth.motor(1, 0);
  }
  return 1;
}

/** int down */
int Center::down(Status requestedStatus) {
  if (requestedStatus == Status::Higher) {
    m_Sabertooth.motor(1, -10);
  } else if (requestedStatus == Status::Stop) {
    m_Sabertooth.motor(1, 0);
  }
  return 2;
}


// original code
//Raise the Center Arm
/*
    if (CenterArmMoveStatus == "" && (PS3.getButtonPress(UP)) && CenterArmStatus != 1) {
      //The user wants to raise the Center arm up
      CenterArmMoveStatus = "High";
      CenterArmMoveTimer = millis();
      CenterArmStatus = centerUp(ST, CenterArmMoveStatus);
    } else if (CenterArmMoveStatus == "High" && (millis() - CenterArmMoveTimer) > 5000) {
      CenterArmMoveStatus = "Stop";
      centerUp(ST, CenterArmMoveStatus);
      CenterArmMoveStatus = "";
    }

    //Lower the Center Arm
    if (CenterArmMoveStatus == "" && (PS3.getButtonPress(DOWN)) && CenterArmStatus != 2) {
      //The user wants to raise the Center arm up
      CenterArmMoveStatus = "Lower";
      CenterArmMoveTimer = millis();
      CenterArmStatus = centerDown(ST, CenterArmMoveStatus);
    } else if (CenterArmMoveStatus == "Lower" && (millis() - CenterArmMoveTimer) > 5000) {
      CenterArmMoveStatus = "Stop";
      centerUp(ST, CenterArmMoveStatus);
      CenterArmMoveStatus = "";
    }
*/