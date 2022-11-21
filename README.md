# PRCodebase
Primary Polar Robotics Codebase

## Filestructure and Inheritance
```
src/
├ Drive/
│ └ Drive.h             Header file for Drive.cpp
│   ├ Drive.cpp         Standard drive code class/object
│   ├ DriveKicker.cpp   Special Drive for Kicker
│   ├ DriveMecanum.cpp  Special Drive for New Center
│   └ DriveQuick.cpp    Special Drive for Runningback
│
├ Robot/
│ ├ Robot.h             Base Robot Parent Class
│ │ ├ Lineman.h         Derived Class for Linemen and Receiver
│ │ │ └ Lineman.cpp     Implementations for Lineman-specific functions and controls
│ │ ├ Runningback.h     Derived Class for Runningback-specific functions and controls
│ │ │ └ Runningback.cpp Implementations for Runningback
│ │ ├ Quarterback.h     Derived Class for Quarterback-specific functions and controls
│ │ │ └ Quarterback.cpp Implementations for flywheels, elevation, conveyor, etc.
│ │ ├ Kicker.h          Derived Class for Kicker-specific functions and controls
│ │ │ └ Kicker.cpp      Implementations for winding/release, etc.
│ │ ├ Center.h          Derived Class for Center-specific functions and controls
│ │ │ └ Center.cpp      Implementations for claw and arm lift, etc.
│ │ └ CenterNew.h       New Center-specific functions, inherits from Robot
│ └ Lights.h            Header file for robot Lights class
│   └ Lights.cpp        Controls robot LEDs
│ 
├ main.cpp              Contains code that initializes the Robot and Drivebase.
└ PolarRobotics.h       Contains globally relevant declarations and enums.
```