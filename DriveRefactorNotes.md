


Wishlist:
- ~~move BSN defines into an array based on motor type~~
- ~~Drive State Machine~~
- DriveType: method of drive, mecanum, differental, swerve... etc...
  - add to robot configuration -> drive params 
- Clean up turning code -> combine into one function (from 3)
- move controller inputs into Drive->action (like Robot)
- consolidate enum and definitions
  - affects: DriveParameters.h, MotorTypes into DriveTypes.h
- add Turn Sensitivity type and amount to Robot configurations and drive params
- maybe make acceleration rate a configurable parameter
- need to make tuning easier and more clear, maybe a guide on how to tune a robots driving...
- HoldAngle needs implemented further (for QB V3)
  - holds robot in place, useful for counteracting the turrets inertia. NOTE: will only work on robots with drive wheels in parallel with the ICC (turning axis center) of the robot
Test if motor curve coeffs are doing anything meaningful

DriveStraight:
- What needs tested:
  - PID class needs to be tested and confirmed working on the robots
  - Drive straight needs gains tuned further
  - try removing motor curve adjustment
  - Test on :) and both new runningbacks
- issues with I2C and PWM being used at the same time on the ESP, trying a breakout board
