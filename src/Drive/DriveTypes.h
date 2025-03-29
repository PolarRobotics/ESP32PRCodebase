#pragma once

#ifndef _DRIVE_TYPES_H
#define _DRIVE_TYPES_H

typedef enum {
  none,         // no drive type, used for robots that dont need to move on the ground (i.e. QB turret)
  differential, // for two-powered wheel configurations (default config for most robots)
  mecanum,      // for wheels in a mecanum configuration
  swerve,       // for wheels in a coaxial swerve configuration, 
                // all wheels are powered and can rotate independently, 
                // meaning there are two motors for each wheel touching the ground
  omni          // for wheels in an omni configuration (i.e. 4 omni wheels)
} drive_type_t;

#define NUM_DIFF_DRIVE_STATES 12 // this number MUST match the number of states in the enum below

typedef enum {
  idle,
  positive,
  negative,
  tank_left,
  tank_right,
  positive_left,
  positive_right,
  negative_left,
  negative_right,
  hold_positive,
  hold_negative,
  hold_position_angle
} differential_drive_state_t;

const char* getDriveStateString(differential_drive_state_t state);

#endif // _DRIVE_TYPES_H