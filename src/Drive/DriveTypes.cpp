#include "Drive/DriveTypes.h"
#include "Utilities/Pair.h"

// DriveState - the a list of possible drive configuration of the robot drive system, 
// with a human-readable C-strings
constexpr Pair<drive_type_t, const char*> DriveType[NUM_DRIVE_TYPES] = {
    { none,         "none"         },
    { differential, "differential" },
    { mecanum,      "mecanum"      },
    { swerve,       "swerve"       },
    { omni,         "omni"         }
};

// Function to map BotTypes to human-readable C-strings
const char* getDriveStateString(differential_drive_state_t state) {
    return DriveType[static_cast<int>(state)].value;
}

// DiffDriveState - the current state of the DIFFERNETIAL drive system
// constexpr to be evaluated at compile time
constexpr Pair<differential_drive_state_t, const char*> DiffDriveState[NUM_DRIVE_STATES] = {
    { idle,                "idle"                },
    { positive,            "positive"            },
    { negative,            "negative"            },
    { tank_left,           "tank_left"           },
    { tank_right,          "tank_right"          },
    { positive_left,       "positive_left"       },
    { positive_right,      "positive_right"      },
    { negative_left,       "negative_left"       },
    { negative_right,      "negative_right"      },
    { hold_positive,       "hold_positive"       },
    { hold_negative,       "hold_negative"       },
    { hold_position_angle, "hold_position_angle" }
};

// Function to map BotTypes to human-readable C-strings
const char* getDriveStateString(differential_drive_state_t state) {
    return DiffDriveState[static_cast<int>(state)].value;
}