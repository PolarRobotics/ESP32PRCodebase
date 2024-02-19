#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/**
 * @brief drive_param
 * a list of parameters pulled from Robot Config for bot-specific parameters related to drive
 * 
 * used in Drive and RobotConfig
*/
typedef struct drive_param {
    MotorType motor_type;
    float gear_ratio;
    float wheel_base;
    float r_min;
    float r_max;
} drive_param_t;

#endif // DRIVE_PARAMETERS_H