#pragma once

#ifndef ROBOT_H_
#define ROBOT_H_

#include <Arduino.h>
#include <PolarRobotics.h>
#include <Drive/Drive.h>

/**
 * @brief Robot Base Class Header File
 * @authors Max Phillips
 */

class Robot {
    private:
        Drive* drive;
        TYPE type;
    public:
        Robot() {};
        Drive* getDrive() { return drive; };
        void setDrive(Drive* d) { drive = d; };
        TYPE getType() { return type; };
        void setType(TYPE t) { type = t; };
        void setType(uint8_t t) { type = static_cast<TYPE>(t); };

        // Virtual function that effectively acts like a constructor
        // "virtual" keyword required to enable runtime polymorphism (i.e. actually use overrides)
        virtual void initialize() {};

        // Virtual function to perform any loop actions for special robots
        virtual void action() {}; 
        
};

#endif /* ROBOT_H_ */