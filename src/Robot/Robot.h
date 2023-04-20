#pragma once

#ifndef ROBOT_H_
#define ROBOT_H_

#include <Arduino.h>
#include <PolarRobotics.h>
#include <Drive/Drive.h>

/**
 * @brief Robot Base Class Header File
 * @authors Max Phillips
 * 
 * This robot base class is an abstraction to allow polymorphic behavior for all robots.
 * Essentially, declare an object of type Robot in `main.cpp`, then implement/override 
 * the `initialize()` and `action()` virtual functions in derived classes.
 * This will allow main to perform runtime polymorphism, effectively simplifying the main
 * code while still allowing individual robot types to have their own special actions.
 * 
 * Each class inheriting from Robot must implement the two virtual functions.
 * It should also set its own drive to a derived class of Drive if necessary.
 **/

class Robot {
  private:
    Drive* drive;
    TYPE type; // `TYPE` enum declared in `PolarRobotics.h`
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