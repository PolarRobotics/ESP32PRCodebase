#ifndef ROBOT_H
#define ROBOT_H

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
    // ! Drive should be completely decoupled from Robot
    eBOT_TYPE type; // `eBOT_TYPE` enum declared in `BotTypes.h`
  protected:
    // ! setType methods should only be called in the constructor of the derived class!
    void setType(eBOT_TYPE t) { type = t; };
    void setType(uint8_t t) { type = static_cast<eBOT_TYPE>(t); };
  public:
    // Robot() {};
    eBOT_TYPE getType() { return type; };


    // TODO: we should be able to avoid using this eventually, and instead use the constructor properly
    // Virtual function that effectively acts like a constructor
    // "virtual" keyword required to enable runtime polymorphism (i.e. actually use overrides)
    // virtual void initialize() = 0;

    // Virtual function to perform any loop actions for special robots
    virtual void action() = 0;
};

#endif // ROBOT_H