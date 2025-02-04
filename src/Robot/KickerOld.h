#ifndef KICKER_OLD_H
#define KICKER_OLD_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

/**
 * @brief Old Kicker header file
 * @authors Andrew Nelson 
 */
class KickerOld : public Robot {
  private:
    bool enabled; // safety feature
    uint8_t kickerPin;
    MotorControl windupMotor;
  public:
    KickerOld(uint8_t kickerPin);
    void action() override; //! robot subclass must override action
    void enable();
    void test(); 
    void turnForward(); 
    void turnReverse(); 
    void stop(); 
};

#endif // KICKER_OLD_H