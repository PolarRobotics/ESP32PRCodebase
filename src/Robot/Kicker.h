#ifndef KICKER_H
#define KICKER_H

#include <MotorInterface.h>
#include <Robot/Robot.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`
#include <Utilities/Debouncer.h>

// Encoder Information
#define KICKER_COUNTS_PER_ENCODER_REV 11  // Number of encoder counts per revolution of base shaft
#define KICKER_COUNTS_PER_ARM_REV 1188    // Number of encoder counts per revolution of output shaft (11 * 108 = 1188)
#define KICKER_COUNTS_PER_ARM_DEGREE 3.3  // Number of encoder counts per degree of the arm (1188 / 360 = 3.3)

#define KICKER_ENABLE_DB_DELAY 100L

// Motor Speeds
#define KICKER_HOMING_SPEED 0.5

/**
 * @brief Kicker V2 Class
 * 
 * Contains logic for the operation of the encoder, motorized kicking arm, and adjusting the angle
 * automatically upon startup. Other features include manual control of kicking arm in case the
 * automatic system fails.
 * 
 * @authors Andrew Nelson, Corbin Hibler
 */
class Kicker : public Robot {
private:
  bool enabled; // Safety feature to ensure robot does not act when it is not supposed to.
  Debouncer* dbEnable;
  Debouncer* dbHome;

  MotorControl triggerMotor;      // on the Kicker V2, this is the motor that releases the mechanism to fire the arm
                                  //* negative is towards limit switch, positive is away from limit switch
  uint16_t triggerMotorHomeAngle; // Angle of the motor when at the limit switch (zeroed/homed).
  uint8_t triggerMotorPin;        // Pin to control the motor of the kicker arm
  uint8_t limitSwitchPin;         // Pin to connect to the limit switch

  static uint8_t kickerEncoderPinA;         // Signal Pin for channel A of the encoder
  static uint8_t kickerEncoderPinB;         // Signal Pin for channel B of the encoder
  static uint8_t kickerEncoderStateB;       // Keeps track of the current state of channel B
  static int32_t currentKickerEncoderCount; // Encoder count of kicker arm motor encoder

public:
  Kicker(
    uint8_t triggerMotorPin, // Pin to control the motor of the kicker arm
    uint8_t limitSwitchPin, // Pin to connect to the limit switch
    uint8_t kickerEncoderPinA, // Signal Pin for channel A of the encoder
    uint8_t kickerEncoderPinB // Signal Pin for channel B of the encoder
  );
  void action() override; //! robot subclass must override action
  void enable();
  void turnForward();
  void turnReverse();
  void stop();
  void homeTriggerMotor();
  void adjustAngle(int angle);
  static void kickerEncoderISR();
  uint16_t getCurrentAngle();
  void printCurrentAngle();
};

#endif // KICKER_H
