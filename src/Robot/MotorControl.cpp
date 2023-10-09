#include <Arduino.h>

#include "MotorControl.h"

void ext_read_encoder0() {
  GlobalClassPointer[0]->readEncoder();
}

void ext_read_encoder1() {
  GlobalClassPointer[1]->readEncoder();
}

/**
 * @brief 
 * A class similar to the servo class, implements a method of choosing timers and channels 
 * based on the number of motors attached, uses writeMicros, etc. Utilizes the LedC built-in functions
 * 
 * since the sabretooth is expecting a duty cycle difference (time the signal is high for) 
 * I wrote a function to convert the time on value to a duty cycle, this would need to be moved into the Control class,
 * 
 * Goals/ Design specs: 
 *  - be able to declare multiple motor objects, with pin definitions.
 *  - avoid two motors on one pin and timers overlapping
 *  - write a value to the motor, maybe -100 100, we can play around with this
 *  - needs to be usable in drive and special bot classes
 * 
 * Example uses:
 * Definition: the motor object type can simply be defined by naming the motor object using the Motor type:
 *      Motor ExampleMotorObj;
 *   
 * and the pin can be attached by calling attach():
 *      ExampleMotorObj.attach();
 * 
 * to write to the motor simply pass a float between -1 and 1 into write()
 *      ExampleMotorObj.write(power);
 * 
 * where -1, 0 and 1 correspond to the throttles in the reverse, stop and forward directions
 * 
 * !TODO:
 * allow for us values outside the 1000us to 2000us, allow user to input the min and max values and pwr2Duty will account for that range
 * 
 * @author Rhys Davies 
 */
MotorControl::MotorControl(MotorType type, bool has_encoder, float gearRatio) {
  this->has_encoder = has_encoder;
  this->motor_type = type;
  this->gear_ratio = gearRatio;
  
  if(ServoCount < MAX_NUM_MOTORS)
    this->motorIndex = ServoCount++;  // assign a servo index to this instance
  else
    this->motorIndex = 255;
  // this->motorIndex = ServoCount < MAX_NUM_MOTORS ? ServoCount++ : 255;

  if (has_encoder) {
    this->encoderIndex = EncoderCount;
    GlobalClassPointer[EncoderCount++] = this;
    init_encoder();
  }

  // Calculate the max rpm by multiplying the nominal motor RPM by the gear ratio
  this->max_rpm = MOTOR_MAX_RPM_ARR[static_cast<uint8_t>(this->motor_type)] * this->gear_ratio;
}

void MotorControl::init_encoder() {
  // for use in void readEncoder()
  this->encoderACount = 0;
  this->b_channel_state = 0;
  this->rollover = 2048;

  // for use in int calcSpeed()
  this->prev_current_count = 0;
  this->rollover_threshold = 500; //this is bases on the fastes speed we expect, if the differace is going to be grater a rollover has likely accured
  this->current_time = 0;
  this->prev_current_time = 0; 
  this->omega = 0;
}

/**
 * @brief setup the given pin to the next free channel, returns channel number or 255 if failure
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * @param mot_pin the pin the motor is connected to
 * 
 * @return uint8_t the channel number the pin is attached to, 255 if failure
 */
uint8_t MotorControl::setup(int mot_pin, int enc_a_chan_pin, int enc_b_chan_pin) {
  this->enc_a_pin = enc_a_chan_pin, this->enc_b_pin = enc_b_chan_pin;
  
  if (this->enc_a_pin != -1 && this->enc_b_pin != -1 && has_encoder) {
    pinMode(this->enc_a_pin, INPUT_PULLUP);
    pinMode(this->enc_b_pin, INPUT);

    switch(this->encoderIndex) {
      case 0: {
        attachInterrupt(this->enc_a_pin, ext_read_encoder0, RISING);
        break;
      }
      case 1: {
        attachInterrupt(this->enc_a_pin, ext_read_encoder1, RISING);
        break;
      }
      default:
        return 254;
    }
  }

  // call the logic to attach the motor pin and setup, return 255 on an error
  return attach(mot_pin, MIN_PWM_US, MAX_PWM_US);
}

/**
 * @brief attach_us the given pin to the next free channel, returns channel number or 255 if failure
 * @author Rhys Davies
 * Updated 9-24-2023
 * 
 * @param pin the pin the motor is connected to
 * @param min min on time in us
 * @param max max on time in us
 * @return uint8_t the channel number the pin is attached to, 255 if failure
 */
uint8_t MotorControl::attach(int pin, int min, int max) {
  if(this->motorIndex < MAX_NUM_MOTORS - 1) {
    // pinMode(pin, OUTPUT);                             // set servo pin to output
    // digitalWrite(pin, LOW);                           // set the servo pin to low to avoid spinouts
    servos[this->motorIndex].pin = pin;                  // assign this servo a pin
    // servos[this->motorIndex].isactive = true;         // set the servo to active
    servos[this->motorIndex].channel = this->motorIndex; // set the servo ledc channel
    this->min_pwm = min; 
    this->max_pwm = max;
    ledcSetup(this->motorIndex, PWM_FREQ, PWM_RES);      // activate the timer channel to be used
    ledcAttachPin(pin, this->motorIndex);                // attach the pin to the timer channel
    ledcWrite(this->motorIndex, 0);                      // write a duty cycle of zero to activate the timer
  }
  return this->motorIndex;
}

/**
 * @brief write writes the pwm duty cycle, converted from the power input to the pin
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * input range:  [-1, 1]
 * 
 * @param pwr input power
*/
void MotorControl::write(float pwr) {
  ledcWrite(this->motorIndex, power2Duty(pwr));
}

void MotorControl::displayPinInfo() {
  Serial.print(F("Motor: "));
  Serial.print(this->motorIndex);
  Serial.print(F(" on Pin #"));
  Serial.print(servos[this->motorIndex].pin);
  Serial.print(F(" Channel: "));
  Serial.print(servos[this->motorIndex].channel);
  Serial.print(F(" Enc Chan A Pin: "));
  Serial.print(this->enc_a_pin);
  Serial.print(F(" Enc Chan B Pin: "));
  Serial.print(this->enc_b_pin);
  Serial.print(F("\r\n"));

    // Serial.print(F("\r\nDuty Cycle: "));
    // Serial.print(power2Duty(pwr));
}

/**
 * @brief power2Duty convert the [-1, 1] motor value to a timeon value is microseconds, 
 * then converts to a duty cycle and then scales that to a 16-bit integer, the resolution of the channel
 * link to math model: https://www.desmos.com/calculator/kmrh5pjrdf
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * input range:  [-1, 1]
 * output range: [0, 65,536] (Really 26215 to 52429)
 * 
 * @param power the input power
*/
uint16_t MotorControl::power2Duty(float power) {
  // this can be written in compiler code, but we are trying to save on flash memory
  // linearly convert a [-1, 1] motor power to a microseconds value between 1000us and 2000us
  this->tempTimeon = (power + 1) * 500 + 1000;
  return (tempTimeon / (PWM_PERIOD * 1000)) * (PWM_MAXDUTY / 1000);
}

/**
 * @brief writelow hopefully helps to prevent the issue with motors spinning at 
 * max speed on boot, sending bots into the stratosphere
 * 
 */
void MotorControl::writelow() {
    digitalWrite(servos[this->motorIndex].pin, LOW);
}


/**
 * @brief 
 * @author Grant Brautigam
 * Updated 9-11-2023
 * 
 * called on an interrupt
 * 
 * when the encoder interrupt is called, read the b pin to see what state it is in
 * this eliminates the need for two seperate interrupts
 * 
*/
void MotorControl::readEncoder() {

  b_channel_state = digitalRead(this->enc_b_pin);

  if (b_channel_state == 1) {
    if (encoderACount >= rollover) {
      encoderACount = 0;
    } else {
      encoderACount = encoderACount + 1;
    }
      
  } else {
    if (encoderACount == 0) {
      encoderACount = rollover;
    } else {
      encoderACount = encoderACount - 1;
    }  
  }
}

/**
 * @brief 
 * @author Grant Brautigam
 * Updated 9-11-2023
 * 
 * 
 * 
*/
int MotorControl::calcSpeed(int current_count) {  
  current_time = millis();
  
  //first check if the curret count has rolled over
  if (abs(current_count - prev_current_count) >= rollover_threshold) {
    if ((current_count-rollover_threshold)>0) {
      omega = float ((current_count-rollover)-prev_current_count)/(current_time-prev_current_time);
    } else {
      omega = float ((current_count+rollover)-prev_current_count)/(current_time-prev_current_time);
    }
  } else {
    omega = float (current_count-prev_current_count)/(current_time-prev_current_time);
  }

  prev_current_count = current_count;
  prev_current_time = current_time;

  return omega*156.25f; // 156.25 for 384, 312.5 for 192, 1250 for 48
}

int MotorControl::Percent2RPM(float pct) {
  return constrain(pct, -1, 1) * this->max_rpm;
}

float MotorControl::RPM2Percent(int rpm) {
  return constrain(rpm, -this->max_rpm, this->max_rpm) / this->max_rpm;
}

// Old code:

// /**
//  * alternate for servos writeMicroseconds, a function to set the motors based on a power input (-1 to 1),
//  * manually sets the motor high for the pwm time.
//  * @author Rhys Davies
//  * Created: 10-5-2022
//  * Updated: 10-29-2022
//  *
//  * Set the motors by outputting a pulse with a period corresponding to the motor power,
//  * determined by Convert2PWMVal. These calculations can not be put into a function,
//  * because it confuses the compiler as these are time critical tasks.
//  * 
//  * FUTURE: write this so both pins are HIGH at the same time, and the one that goes low first is called,
//  * because the beginning of the pulse is at the same time for both pins, but this isnt entirely necessary
//  * considering both motors operate independently on the sabertooth
//  *
//  * @param pwr the motor power to be set
//  * @param pin the motor to be set (0 for left, 1 for right)
// */
// void Drive::setMotorPWM(float pwr, byte pin) {
//     digitalWrite(motorPins[pin], HIGH);
//     delayMicroseconds(Convert2PWM(pwr) - 40);
//     digitalWrite(motorPins[pin], LOW);
//     delayMicroseconds(2000 - Convert2PWM(pwr) - 40); //-170
//     // digitalWrite(motorPins[0], HIGH);
//     // delayMicroseconds(Convert2PWMVal(requestedMotorPower[0]) - 40);
//     // digitalWrite(motorPins[0], LOW);
//     // // delayMicroseconds(2000 - Convert2PWMVal(requestedMotorPower[0]) - 40); //-170
//     // digitalWrite(motorPins[1], HIGH);
//     // delayMicroseconds(Convert2PWMVal(requestedMotorPower[1]) - 40);
//     // digitalWrite(motorPins[1], LOW);
//     // delayMicroseconds(2000 - Convert2PWMVal(requestedMotorPower[1]) - 40); //-170
// }


