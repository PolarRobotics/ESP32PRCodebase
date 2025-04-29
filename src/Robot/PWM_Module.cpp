#include <Arduino.h>
#include <Robot/PWM_Module.h>

// Define the pwm_test_module variable
// Adafruit_PWMServoDriver pwm_test_module = Adafruit_PWMServoDriver(PWM_ADDRESS);
// Adafruit_PWMServoDriver PWM_Module::pwm_test_module = Adafruit_PWMServoDriver(PWM_ADDRESS);

Adafruit_PWMServoDriver* PWM_Module::pwm_test_module = nullptr;

Adafruit_PWMServoDriver* PWM_Module::getPWMInstance() {
  if (pwm_test_module == nullptr) {
    pwm_test_module = new Adafruit_PWMServoDriver(PWM_ADDRESS);
    pwm_test_module->begin();
    pwm_test_module->setPWMFreq(PWM_FREQ); // Set frequency
    pwm_test_module->setOscillatorFrequency(27000000);
  }
  return pwm_test_module;
}

PWM_Module::PWM_Module() {
  if(MotorCount == 0){
    getPWMInstance(); // initialize the PWM module
  }
  if(MotorCount < MAX_NUM_MOTORS){
    this->motorIndex = MotorCount; // assign a servo index to this instance
    MotorCount++;
  }
  else
    this->motorIndex = 255;
}

uint8_t PWM_Module::attach(int pin, int min = MIN_PWM_US, int max = MAX_PWM_US) {
  if(this->motorIndex < MAX_NUM_MOTORS - 1) {
    motors[this->motorIndex].pin = pin; // assign this servo a pin
    this->min = min; 
    this->max = max;
    pwm_test_module->setPWM(this->motorIndex, 0, power2Duty(0)); // No need to set channel
  }
  return this->motorIndex;
}

void PWM_Module::write(float pwr) {
  pwm_test_module->setPWM(this->motorIndex, 0, power2Duty(pwr));
}

uint16_t PWM_Module::power2Duty(float power) {
  // this can be written in compiler code, but we are trying to save on flash memory
  this->tempTimeon = (power + 1) * 500 + 1000;
  return (tempTimeon / (PWM_PERIOD* 1000000)) * PWM_MAXDUTY;
}

void PWM_Module::writelow() {
  write(0);
}

void PWM_Module::displayPinInfo() {
  Serial.print(F("Motor: "));
  Serial.print(this->motorIndex);
  Serial.print(F(" on Pin #"));
  Serial.print(motors[this->motorIndex].pin);

    // Serial.print(F("\r\nDuty Cycle: "));
    // Serial.print(power2Duty(pwr));
}