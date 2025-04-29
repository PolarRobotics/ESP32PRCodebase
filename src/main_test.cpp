/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  1000 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  3095 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  4000 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  8000 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define PWM_PERIOD 0.0025   // 2500 us
#define SERVO_FREQ 1/PWM_PERIOD
#define PWM_RES 12 
#define PWM_MAXDUTY (1 << PWM_RES) - 1

// our servo # counter
uint8_t servonum = 1;
void setup(){
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

uint16_t power2Duty(float power) {
  // this can be written in compiler code, but we are trying to save on flash memory
  float tempTimeon = (power + 1) * 500 + 1000;
  return tempTimeon / (PWM_PERIOD * 1000000) * (PWM_MAXDUTY);
}

void loop() {
  // Drive each servo one at a time using setPWM()
  // Serial.println(servonum);
  // for (uint16_t pulselen = SERVOMIN; pulselen < 2048; pulselen++) {
  //   pwm.setPWM(servonum, 0, pulselen);
  //   Serial.print("Pulse Length: ");
  //   Serial.println(pulselen);
  // }
  // delay(500);
  // for (uint16_t pulselen = 2048; pulselen > SERVOMIN; pulselen--) {
  //   pwm.setPWM(servonum, 0, pulselen);
  //   Serial.print("Pulse Length: ");
  //   Serial.println(pulselen);
  // }


  // for(float i = -1; i < 1; i+=0.05){
  //   Serial.print("i: ");
  //   Serial.print(i);
  //   Serial.print("  p2d: ");
  //   Serial.println(power2Duty(i));
  //   pwm.setPWM(1,0,power2Duty(i));
  //   delay(100);
  // }
  // for(float i = 1; i > -1; i-=0.05){
  //   pwm.setPWM(1,0,power2Duty(i));
  //   delay(100);
  // }

  power2Duty(0.0f);
  // delay(500);

}
