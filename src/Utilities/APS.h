#ifndef APS_H
#define APS_H

#include <BNO055ESP32.h>
#include <PolarRobotics.h>
#include <Wire.h>

#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_PORT_NUMBER 0
#define I2C_ADDRESS 0x29

/**
 * APS = Absolute Positioning System
 * Class that utilizes a modified BNO055 library "BNO055ESP32" (in PR-Lib) to
 * access important absolute positioning information, such as: Euler angles,
 * Quaternions, compass heading, and acceleration vector. 
 * 
 * @author Corbin Hibler
 * @date 2025-03-31
 */
class APS {
private:
  BNO055* bno055 = nullptr;
  void I2CSetup();
  void BNOInit();
public:
  void initialize();
  uint16_t getHeading();
};

#endif // APS_H
