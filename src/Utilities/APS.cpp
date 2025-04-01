#include "APS.h"

/**
 * This is the main initialize method that causes delays in the runtime.
 * Note: Blocking - must be called before using :)
 * 
 * @author Corbin Hibler
 * @date 2025-03-31
 */
void APS::initialize() {
  printf("Absolute Positioning System Init\n");
  I2CSetup();
  BNOInit();
}

/**
 * Common I2C setup for communication with BNO055's SCL and SDA pins
 * Will be used regardless of BNO055 configuration, that can be changed
 * in the method BNOInit().
 * 
 * @author Corbin Hibler
 * @date 2025-03-31
 */
void APS::I2CSetup() {
  static i2c_config_t Config;
  memset(&Config, 0, sizeof(i2c_config_t));
  Config.mode = I2C_MODE_MASTER;
  Config.sda_io_num = (gpio_num_t)I2C_SDA;
  Config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  Config.scl_io_num = (gpio_num_t)I2C_SCL;
  Config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  Config.master.clk_speed = 500000;
  i2c_param_config((i2c_port_t)I2C_PORT_NUMBER, &Config);
  i2c_driver_install((i2c_port_t)I2C_PORT_NUMBER, Config.mode, 0, 0, 0);
  i2c_set_timeout((i2c_port_t)I2C_PORT_NUMBER,
                  (I2C_APB_CLK_FREQ / Config.master.clk_speed) * 1024);

  // vTaskDelay puts the current method into blocked 
  // mode, runs other things in the meantime
  vTaskDelay(750 / portTICK_PERIOD_MS); 
};

/**
 * Initialization of the BNO055 object
 * Important to keep this private, while APS is kept public
 * 
 * @author Corbin Hibler
 * @date 2025-03-31
 */
void APS::BNOInit() {
  this->bno055 = new BNO055((i2c_port_t)I2C_PORT_NUMBER, 0x28);
  this->bno055->begin(); // BNO055 is in CONFIG_MODE until it is changed
  this->bno055->enableExternalCrystal();
  // bno.setSensorOffsets(storedOffsets);
  // bno055->setAxisRemap(BNO055_REMAP_CONFIG_P5, BNO055_REMAP_SIGN_P0); // see
  // datasheet, section 3.4
  /* you can specify a PoWeRMode using:
      - setPwrModeNormal(); (Default on startup)
      - setPwrModeLowPower();
      - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
    */
  this->bno055->setOprModeCompass();
  /* Gyroscope is not accessible during compass mode. Compass mode
   is all we really need, and allows for true north and absolute orientation.
   If a future robot requires the gyroscope, please create an overloaded method
   of this method. 
  */
}

/**
 * Returns the compass heading from the Euler angle vector.
 * The Euler angle returned is on the alpha rotation plane, 
 * or the X, Y plane in Cartesian.
 * 
 * For some reason in the library this is called "x" which is
 * not very specific to Euler angles or the Cartesian plane, 
 * but it's not too important to worry about.
 * 
 * @author Corbin Hibler
 * @date 2025-03-31
 */
uint16_t APS::getHeading() {
  bno055_vector_t eulerVector = this->bno055->getVectorEuler();
  return eulerVector.x;
}