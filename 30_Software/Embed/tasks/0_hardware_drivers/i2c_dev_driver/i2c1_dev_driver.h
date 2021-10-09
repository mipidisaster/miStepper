/**************************************************************************************************
 * @file        i2c1_dev_driver.h
 * @author      Thomas
 * @brief       Header file for I2C Device Driver task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This is the main header file for the I2C Device(s) driver, and will manage the communication
 * with the external device sensing temperature (pressure and humdity with the BME device).
 * 
 * The files will be structed as such:
 *    "i2c_dev_driver"              -> This will include the main task which will be looped
 *    "i2c_dev_driver_parameters"   -> This will include constants, etc. which are used within the
 *                                     I2C system
 *    "i2c_board_temperature"       -> This will include the functions for interfacing with the
 *                                     temperature sensor(s) for board temperature
 *************************************************************************************************/
#ifndef I2C1_DEV_DRIVER_H_
#define I2C1_DEV_DRIVER_H_

/**************************************************************************************************
 * Include all files that are needed to understand this header
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// C System Header(s)
// ------------------
// None

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
// None

// Project Libraries
// -----------------
#include "main.h"                       // Include main header file, as this contains the defines
                                        // for GPIO signals
#include "stm32l4xx_hal.h"              // Include the HAL library

#ifdef __cplusplus
extern "C" {
#endif

//=================================================================================================

/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported types
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void vI2C1DeviceHAL(void const * argument); // "main" task for I2C handle

void I2C1_EV_IRQHandler(void);              // This task file also includes the prototype used to
void I2C1_ER_IRQHandler(void);              // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* I2C1_DEV_DRIVER_H_ */
