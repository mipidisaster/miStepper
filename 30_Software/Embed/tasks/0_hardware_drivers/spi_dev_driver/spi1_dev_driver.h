/**************************************************************************************************
 * @file        spi1_dev_driver.h
 * @author      Thomas
 * @brief       Header file for the SPI Device Driver task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This is the main header file for the SPI Device(s) driver, and will manage the communication
 * with the external device sensing the angular position of the stepper motor.
 *
 * The files will be structured as such:
 *    "spi_dev_driver"              -> This will include the main task which will be looped
 *    "spi_dev_driver_parameters"   -> This will include constants, etc. which are used within the
 *                                     SPI system
 *    "spi_angle_sensor"            -> This will include the functions for interfacing with the
 *                                     angle sensor device.
 *************************************************************************************************/
#ifndef SPI1_DEV_DRIVER_H_
#define SPI1_DEV_DRIVER_H_

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
void vSPI1DeviceHAL(void const * argument); // "main" task for SPI handle

void SPI1_IRQHandler(void);                 // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* SPI_DEV_DRIVER_H_ */
