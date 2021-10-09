/**************************************************************************************************
 * @file        stepper_driver.h
 * @author      Thomas
 * @brief       Header file for Stepper Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This is the main header file for the STEPPER driver, and will manage the setting up of the
 * class, and extra GPIOs connected to the IC controlling the Stepper.
 *
 * The files will be structured as such:
 *    "stepper_driver"              -> This will include the main task which will be looped
 *    "stepper_driver_parameters"   -> This will include constants, etc. which are used within the
 *                                     STEPPER task
 *************************************************************************************************/
#ifndef STEPPER_DRIVER_H_
#define STEPPER_DRIVER_H_

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
void vSTPMotorHAL(void const * argument);   // "main" task for Stepper Motor handle

void DMA1_Channel7_IRQHandler(void);        // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* STEPPER_DRIVER_H_ */
