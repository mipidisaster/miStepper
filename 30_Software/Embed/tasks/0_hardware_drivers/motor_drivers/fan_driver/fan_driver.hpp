/**************************************************************************************************
 * @file        fan_driver.hpp
 * @author      Thomas
 * @brief       Header file for Fan Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This is the main header file for the FAN driver, and will manage the setting up of the Timer,
 * and the PWM for this MOSFET.
 *
 * The files will be structured as such:
 *    "fan_driver"                  -> This will include the main task which will be looped
 *    "fan_driver_parameters"       -> This will include constants, etc. which are used within the
 *                                     FAN task
 *************************************************************************************************/
#ifndef FAN_DRIVER_H_
#define FAN_DRIVER_H_

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

//=================================================================================================

namespace _motor::_fan {
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
void setup(void);                       // Configure the FAN Timer(s)/PWM(s)
void firstpass(void);                   // Things to do only ONCE at initial pass through

void updatespeed(float new_speed);      // Update the FAN speed to new demand

}
#endif /* FAN_DRIVER_H_ */
