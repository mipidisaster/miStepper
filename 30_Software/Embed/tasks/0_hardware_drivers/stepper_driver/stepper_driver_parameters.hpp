/**************************************************************************************************
 * @file        stepper_driver_parameters.hpp
 * @author      Thomas
 * @brief       Header file for STEPPER parameter(s)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Parameters/constants specific for the STEPPER device. These are expected not to change during
 * run-time.
 *************************************************************************************************/
#ifndef STEPPER_DRIVER_PARAMETERS_HPP_
#define STEPPER_DRIVER_PARAMETERS_HPP_

/**************************************************************************************************
 * Include all files that are needed to understand this header
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
// None

// Project Libraries
// -----------------
// None

//=================================================================================================

namespace _stepper::_param {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
inline constexpr uint8_t    ktask_rate      = 100;    // Task rate of the STEPPER task
/** READ ME BEFORE CHANGING 'FAN___HAL_Time'
  *
  * Currently the TIMER linked to the STEPPER (TIM1) is configured to give a resolution of 1us
  * (Prescaler = 79 -> 1MHz). With the size of hardware register limited to 16bits (65535), this
  * gives the slowest STEP pulse train of ~65ms (15Hz).
  * So the FAN HAL task cannot go any faster than this, if slowest speeds are expected.
  * Therefore the task, is limited to allow a minimum speed of HALF the iteration speed (software
  * limited at compilation time).
  *
  * With the current iteration rate of 100ms, this gives the slowest STEP pulse train of 50ms
  * (20Hz).
  ************************************************************************************************/

/**************************************************************************************************
 * Exported types
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

}
#endif /* STEPPER_DRIVER_PARAMETERS_HPP_ */
