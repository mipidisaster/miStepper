/**************************************************************************************************
 * @file        motor_driver_parameters.hpp
 * @author      Thomas
 * @brief       Header file for Motor task
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Parameters/constants specific to the MOTOR generic task handler. There are expected not to
 * change during run-time.
 *************************************************************************************************/
#ifndef MOTOR_DRIVER_PARAMETERS_HPP_
#define MOTOR_DRIVER_PARAMETERS_HPP_

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

namespace _motor::_param {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
inline constexpr uint8_t    ktask_rate      = 100;    // Task rate of the FAN task
/** READ ME BEFORE CHANGING 'ktask_rate'
  *
  * Currently the counter used to observe the task duration and periods is based upon TIM15 (also
  * used for Fan PWM), which currently set to:
  *     fcpu input          =   80MHz
  *     TIM15 Prescaler     =   3       = 80MHz  / (3    + 1)  = 20MHz          50ns
  *     Count Period        =   999     - 20MHz  / (999  + 1)  = 20KHz          50us
  *                                                                             0.05ms
  *     Therefore there are 20 counts to 1ms
  ************************************************************************************************/
/** READ ME BEFORE CHANGING 'ktask_rate'
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
#endif /* MOTOR_DRIVER_PARAMETERS_HPP_ */
