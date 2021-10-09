/**************************************************************************************************
 * @file        ifan_hal.hpp
 * @author      Thomas
 * @brief       HAL interface layer for the FAN task (header)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Header includes all the parameters that are output of the FAN Task.
 * Function - interfaceInitialise() is to be called prior to signals are used correctly.
 *************************************************************************************************/
#ifndef IFAN_HAL_HPP_
#define IFAN_HAL_HPP_

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
#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

//=================================================================================================

namespace _ihal::_ifan {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
extern Semaphore< float > fan_demand;

/**************************************************************************************************
 * Exported types
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void interfaceInitialise(void);         // Initialise all ispi parameters

}
#endif /* TASKS_1_HARDWARE_ARBITRATION_LAYER_IFAN_HAL_HPP_ */