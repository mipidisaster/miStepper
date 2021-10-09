/**************************************************************************************************
 * @file        ii2c_hal.hpp
 * @author      Thomas
 * @brief       HAL interface layer for the I2C task (header)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Header includes all the parameters that are output of the I2C Task.
 * Function - interfaceInitialise() is to be called prior to signals are used correctly.
 *************************************************************************************************/
#ifndef II2C_HAL_HPP_
#define II2C_HAL_HPP_

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

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_I2CPe__HD               // Include the I2C class handler
#include FilInd_AD741x_HD               // Include the device AD741x handler

//=================================================================================================

namespace _ihal::_ii2c1 {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
extern Semaphore<  Param<float>  > internal_top_temp_raw;
extern Semaphore<  Param<float>  > internal_bottom_temp_raw;

extern Semaphore< I2CPeriph::DevFlt > comm_flt;
extern Semaphore< DevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> > top_temp_sensor_flt;
extern Semaphore< DevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> > bottom_temp_sensor_flt;

/**************************************************************************************************
 * Exported types
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void interfaceInitialise(void);         // Initialise all ii2c parameters

}
#endif /* II2C_HAL_HPP_ */
