/**************************************************************************************************
 * @file        i2c_board_temperature.hpp
 * @author      Thomas
 * @brief       Header file for I2C board temperature sensor
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Script to contain the generation and handling of the internal (board) temperature sensor
 * connected to the I2C interface
 *************************************************************************************************/
#ifndef I2C1_BOARD_TEMPERATURE_HPP_
#define I2C1_BOARD_TEMPERATURE_HPP_

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

namespace _i2c1_dev::_int_temp {
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
AD741x  generateInternalTopTemperatureSensor(void);
AD741x  generateInternalBottomTemperatureSensor(void);

void internalTemperatureSensorManagement(I2CPeriph *hi2c, AD741x *device,
                             uint8_t *wrBuff, uint8_t *rdBuff,
                             _ihal::DevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> *CommState);

}
#endif /* I2C1_BOARD_TEMPERATURE_HPP_ */
