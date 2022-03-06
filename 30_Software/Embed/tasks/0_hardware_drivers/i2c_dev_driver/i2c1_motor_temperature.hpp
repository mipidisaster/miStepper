/**************************************************************************************************
 * @file        i2c1_motor_temperature.hpp
 * @author      Thomas
 * @brief       Header file for I2C motor temperature sensor
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
#ifndef I2C1_MOTOR_TEMPERATURE_HPP_
#define I2C1_MOTOR_TEMPERATURE_HPP_

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
#include FilInd_BME280_HD               // Header for BME280 Device
#include FilInd_BMEI2C_HD               // Header for the BME280 I2C interface

//=================================================================================================

namespace _i2c1_dev::_motor_temp {
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
BME280I2C   generateMotorRightTemperatureSensor(void);
BME280I2C   generateMotorLeftTemperatureSensor(void);

void    manageMotorRightTemperatureSensor(I2CPeriph *hi2c, BME280I2C *right_device,
        _ihal::DevComFlt<BME280::DevFlt, I2CPeriph::DevFlt> *CommState, uint8_t *first_pass);
void    manageMotorLeftTemperatureSensor(I2CPeriph *hi2c, BME280I2C *left_device,
        _ihal::DevComFlt<BME280::DevFlt, I2CPeriph::DevFlt> *CommState, uint8_t *first_pass);

}
#endif /* I2C1_MOTOR_TEMPERATURE_HPP_ */
