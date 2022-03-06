/**************************************************************************************************
 * @file        i2c1_dev_driver_parameters.hpp
 * @author      Thomas
 * @brief       Header file for I2C parameter(s)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Parameters/constants specific for the I2C1 device. These are expected not to change during
 * run-time.
 *************************************************************************************************/
#ifndef I2C1_DEV_DRIVER_PARAMETERS_HPP_
#define I2C1_DEV_DRIVER_PARAMETERS_HPP_

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

namespace _i2c1_dev::_param {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// Task rate of SPI main()
inline constexpr uint8_t    ktask_rate      = 5;        // Task rate of the I2C1 device

// Internal class form size(s)
inline constexpr uint16_t   ki2c_form_size      = 16;   // Queue size of the I2C1 class forms
inline constexpr uint8_t    kboard_form_depth   = 16;   // Queue size of the AD741x device(s)
inline constexpr uint8_t    kmotor_form_depth   = 16;   // Queue size of the BME280 device(s)

// Communication register size(s)
inline constexpr uint8_t    kboard_buff_size    = 16;   // Register size(s) of the I2C1 data
inline constexpr uint8_t    kmotor_buff_size    = 16;   // transmit and receive

// Fault threshold(s)
inline constexpr uint8_t    kfault_count        = 6;    // Maximum number of no data, before faults

//////
inline constexpr uint8_t    kwrte_loc       = 0;        // Array position for Write
inline constexpr uint8_t    kread_loc       = 1;        // Array position for Read
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
#endif /* I2C1_DEV_DRIVER_PARAMETERS_HPP_ */
