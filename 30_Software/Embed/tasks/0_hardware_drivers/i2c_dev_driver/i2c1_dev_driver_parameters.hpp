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
inline constexpr uint16_t   kform_size      = 16;   // Queue size of the I2C1 class forms
inline constexpr uint8_t    kbuff_size      = 64;   // Queue size of the I2C1 data transmit
                                                    // and receive buffer

inline constexpr uint8_t    kwrte_loc                   = 0;    // Array position for Write
inline constexpr uint8_t    kread_top_temp_sensor       = 1;    // Array position for Read
                                                                // (Top Temperature Sensor)
inline constexpr uint8_t    kread_bottom_temp_sensor    = 2;    // Array position for Read
                                                                // (Bottom Temperature Sensor)


inline constexpr uint8_t    ktask_rate      = 5;    // Task rate of the I2C1 device

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
