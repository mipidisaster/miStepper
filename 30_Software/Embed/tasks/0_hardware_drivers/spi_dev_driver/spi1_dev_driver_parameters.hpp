/**************************************************************************************************
 * @file        spi1_dev_driver_parameters.h
 * @author      Thomas
 * @brief       Header file for SPI parameter(s)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Parameters/constants specific for the SPI1 device. These are expected not to change during
 * run-time.
 *************************************************************************************************/
#ifndef SPI1_DEV_DRIVER_PARAMETERS_HPP_
#define SPI1_DEV_DRIVER_PARAMETERS_HPP_

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

namespace _spi1_dev::_param {
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
inline constexpr uint8_t    ktask_rate      = 5;        // Task rate of the SPI1 device

// Internal class form size(s)
inline constexpr uint16_t   kspi_form_size      = 16;   // Queue size of the SPI1 class forms
inline constexpr uint8_t    kangle_form_depth   = 16;   // Queue size for the AS5047 device

// Communication register size(s)
inline constexpr uint8_t    kangle_buff_size    = 32;   // Register size(s) of the SPI1 data
                                                        // transmit and receive

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
#endif /* SPI1_DEV_DRIVER_PARAMETERS_HPP_ */
