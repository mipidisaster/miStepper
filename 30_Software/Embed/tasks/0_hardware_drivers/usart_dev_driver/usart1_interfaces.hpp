/**************************************************************************************************
 * @file        usart1_interfaces.hpp
 * @author      Thomas
 * @brief       Header file for USART interfaces (to the miStepperUSART class)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Script to contain the linkage of the external (to USART function) parameters to the
 * miStepperUSART class.
 *************************************************************************************************/
#ifndef USART1_INTERFACES_HPP_
#define USART1_INTERFACES_HPP_

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
#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilIndMStpUARTHD               // Include the miStepper USART protocol class

//=================================================================================================

namespace _usart1_dev::_interfaces {
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
void    spi1(miStepperUSART     *mistepper_handle);
void    i2c1(miStepperUSART     *mistepper_handle);
void    adc1(miStepperUSART     *mistepper_handle);
void    fan(miStepperUSART      *mistepper_handle);
void    stepper(miStepperUSART  *mistepper_handle);
void    usart1(miStepperUSART   *mistepper_handle);
/* Individual functions to be called to connect the _ihal parameters to the miStepperUSART class,
 * ready for transmission.
 */

}
#endif /* USART1_INTERFACES_HPP_ */
