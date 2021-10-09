/**************************************************************************************************
 * @file        itimer_hal.hpp
 * @author      Thomas
 * @brief       HAL interface layer for the Timer interrupt handler (header)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Header includes all the functions and parameters that are used outside of the Timer interrupt
 * handler.
 *************************************************************************************************/
#ifndef ITIMER_HAL_HPP_
#define ITIMER_HAL_HPP_

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
namespace _ihal::_itimer {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
extern volatile uint16_t   time_counter;            // Variable for counting time management
extern volatile uint32_t   idle_counter;            // Variable for idle counter

/**************************************************************************************************
 * Exported types
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
uint16_t tic(void);                         // Capture the time at start of function

uint16_t toc(uint16_t prev_time);           // Capture the time at end   of function
uint16_t toc(uint16_t *stored_time);        // Returns difference between 'stored_time', and
                                            // current time. Updates 'stored_time'

void recordTimeSheet(_ihal::TimedTasks task, uint16_t task_period, uint16_t task_duration);
    // Function to record the task duration and period into array

uint32_t timeStates(_ihal::TimedTasks task);    // Retrieve the time states of input 'task'

}
#endif /* ITIMER_HAL_HPP_ */
