/**************************************************************************************************
 * @file        itimer.cpp
 * @author      Thomas
 * @brief       HAL interface layer for the Timer interrupt handler (source)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"
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
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
volatile uint16_t   time_counter    = 0;    // Variable for counting time management
volatile uint32_t   idle_counter    = 0;    // Variable for idle counter

static   uint32_t   time_sheet[_ihal::TimedTasks::kMAX_LIMIT] = { 0 };
    /* Array to store the compressed Timed Task Duration and Task Period
     *      Formed [  16bits DURATION  ] [   16bits PERIOD   ]
     */
//=================================================================================================

/**
  * @brief:  Return the current 'time_counter' value
  * @param:  Void
  * @retval: None (void output)
  */
uint16_t tic(void) {
    return ( time_counter );
}

/**
  * @brief:  Calculate difference in times, between the 'prev_time' recorded and the current time
  * @param:  unsigned 16bit integer of the previous recording of time
  * @retval: None (void output)
  */
uint16_t toc(uint16_t prev_time) {
    // The modulus isn't really required, as the value will already overflow and give modulus
    return ( (time_counter - prev_time) % 0xFFFF );
}

/**
  * @brief:  Calculate difference between the current time and 'stored_time', returning the
  *          difference, additionally will update the 'stored_time'
  * @param:  Pointer to current stored time
  * @retval: None (void output)
  */
uint16_t toc(uint16_t *stored_time) {
    uint16_t temp = toc(*stored_time);

    *stored_time = time_counter;

    return ( temp );
}

/**
  * @brief:  Calculate difference in times, between the 'prev_time' recorded and the current time
  * @param:  Enumerate for the position of the task within the 'time_sheet' array
  * @param:  unsigned 16bit integer of the calculated task period
  * @param:  unsigned 16bit integer of the calculated task duration
  * @retval: None (void output)
  */
void recordTimeSheet(_ihal::TimedTasks task, uint16_t task_period, uint16_t task_duration) {
    time_sheet[task]    = ( task_duration ) << 16;
    time_sheet[task]   |=   task_period;
}

/**
  * @brief:  Return the contents of the time sheet for input 'task'
  * @param:  Enumerate for the position of the task within the 'time_sheet' array
  * @retval: 32bit (unsigned) contents of task timing(s)
  */
uint32_t timeStates(_ihal::TimedTasks task) {
    return (  time_sheet[task]  );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None
}
