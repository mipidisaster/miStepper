/**************************************************************************************************
 * @file        timer_dev_interrupt.cpp
 * @author      Thomas
 * @brief       << Manually Entered >>
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/timer_dev/timer_dev_interrupt.h"
// C System Header(s)
// ------------------
#include <stdint.h>
#include "cmsis_os.h"                   // Header for introducing FreeRTOS to device

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
// None

// Project Libraries
// -----------------
#include "stm32l4xx_hal.h"              // Include the HAL library
#include "tim.h"                        // Include the timer header file, as this contains the
                                        // htim15 handle
#include "FreeRTOSConfig.h"             // Read in the RTOS configuration, so as to determine if
                                        // the run time stats is needed in RTOS form

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

//=================================================================================================

/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// Variable counter to only be generated if the RTOS run time states is enabled
#ifdef configGENERATE_RUN_TIME_STATS
static uint32_t rtos_counter = 0;
#endif

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

//=================================================================================================

/**
  * @brief:  Timer 15 Interrupt Service Routine handler (does include TIM1 break)
  * @param:  None (void input)
  * @retval: None (void output)
  */
void TIM1_BRK_TIM15_IRQHandler(void) {
    __HAL_TIM_CLEAR_IT(&htim15,             // Interrupt is only expected for TIM16 event
                       TIM_FLAG_UPDATE);    // interrupt, therefore clear flag

    _ihal::_itimer::time_counter++;         // Increment time keeper parameter

    // Only increment the RTOS timer if runtime states is enabled
#ifdef configGENERATE_RUN_TIME_STATS
    rtos_counter++;
#endif
}

/**
  * @brief:  Idle function call, which will basically just increment a counter, which can be used
  *          to determine how the load of functions varies over time.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void vApplicationIdleHook( void ) {
    _ihal::_itimer::idle_counter++;

}

// Functions used within the RTOS functions - so as to get run time states of which tasks are
// taking up a majority of time.
#ifdef configGENERATE_RUN_TIME_STATS
void configureTimerForRunTimeStats(void) { rtos_counter = 0 ; }
unsigned long getRunTimeCounterValue(void) { return (rtos_counter); }; // FreeRTOS counter
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None
