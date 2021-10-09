/**************************************************************************************************
 * @file        timer_dev_interrupt.h
 * @author      Thomas
 * @brief       Header for the Timer Interrupt service routine
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This is the main header file for the main timers within the embedded device. Capturing the
 * interrupt call of the timer, as well as the RTOS idler.
 * The source file also includes the extra functions needed to populate the RTOS run time stats,
 * if this is enabled via STM32CubeMX.
 *************************************************************************************************/
#ifndef TIMER_DEV_INTERRUPT_H_
#define TIMER_DEV_INTERRUPT_H_

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
#include "main.h"                       // Include main header file, as this contains the defines
                                        // for GPIO signals
#include "stm32l4xx_hal.h"              // Include the HAL library

#ifdef __cplusplus
extern "C" {
#endif

//=================================================================================================

/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
/** READ ME BEFORE CHANGING 'ktask_rate' of fan / TIM15
  *
  * Currently the counter used to observe the task duration and periods is based upon TIM15 (also
  * used for Fan PWM), which currently set to:
  *     fcpu input          =   80MHz
  *     TIM15 Prescaler     =   3       = 80MHz  / (3    + 1)  = 20MHz          50ns
  *     Count Period        =   999     - 20MHz  / (999  + 1)  = 20KHz          50us
  *                                                                             0.05ms
  *     Therefore there are 20 counts to 1ms
  ************************************************************************************************/

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
void TIM1_BRK_TIM15_IRQHandler(void);       // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls

void vApplicationIdleHook( void );          // Idle function call

#ifdef __cplusplus
}
#endif
#endif /* TIMER_DEV_INTERRUPT_H_ */
