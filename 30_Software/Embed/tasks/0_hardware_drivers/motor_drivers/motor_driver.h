/**************************************************************************************************
 * @file        motor_driver.h
 * @author      Thomas
 * @brief       Header file for Motor(s) task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This is the main header file for the motor driver(s) - FAN and STEPPER. Managing the initial
 * setup of the Timers/PWM/classes/extra GPIOs connected/etc.
 *
 * The files will be structured as such:
 *    "motor_driver"                -> This will include the main task which will be looped
 *    "motor_driver_parameters"     -> This will include constants, etc. which are used within the
 *                                     motor task
 *                                     >> NOTE <<
 *                                     The FAN and STEPPER drivers will have separate parameters /
 *                                     files to be called.
 *************************************************************************************************/
#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

/**************************************************************************************************
 * Include all files that are needed to understand this header
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// C System Header(s)
// ------------------
// None

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
#define zz__StepperDMAControl__zz
#ifdef zz__StepperDMAControl__zz
/* If enabling the DMA control then, within the CubeMX options:
 *      > Ensure that the interrupt for 'DMA1 Channel 7' is enabled (NVIC)
 *          (Also untick the 'Generate IRQ handler')
 *          (Also untick the interrupt for 'TIM1 update interrupt')
 *      > Within TIM1 update the 'auto-reload preload' to Disable
 */
#else
/* If enabling the TIM control then, within the CubeMX options:
 *      > Ensure that the interrupt for 'TIM1 update interrupt' is enabled (NVIC)
 *          (Also untick the 'Generate IRQ handler')
 *          (Also untick the interrupt for 'DMA1 Channel 7')
 *      > Within TIM1 update the 'auto-reload preload' to Enable
 */
#endif
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
void vMotorHAL(void const * argument);      // "main" task for Motor handle(s)

#ifdef zz__StepperDMAControl__zz
void DMA1_Channel7_IRQHandler(void);        // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls
#else
void TIM1_UP_TIM16_IRQHandler(void);        // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls

#endif
#ifdef __cplusplus
}
#endif
#endif /* MOTOR_DRIVER_H_ */
