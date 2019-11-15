/**************************************************************************************************
 * @file        fan_hal.h
 * @author      Thomas
 * @version     V2.1
 * @date        28 Sept 2019
 * @brief       Header file for Fan Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 *
 * This contains the loop required to manage the Fan Motor/PWM hardware.
 * This task will setup:
 *      PWM for modulated fan control
 *
 * This will then periodically read in the current Fan Demand, convert this to the required PWM
 * width and update the PWM generator. If the demand is outside of width, the demand will be
 * saturated, and provided as output of the task.
 *
 *************************************************************************************************/

#ifndef FAN_HAL_H_
#define FAN_HAL_H_

/**************************************************************************************************
 * Include all files that support this task/thread
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#include "EmbedIndex.h"                 // Include generic code library, as well as embedded
                                        // device specific functions
#include "miStepperCONFIG.h"            // Include miStepper Configuration parameters
                                        //
#include "stm32l4xx_hal.h"              // Include the HAL library
#include "main.h"                       // Include main header file, as this contains the defines
                                        // for GPIO signals
#include "math.h"                       // Include Math library
// #####
// ##   RTOS:
// #####
#include "cmsis_os.h"                   // Header for introducing FreeRTOS to device

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 * Define externally used structures
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * MACROs used within task
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// FAN Driver
#define PWMWidth             999L           //

/**************************************************************************************************
 * Define externally used global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Prototypes for functions used externally
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/

void vFANMotorHAL(void const * argument);   // "main" task for FAN handle

#ifdef __cplusplus
}
#endif
#endif /* FAN_HAL_H_ */
