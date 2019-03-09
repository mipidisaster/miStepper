/**************************************************************************************************
 * @file        stepper_hal.h
 * @author      Thomas
 * @version     V0.1
 * @date        09 Mar 2019
 * @brief       Header file for Stepper Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 *
 * This contains the loop required to manage the Stepper Motor hardware
 * This task will setup:
 *      Auxillery GPIOs for Stepper IC
 *      Stepper class for hardware/Interrupt Handling
 *
 * This will then periodically read the current request inputs, and if there is a request to
 * enable/move stepper a specific sequence of movement will be made. The current demand will then
 * provided as output of task.
 *
 * When task is triggered, it will be expecting a void type casted parameter containing:
 *      Configuration parameters    (DMA, and TIM global parameters)
 *      Input signals               (Stepper requests)
 *      Output signals              (Actual Stepper Demand)
 *
 *************************************************************************************************/

#ifndef STEPPER_HAL_H_
#define STEPPER_HAL_H_

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

#include FilInd_GPIO___HD               // Include the GPIO class handler
#include FilInd_Stppr__HD               // Include the Stepper class handler

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
typedef struct {
    struct {
        TIM_HandleTypeDef                                   *dev_timer;
        DMA_HandleTypeDef                                   *dev_dma;

    } config;

    struct {
        uint8_t                                             *enable;
        uint8_t                                             *move;

    } input;

    struct {
        _HALParam                                           *movement;

    } output;

}   _taskSTP;

/**************************************************************************************************
 * MACROs used within task
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Define externally used global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None


/**************************************************************************************************
 * Prototypes for functions used externally
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void vSTPMotorHAL(void const * pvParameters);   // "main" task for Stepper Motor handle

void TIM1_UP_TIM16_IRQHandler(void);        // This task file also includes the prototype used to
void TIM1_CC_IRQHandler(void);              // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* STEPPER_HAL_H_ */
