/**************************************************************************************************
 * @file        usart_dev_hal.h
 * @author      Thomas
 * @version     V0.1
 * @date        09 Mar 2019
 * @brief       Header file for USART communication task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 *
 * This contains the loop required to manage the USART interface to a computer for troubleshooting
 * and system identification activities.
 * This task will setup:
 *      USART1 class for hardware/Interrupt Handling
 *
 * This will then periodically display parameters captured within the 'miStepper' system, i.e.
 * angular position, temperature, etc. Will also read in characters from USART, and depending upon
 * the characters will enable/disable specific functions (Fan and Stepper).
 *
 * When task is triggered, it will be expecting a void type casted parameter containing:
 *      Configuration parameters    (USART global parameter)
 *      Input signals               (Angular position, temperatures, voltages, and currents)
 *      Output signals              (Fan demand, Stepper motor demand(s))
 *
 *************************************************************************************************/
#ifndef USART_DEV_HAL_H_
#define USART_DEV_HAL_H_

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
#include <stdio.h>                      // Include the standard I/O library

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class
#include FilInd_USART__HD               // Include the USART class handler
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
        UART_HandleTypeDef                                  *dev_handle;

    } config;

    struct {
        _HALParam                                           *AngPos;
        _HALParam                                           *ExtTmp;

        _HALParam                                           *IntVrf;
        _HALParam                                           *IntTmp;

        _HALParam                                           *FanVlt;
        _HALParam                                           *FanCur;
        _HALParam                                           *FanAct;

        _HALParam                                           *StpVlt;
        _HALParam                                           *StpCur;
        _HALParam                                           *movement;

    } input;

    struct {
        _HALParam                                           *FanDmd;

        uint8_t                                             *enable;
        uint8_t                                             *move;

    } output;

}   _taskUSART;

/**************************************************************************************************
 * MACROs used within task
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#define MaxCharinLine           50      // Define maximum number of characters per line


#define TransmitData            0       // Bit position for transmitting data to be displayed

// Following modes are only enabled if TransmitData is set TRUE
#define FanEnable               1       // Bit position for enabling the fan motor
#define StpEnable               2       // Bit position for enabling the stepper motor
#define StpMove                 3       // Bit position for moving the stepper motor

/**************************************************************************************************
 * Define externally used global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Prototypes for functions used externally
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/

void vUSARTDeviceHAL(void const * pvParameters);    // "main" task for USART handle

void USART1_IRQHandler(void);               // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* USART_DEV_HAL_H_ */
