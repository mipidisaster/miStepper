/**************************************************************************************************
 * @file        usart_dev_hal.h
 * @author      Thomas
 * @version     V2.1
 * @date        28 Sept 2019
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
#include FilInd_DATMngrHD               // Provide the function set for Data Manipulation

#include FilInd_USART__HD               // Include the USART class handler
#include FilIndUSARTDMAHD               // Include the USART/DMA class handler

#include FilInd_SPIPe__HD               // Include the SPI class handler
#include FilInd_AS5x4x_HD               // Include the device AS5x4 handler

#include FilInd_I2CPe__HD               // Include the I2C class handler
#include FilInd_AD741x_HD               // Include the device AD741x handler
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
#define TransmitDataBit     0       // Bit position for transmitting data to be displayed
#define ResetPktCountBt     1       // Bit position for clearing the Package counter

#define EnableInputBit      3       // Bit position for enabling interface with ROS

#define MISTEPPER_RECEIVE   18      // Number of bytes for receiving requests

/**************************************************************************************************
 * Define externally used global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Prototypes for functions used externally
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/

void vUSART1DeviceHAL(void const * argument);   // "main" task for USART handle

void USART1_IRQHandler(void);               // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls
void DMA1_Channel4_IRQHandler(void);        // Interrupt Service Call for DMA1 Channel 4 (Tx)
void DMA1_Channel5_IRQHandler(void);        // Interrupt Service Call for DMA1 Channel 5 (Rx)

#ifdef __cplusplus
}
#endif
#endif /* USART_DEV_HAL_H_ */
