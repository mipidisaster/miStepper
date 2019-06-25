/**************************************************************************************************
 * @file        usart_dev_hal.h
 * @author      Thomas
 * @version     V1.1
 * @date        25 Jun 2019
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
#include FilInd_DATMngrHD               // Provide the function set for Data Manipulation

#include FilInd_USART__HD               // Include the USART class handler

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
typedef struct {
    struct {
        UART_HandleTypeDef                                  *usart1_handle;

    } config;

    struct {
        _HALParam                                           *AngPos;
        SPIPeriph::DevFlt                                   *SPI1CommFlt;
        HALDevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt>     *AS5048AFlt;

        _HALParam                                           *ExtTmp;
        I2CPeriph::DevFlt                                   *I2C1CommFlt;
        HALDevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt>     *AD74151Flt;

        _HALParam                                           *IntVrf;
        _HALParam                                           *IntTmp;

        _HALParam                                           *FanVlt;
        _HALParam                                           *FanCur;
        _HALParam                                           *FanAct;

        _HALParam                                           *StpVlt;
        _HALParam                                           *StpCur;
        uint16_t                                            *StpFreqAct;
        uint16_t                                            *StpStatAct;
        uint32_t                                            *StpcalPost;

    } input;

    struct {
        float                                               *FanDmd;

        uint8_t                                             *StpEnable;
        uint8_t                                             *StpGear;
        uint8_t                                             *StpDirct;
        uint16_t                                            *StpFreqDmd;

    } output;

}   _taskUSART1;

/**************************************************************************************************
 * MACROs used within task
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#define TransmitDataBit     0       // Bit position for transmitting data to be displayed
#define ResetPktCountBt     1       // Bit position for clearing the Package counter

#define EnableInputBit      3       // Bit position for enabling interface with ROS

/**************************************************************************************************
 * Define externally used global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Prototypes for functions used externally
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/

void vUSART1DeviceHAL(void const * pvParameters);   // "main" task for USART handle

void USART1_IRQHandler(void);               // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* USART_DEV_HAL_H_ */
