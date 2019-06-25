/**************************************************************************************************
 * @file        adc_hal.h
 * @author      Thomas
 * @version     V0.1
 * @date        09 Mar 2019
 * @brief       Header file for ADC input task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 *
 * This contains the loop required to manage the ADC1, and the expected analog inputs.
 * This task will setup:
 *      ADC1 hardware/DMA linkage and Interrupt Handling
 *
 * This will then periodically check for a complete 'record' of ADC data, then convert these
 * conversions into a more useful format (i.e. Voltage, Current, Temperature, etc.), and provide
 * the current values of each as output of task.
 *
 * When task is triggered, it will be expecting a void type casted parameter containing:
 *      Configuration parameters    (ADC, DMA, and TIM global parameters)
 *      Input signals               (NONE)
 *      Output signals              (Internal Voltage, Temperature, Fan Voltage and Current,
 *                                   Stepper Motor Voltage and Current)
 *
 *************************************************************************************************/

#ifndef ADC_HAL_H_
#define ADC_HAL_H_

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

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class
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
        ADC_HandleTypeDef                                   *adc1_handle;
        DMA_HandleTypeDef                                   *adc1_dma;
        TIM_HandleTypeDef                                   *adc1_timer;

    } config;

    struct {
        _HALParam                                           *IntVrf;
        _HALParam                                           *IntTmp;

        _HALParam                                           *FanVlt;
        _HALParam                                           *FanCur;

        _HALParam                                           *StpVlt;
        _HALParam                                           *StpCur;

    } output;

}   _taskADC1;

/**************************************************************************************************
 * MACROs used within task
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// STM32 Hardware register location for Internal Temperature and VRef calibration
#define TS_CAL1         *((uint16_t*) 0x1FFF75A8UL)     // 1034
#define TSCAL1Tmp       30L                             // Tempature at calibration point 1
#define TS_CAL2         *((uint16_t*) 0x1FFF75CAUL)     // 1367
#define TSCAL2Tmp       130L                            // Tempature at calibration point 2

#define VREFINT         *((uint16_t*) 0x1FFF75AAUL)

#define ADC_Resolution  4095L

// miStepper board configuration ('StepperController_v1-0')
#define FAN_RUpper      60.4L       //
#define FAN_RLower      40.2L       //
#define FAN_Rsense      0.05L       //
#define FAN_CS30        100L        //

#define STP_RUpper      60.4L       //
#define STP_RLower      12.1L       //
#define STP_Rsense      0.05L       //
#define STP_CS30        20L         //

// Signal position within ADC sequence conversion
//  To reorder, need to update miStepper.ioc first
#define VRefLoc         0        // Conversion location for internal voltage reference
#define ITmpLoc         1        // Conversion location for internal temperature reading
#define STPVLoc         2        // Conversion location for Stepper Motor Voltage (Volts) reading
#define STPILoc         3        // Conversion location for Stepper Motor Current (Amps)  reading
#define FANVLoc         4        // Conversion location for FAN Motor Voltage     (Volts) reading
#define FANILoc         5        // Conversion location for FAN Motor Current     (Amps)  reading

/**************************************************************************************************
 * Define externally used global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None


/**************************************************************************************************
 * Prototypes for functions used externally
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/

void vADC1DeviceHAL(void const * pvParameters); // "main" task for ADC handle

void ADC1_IRQHandler(void);                 // This task file also includes the prototype used to
void DMA1_Channel1_IRQHandler(void);        // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* ADC_HAL_H_ */
