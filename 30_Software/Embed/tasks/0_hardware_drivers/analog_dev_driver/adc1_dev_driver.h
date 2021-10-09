/**************************************************************************************************
 * @file        adc1_dev_driver.h
 * @author      Thomas
 * @brief       Header file for ADC Device Driver task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * 
 * 
 *************************************************************************************************/
#ifndef ADC1_DEV_DRIVER_H_
#define ADC1_DEV_DRIVER_H_

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
// None

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
void vADC1DeviceHAL(void const * argument); // "main" task for ADC handle

void ADC1_IRQHandler(void);                 // This task file also includes the prototype used to
void DMA1_Channel1_IRQHandler(void);        // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* ADC1_DEV_DRIVER_H_ */