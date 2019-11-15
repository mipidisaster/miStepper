/**************************************************************************************************
 * @file        spi_dev_hal.h
 * @author      Thomas
 * @version     V2.1
 * @date        28 Sept 2019
 * @brief       Header file for SPI Device task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 *
 * This contains the loop required to manage the SPI device(s) attached within the miStepper
 * hardware.
 * This task will setup:
 *      SPI1 class for hardware/Interrupt Handling
 *      AS5048 data communication class
 *
 * This will then periodically check the status of the SPI interrupt management and communication
 * status. If there is new data, the AS5048 class will be updated with the latest data, and the
 * current value of angular position will be an output of the task
 *
 *************************************************************************************************/
#ifndef SPI_DEV_HAL_H_
#define SPI_DEV_HAL_H_

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
#include FilInd_GPIO___HD               // Include the GPIO class handler
#include FilInd_SPIPe__HD               // Include the SPI class handler
#include FilInd_AS5x4x_HD               // Include the device AS5x4 handler
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
#define     SPIWt_Arry      0           // GenBuffer array entry which contains data to be written
                                        // to target SPI device
#define     SPIRd_Arry      1           // GenBuffer array entry which contains data to be read
                                        // from target SPI device

/**************************************************************************************************
 * Define externally used global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Prototypes for functions used externally
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/

void vSPI1DeviceHAL(void const * argument); // "main" task for SPI handle

void SPI1_IRQHandler(void);                 // This task file also includes the prototype used to
                                            // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* SPI_DEV_HAL_H_ */
