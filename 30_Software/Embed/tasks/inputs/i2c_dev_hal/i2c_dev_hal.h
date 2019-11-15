/**************************************************************************************************
 * @file        i2c_dev_hal.h
 * @author      Thomas
 * @version     V2.1
 * @date        28 Sept 2019
 * @brief       Header file for I2C Device task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 *
 * This contains the loop required to manage the I2C device(s) attached within the miStepper
 * hardware.
 * This task will setup:
 *      I2C1 class for hardware/Interrupt Handling
 *      AD7415 data communication class
 *
 *  This will then periodically check the status of the I2C interrupt management and communication
 *  status. If there is new data, the AD7415 class will be updated with the latest data, and the
 *  current value of external temperature will be an output of the task.
 *
 *************************************************************************************************/

#ifndef I2C_DEV_HAL_H_
#define I2C_DEV_HAL_H_

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
// None

/**************************************************************************************************
 * Define externally used global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#define     I2CWt_Arry      0           // GenBuffer array entry which contains data to be written
                                        // to target I2C device - this is to be used for "generic"
                                        // write data to device; no read back required
#define     I2C_AD7415_G    1           // GenBuffer array entry which contains data read back
                                        // from AD7415_1 device (Address AD7415_1 GND)

/**************************************************************************************************
 * Prototypes for functions used externally
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/

void vI2C1DeviceHAL(void const * argument); // "main" task for I2C handle

void I2C1_EV_IRQHandler(void);              // This task file also includes the prototype used to
void I2C1_ER_IRQHandler(void);              // handle Interrupt Service Calls

#ifdef __cplusplus
}
#endif
#endif /* TASKS_INPUTS_I2C_DEV_HAL_I2C_DEV_HAL_H_ */
