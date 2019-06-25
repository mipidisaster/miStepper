
/**************************************************************************************************
 * @file        EmbedIndex.h
 * @author      Thomas
 * @version     V0.2
 * @date        25 Jun 2019
 * @brief       File index for miStepper device
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This header is used to allow for "Computed Includes" within all the code and functions that I
 * have created.
 * This will contain a list of defines, which can be changed allowing for a single point of update
 * for all of the files. This will allow for re-arranging of the folder structure without having
 * to update ALL of the files.
 *************************************************************************************************/
#ifndef EMDEDINDEX_H_
#define EMBEDINDEX_H_

#include <FileIndex.h>      // Include file structure for the Library
/**************************************************************************************************
 * Input Acquisition Tasks
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#define EMBD_SPITask        "tasks\inputs\spi_dev_hal\spi_dev_hal.h"
#define EMBD_I2CTask        "tasks\inputs\i2c_dev_hal\i2c_dev_hal.h"
#define EMBD_ADCTask        "tasks\inputs\adc_hal\adc_hal.h"

/**************************************************************************************************
 * Output Controller Tasks
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#define EMBD_FANTask        "tasks\outputs\fan_hal\fan_hal.h"
#define EMBD_STPTask        "tasks\outputs\stepper_hal\stepper_hal.h"

/**************************************************************************************************
 * External communication Tasks
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#define EMBD_USARTTask      "tasks\comms\usart_dev_hal\usart_dev_hal.h"


#endif /* EMBEDINDEX_H_ */
