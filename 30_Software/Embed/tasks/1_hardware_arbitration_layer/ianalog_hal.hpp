/**************************************************************************************************
 * @file        iadc_hal.hpp
 * @author      Thomas
 * @brief       HAL interface layer for the ADC task (header)
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
#ifndef IADC_HAL_HPP_
#define IADC_HAL_HPP_

/**************************************************************************************************
 * Include all files that are needed to understand this header
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
// None

// Project Libraries
// -----------------
#include "tasks/0_hardware_drivers/analog_dev_driver/adc1_dev_driver_parameters.hpp"
#include "tasks/0_hardware_drivers/analog_dev_driver/dac1_dev_driver_parameters.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

//=================================================================================================

namespace _ihal {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
namespace _iadc1 {
extern Semaphore<  Param<float>  > internal_voltage_reference;
extern Semaphore<  Param<float>  > cpu_temperature;

extern Semaphore<  Param<float>  > fan_voltage;
extern Semaphore<  Param<float>  > fan_current;
extern Semaphore<  Param<float>  > stepper_voltage;
extern Semaphore<  Param<float>  > stepper_current;
extern Semaphore<  Param<float>  > stepper_ic_limit;

extern Semaphore<_adc1_dev::_param::DevFlt> conversion_flt;
}

namespace _idac1 {
extern Semaphore< float > stepper_ic_limit_demand;

extern Semaphore<_dac1_dev::_param::DevFlt> realisation_flt;
}
/**************************************************************************************************
 * Exported function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
namespace _iadc1 {
void interfaceInitialise(void);         // Initialise all iadc parameters

void setADCFaults(void);
void clearADCFaults(void);

}

namespace _idac1 {
void interfaceInitialise(void);         // Initialise all idac parameters

}
}
#endif /* IADC_HAL_HPP_ */
