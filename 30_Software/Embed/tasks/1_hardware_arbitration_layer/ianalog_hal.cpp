/**************************************************************************************************
 * @file        iadc_hal.cpp
 * @author      Thomas
 * @brief       HAL interface layer for the ADC task (source)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/1_hardware_arbitration_layer/ianalog_hal.hpp"
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
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
namespace _iadc1 {
Semaphore<  Param<float>  > internal_voltage_reference = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore<  Param<float>  > cpu_temperature = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore<  Param<float>  > fan_voltage = {
        .content = { 0 },
        .lock    = LockState::kLocked
};
Semaphore<  Param<float>  > fan_current = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore<  Param<float>  > stepper_voltage = {
        .content = { 0 },
        .lock    = LockState::kLocked
};
Semaphore<  Param<float>  > stepper_current = {
        .content = { 0 },
        .lock    = LockState::kLocked
};
Semaphore<  Param<float>  > stepper_ic_limit = {
        .content = { 0 },
        .lock    = LockState::kLocked
};

Semaphore<_adc1_dev::_param::DevFlt> conversion_flt = {
        .content = _adc1_dev::_param::DevFlt::kInitialised,
        .lock    = LockState::kLocked
};
}

namespace _idac1 {
Semaphore< float > stepper_ic_limit_demand = {
        .content = { 0.00 },
        .lock    = LockState::kLocked
};

Semaphore<_dac1_dev::_param::DevFlt> realisation_flt = {
        .content = _dac1_dev::_param::DevFlt::kInitialised,
        .lock    = LockState::kLocked
};
}

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

//=================================================================================================

namespace _iadc1 {
/**
  * @brief:  Initialises all the ADC interface hal parameters.
  * @param:  void
  * @retval: void
  */
void interfaceInitialise(void) {
    internal_voltage_reference.content.data = -999.0f;
    internal_voltage_reference.content.flt  = FltState::kFaulty;
    internal_voltage_reference.lock         = LockState::kUnlocked;

    cpu_temperature.content.data            = -999.0f;
    cpu_temperature.content.flt             = FltState::kFaulty;
    cpu_temperature.lock                    = LockState::kUnlocked;

    fan_voltage.content.data                = -999.0f;
    fan_voltage.content.flt                 = FltState::kFaulty;
    fan_voltage.lock                        = LockState::kUnlocked;

    fan_current.content.data                = -999.0f;
    fan_current.content.flt                 = FltState::kFaulty;
    fan_current.lock                        = LockState::kUnlocked;

    stepper_voltage.content.data            = -999.0f;
    stepper_voltage.content.flt             = FltState::kFaulty;
    stepper_voltage.lock                    = LockState::kUnlocked;

    stepper_current.content.data            = -999.0f;
    stepper_current.content.flt             = FltState::kFaulty;
    stepper_current.lock                    = LockState::kUnlocked;

    stepper_ic_limit.content.data           = -999.0f;
    stepper_ic_limit.content.flt            = FltState::kFaulty;
    stepper_ic_limit.lock                   = LockState::kUnlocked;

    conversion_flt.content                  = _adc1_dev::_param::DevFlt::kInitialised;
    conversion_flt.lock                     = LockState::kUnlocked;
}

/**
  * @brief:  Set all the fault flags for the ADC parameters
  * @param:  void
  * @retval: void
  */
void setADCFaults(void){
    setFault(&internal_voltage_reference);
    setFault(&cpu_temperature);

    setFault(&fan_voltage);
    setFault(&fan_current);

    setFault(&stepper_voltage);
    setFault(&stepper_current);
    setFault(&stepper_ic_limit);
}

/**
  * @brief:  Clear all the fault flags for the ADC parameters
  * @param:  void
  * @retval: void
  */
void clearADCFaults(void) {
    clearFault(&internal_voltage_reference);
    clearFault(&cpu_temperature);

    clearFault(&fan_voltage);
    clearFault(&fan_current);

    clearFault(&stepper_voltage);
    clearFault(&stepper_current);
    clearFault(&stepper_ic_limit);
}
}

namespace _idac1 {
/**
  * @brief:  Initialises all the DAC interface hal parameters.
  * @param:  void
  * @retval: void
  */
void interfaceInitialise(void) {
    stepper_ic_limit_demand.content         = -999.0f;
    stepper_ic_limit_demand.lock            = LockState::kUnlocked;


    realisation_flt.content                 = _dac1_dev::_param::DevFlt::kInitialised;
    realisation_flt.lock                    = LockState::kUnlocked;
}
}
///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None
}
