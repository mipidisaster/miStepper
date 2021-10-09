/**************************************************************************************************
 * @file        dac1_stepper_vref.cpp
 * @author      Thomas
 * @brief       << Manually Entered >>
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/analog_dev_driver/dac1_stepper_vref.hpp"
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
#include "tasks/0_hardware_drivers/analog_dev_driver/dac1_dev_driver_parameters.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ianalog_hal.hpp"

//=================================================================================================

namespace _dac1_dev::_stepper_vref {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

//=================================================================================================

/**
  * @brief:  Calculates the required DAC output for the requested Stepper IC peak load
  * @param:  Float input with the demand for IC peak current
  * @retval: return required DAC register value
  */
uint16_t calcStepperMotorICLimit(float ic_current_demand) {
    float temp_value = ic_current_demand * _dac1_dev::_stepper_vref::ksense_resistor;
        // Create a temporary variable to contain the calcation, and initialise with the required
        // Vref for desired IC current.
        //  Calculated by multiplying the current by sense resistor (see page 16 of STSPIN820
        //  datasheet)

    temp_value *= (  (kvref_r_upper + kvref_r_lower)  /  kvref_r_lower  );
        // Take into account the potential divider on the DAC output.
    // temp_value is now the required voltage the DAC output needs to be.
    temp_value *= _dac1_dev::_param::resolution_mask;   // Multiple by the DAC resolution

    if (_ihal::getFault(&_ihal::_iadc1::internal_voltage_reference) == _ihal::FltState::kNoFault )
    {
        temp_value = temp_value / _ihal::getValue(&_ihal::_iadc1::internal_voltage_reference);
    }
    else {
        temp_value = temp_value / 3.3f;
    }

    return ((uint16_t)  temp_value );
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
