/**************************************************************************************************
 * @file        adc1_monitor.cpp
 * @author      Thomas
 * @brief       Source file for the ADC monitors
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/analog_dev_driver/adc1_monitors.hpp"
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
#include "tasks/0_hardware_drivers/analog_dev_driver/dac1_stepper_vref.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ianalog_hal.hpp"

//=================================================================================================

namespace _adc1_dev::_monitors {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
    float current_vref[ksequences] = { 0.00 };          // Array to contain the vref parameters

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
float averageParameter(float *param);

//=================================================================================================
/**
  * @brief:  Calculates the Internal voltage reference of STM32 device (this needs to be
  *          determined first for other conversions
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @retval: None (void output)
  */
void internalVoltage(ADCDMARecord *Record) {
    uint8_t dataloc = 0;    // Location within array for V

    for (uint8_t i = 0; i != ksequences; i++) {     // Loop through copies of data within record
        dataloc = (uint8_t) ( (kconversions * i) + kvref_location );
            // Calculate position of data within array

        if (Record->Analog[dataloc] == 0)           // If the data is 0
            current_vref[i]  = 0.00L;               // Set Vref to 0, to protect against divide by
                                                    // 0
        else
            current_vref[i]   = (float) ( 3.0L * (kvref_init /
                                                 ((float) Record->Analog[dataloc])) );
            // Convert the ADC entry to voltage reference
    }

    _ihal::pushValue(&_ihal::_iadc1::internal_voltage_reference,
                     averageParameter(&current_vref[0]));
}

/**
  * @brief:  Calculates the Temperature reading internal to STM32 device
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @retval: None (void output)
  */
void internalTemperature(ADCDMARecord *Record) {
    float temp_values[kconversions] = { 0.00 };
    uint8_t dataloc = 0;    // Location within array for V

    // Following variables are used to convert internal calibration registers to voltages (V) in
    // float format.
    float CAL1Volt = float ( ( 3.0L   *   kts_cal1 ) / kadc_resolution );
    float CAL2Volt = float ( ( 3.0L   *   kts_cal2 ) / kadc_resolution );

    for (uint8_t i = 0; i != ksequences; i++) {     // Loop through copies of data within record
        dataloc = (uint8_t) ( (kconversions * i) + kcpu_temp_location );
            // Calculate position of data within array

        temp_values[i]   = ( current_vref[i] * (float)Record->Analog[dataloc] ) / kadc_resolution;
            // Get the current voltage from Temperature sensor
        temp_values[i]   -=  CAL1Volt;      // Account for first calibration voltage
        temp_values[i]   *=  (kts_cal2_tmp  -  kts_cal1_tmp) / (CAL2Volt  -  CAL1Volt);
            // Multiply value by conversion slope
        temp_values[i]   +=  kts_cal1_tmp;  // Again offset based upon first calibration
                                            // temperature
    }

    _ihal::pushValue(&_ihal::_iadc1::cpu_temperature,
                     averageParameter(&temp_values[0]));
}

/**
  * @brief:  Calculates the Fan Voltage being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @retval: None (void output)
  */
void fanMotorVoltage(ADCDMARecord *Record) {
    float temp_values[kconversions] = { 0.00 };
    uint8_t dataloc = 0;    // Location within array for V

    for (uint8_t i = 0; i != ksequences; i++) {     // Loop through copies of data within record
        dataloc = (uint8_t) ( (kconversions * i) + kfan_v_location );
            // Calculate position of data within array

        temp_values[i] = ( current_vref[i] * (float)Record->Analog[dataloc] ) / kadc_resolution;
        temp_values[i] *= (  (kfan_r_upper + kfan_r_lower)  /  kfan_r_lower  );
        // Convert the ADC entry to Fan Voltage
    }

    _ihal::pushValue(&_ihal::_iadc1::fan_voltage,
                     averageParameter(&temp_values[0]));
}

/**
  * @brief:  Calculates the Stepper Voltage being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @retval: None (void output)
  */
void stepperMotorVoltage(ADCDMARecord *Record) {
    float temp_values[kconversions] = { 0.00 };
    uint8_t dataloc = 0;    // Location within array for V

    for (uint8_t i = 0; i != ksequences; i++) {     // Loop through copies of data within record
        dataloc = (uint8_t) ( (kconversions * i) + kstepper_v_location );
            // Calculate position of data within array

        temp_values[i] = ( current_vref[i] * (float)Record->Analog[dataloc] ) / kadc_resolution;
        temp_values[i] *= (  (kstp_r_upper + kstp_r_lower)  /  kstp_r_lower  );
        // Convert the ADC entry to Stepper Voltage
    }

    _ihal::pushValue(&_ihal::_iadc1::stepper_voltage,
                     averageParameter(&temp_values[0]));
}

/**
  * @brief:  Calculates the Stepper IC limit being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @retval: None (void output)
  */
void stepperMotorICLimit(ADCDMARecord *Record) {
float temp_values[kconversions] = { 0.00 };
uint8_t dataloc = 0;    // Location within array for V

for (uint8_t i = 0; i != ksequences; i++) {     // Loop through copies of data within record
    dataloc = (uint8_t) ( (kconversions * i) + kstepper_ic_limit_location );
        // Calculate position of data within array

    temp_values[i] = ( current_vref[i] * (float)Record->Analog[dataloc] ) / kadc_resolution;
    temp_values[i] = temp_values[i] / _dac1_dev::_stepper_vref::ksense_resistor;
    // Convert the ADC entry to Stepper Peak Current Load
}

_ihal::pushValue(&_ihal::_iadc1::stepper_ic_limit,
                 averageParameter(&temp_values[0]));
}

/**
  * @brief:  Calculates the Fan Current being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @retval: None (void output)
  */
void fanMotorCurrent(ADCDMARecord *Record) {
    float temp_values[kconversions] = { 0.00 };
    uint8_t dataloc = 0;    // Location within array for V

    for (uint8_t i = 0; i != ksequences; i++) {     // Loop through copies of data within record
        dataloc = (uint8_t) ( (kconversions * i) + kfan_i_location );
            // Calculate position of data within array

        temp_values[i] = ( current_vref[i] * (float)Record->Analog[dataloc] ) / kadc_resolution;
        temp_values[i] *= (  1  / (kfan_cs30 * kfan_r_sense)  );
        // Convert the ADC entry to Fan Current
    }

    _ihal::pushValue(&_ihal::_iadc1::fan_current,
                     averageParameter(&temp_values[0]));
}

/**
  * @brief:  Calculates the Stepper Current being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @retval: None (void output)
  */
void stepperMotorCurrent(ADCDMARecord *Record) {
    float temp_values[kconversions] = { 0.00 };
    uint8_t dataloc = 0;    // Location within array for V

    for (uint8_t i = 0; i != ksequences; i++) {     // Loop through copies of data within record
        dataloc = (uint8_t) ( (kconversions * i) + kstepper_i_location );
            // Calculate position of data within array

        temp_values[i] = ( current_vref[i] * (float)Record->Analog[dataloc] ) / kadc_resolution;
        temp_values[i] *= (  1  / (kstp_cs30 * kstp_r_sense)  );
        // Convert the ADC entry to Stepper Current
    }

    _ihal::pushValue(&_ihal::_iadc1::stepper_current,
                     averageParameter(&temp_values[0]));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @brief:  Calculates the average of the parameter array, and puts output into first entry.
  *          Starts the averaging from the second entry.
  * @retval: float value containing the average of input array
  */
float averageParameter(float *param) {
    float accumulator = param[0];   // Initialise the average output to first data point

    for (uint8_t i = 0; i != (ksequences - 1); i++) {   // Loop through copies of data
        accumulator += param[i + 1];                    // Add the next parameter to accumulator
    }

    accumulator = accumulator / ((float) ksequences);   // Divide accumulation by number of
                                                        // parameters so as to get the average
    return ( accumulator );
}
}
