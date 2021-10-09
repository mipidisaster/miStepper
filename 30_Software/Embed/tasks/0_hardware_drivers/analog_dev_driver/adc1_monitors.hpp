/**************************************************************************************************
 * @file        adc1_monitors.hpp
 * @author      Thomas
 * @brief       Header file for the ADC monitors
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Script to contain the generation and handling of the in-build ADC(1), and the conversion of the
 * read voltage to the correct values as per the system (i.e. any potential dividers, etc.)
 *************************************************************************************************/
#ifndef ADC1_MONITORS_HPP_
#define ADC1_MONITORS_HPP_

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

//=================================================================================================

namespace _adc1_dev::_monitors {
/**************************************************************************************************
 * Exported MACROS
 * ~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Exported Variables
 * ~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
inline constexpr uint8_t    ksequences      = 5;    // Number of sequences that can be contained
                                                    // a single DMA record

inline constexpr uint8_t    kvref_location              = 0;
inline constexpr uint8_t    kcpu_temp_location          = 1;
inline constexpr uint8_t    kstepper_v_location         = 2;
inline constexpr uint8_t    kstepper_i_location         = 3;
inline constexpr uint8_t    kfan_v_location             = 4;
inline constexpr uint8_t    kfan_i_location             = 5;
inline constexpr uint8_t    kstepper_ic_limit_location  = 6;

inline constexpr uint8_t    kconversions                = 7;
    // Number of conversions per sequence

// CPU Vreference
// ~~~~~~~~~~~~~~
inline const uint16_t       kvref_init      = *((uint16_t*) 0x1FFF75AAUL);
inline constexpr uint16_t   kadc_resolution = 0x0FFF;

// CPU Temperature parameters
// ~~~~~~~~~~~~~~~~~~~~~~~~~~
inline const uint16_t       kts_cal1        = *((uint16_t*) 0x1FFF75A8UL);
inline constexpr uint8_t    kts_cal1_tmp    = 30L;

inline const uint16_t       kts_cal2        = *((uint16_t*) 0x1FFF75CAUL);
inline constexpr uint8_t    kts_cal2_tmp    = 130L;

// FAN Potential Divider & Current monitor
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
inline constexpr float      kfan_r_upper    = 9.76f;
inline constexpr float      kfan_r_lower    = 6.49f;
inline constexpr float      kfan_r_sense    = 0.05f;
inline constexpr float      kfan_cs30       = 100.0f;

// STEPPER Potential Divider & Current monitor
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
inline constexpr float      kstp_r_upper    = 9.76f;
inline constexpr float      kstp_r_lower    = 1.96f;
inline constexpr float      kstp_r_sense    = 0.05f;
inline constexpr float      kstp_cs30       = 20.0f;

/**************************************************************************************************
 * Exported types
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
struct ADCDMARecord {                                   // ADC Record structure
    uint16_t    Analog[_monitors::ksequences * _monitors::kconversions];
                // Array to contain all conversions for defined number of runs
    _param::DevFlt Flt;
};

/**************************************************************************************************
 * Exported function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void internalVoltage(ADCDMARecord *Record);
void internalTemperature(ADCDMARecord *Record);

void fanMotorVoltage(ADCDMARecord *Record);
void fanMotorCurrent(ADCDMARecord *Record);

void stepperMotorVoltage(ADCDMARecord *Record);
void stepperMotorCurrent(ADCDMARecord *Record);
void stepperMotorICLimit(ADCDMARecord *Record);
}
#endif /* ADC1_MONITORS_HPP_ */
