/**************************************************************************************************
 * @file        usart1_interfaces.cpp
 * @author      Thomas
 * @brief       Source file for USART interfaces (to the miStepperUSART class)
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/usart_dev_driver/usart1_interfaces.hpp"
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
#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

// IHAL parameters:
#include "tasks/1_hardware_arbitration_layer/ispi_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/ii2c_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/ianalog_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/ifan_hal.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilIndMStpUARTHD               // Include the miStepper USART protocol class

//=================================================================================================

namespace _usart1_dev::_interfaces {
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
  * @brief:  Connect all the required spi1 parameters to the 'mistepper_handle' ready for
  *          transmission
  * @param:  miStepperUSART pointer
  * @retval: None (void output)
  */
void    spi1(miStepperUSART     *mistepper_handle) {
    mistepper_handle->angular_position              = *(uint32_t *)
                    &_ihal::_ispi1::ang_pos_raw.content.data;

    mistepper_handle->spi1_fault                    = (uint8_t) _ihal::_ispi1::comm_flt.content;
    mistepper_handle->angle_sensor_spi_fault        = (uint8_t)
                    _ihal::_ispi1::angle_sensor_flt.content.ComFlt;
    mistepper_handle->angle_sensor_fault            = (uint8_t)
                    _ihal::_ispi1::angle_sensor_flt.content.DevFlt;
    mistepper_handle->angle_sensor_idle_count       = (uint8_t)
                    _ihal::_ispi1::angle_sensor_flt.content.IdleCount;

    mistepper_handle->spi1_task_time                = (uint32_t)
                    _ihal::_itimer::timeStates(_ihal::TimedTasks::kSPI1);
}

/**
  * @brief:  Connect all the required i2c1 parameters to the 'mistepper_handle' ready for
  *          transmission
  * @param:  miStepperUSART pointer
  * @retval: None (void output)
  */
void    i2c1(miStepperUSART     *mistepper_handle) {
    mistepper_handle->internal_temperature_top      = *(uint32_t *)
                    &_ihal::_ii2c1::internal_top_temp_raw.content.data;

    mistepper_handle->i2c1_fault                    = (uint8_t) _ihal::_ii2c1::comm_flt.content;
    mistepper_handle->top_temp_sensor_i2c_fault     = (uint8_t)
                _ihal::_ii2c1::top_temp_sensor_flt.content.ComFlt;
    mistepper_handle->top_temp_sensor_fault         = (uint8_t)
                _ihal::_ii2c1::top_temp_sensor_flt.content.DevFlt;
    mistepper_handle->top_temp_sensor_idle_count    = (uint8_t)
                _ihal::_ii2c1::top_temp_sensor_flt.content.IdleCount;

    mistepper_handle->i2c1_task_time                = (uint32_t)
                _ihal::_itimer::timeStates(_ihal::TimedTasks::kI2C1);
}

/**
  * @brief:  Connect all the required adc1 parameters to the 'mistepper_handle' ready for
  *          transmission
  * @param:  miStepperUSART pointer
  * @retval: None (void output)
  */
void    adc1(miStepperUSART     *mistepper_handle) {
    mistepper_handle->internal_voltage_reference    = *(uint32_t *)
                &_ihal::_iadc1::internal_voltage_reference.content;
    mistepper_handle->cpu_temperature               = *(uint32_t *)
                &_ihal::_iadc1::cpu_temperature.content;
    mistepper_handle->fan_voltage                   = *(uint32_t *)
                &_ihal::_iadc1::fan_voltage.content;
    mistepper_handle->fan_current                   = *(uint32_t *)
                &_ihal::_iadc1::fan_current.content;
    mistepper_handle->stepper_voltage               = *(uint32_t *)
                &_ihal::_iadc1::stepper_voltage.content;
    mistepper_handle->stepper_current               = *(uint32_t *)
                &_ihal::_iadc1::stepper_current.content;
    mistepper_handle->conversion_fault              = *(uint32_t *)
                &_ihal::_iadc1::conversion_flt.content;

    mistepper_handle->adc1_task_time                = (uint32_t)
                _ihal::_itimer::timeStates(_ihal::TimedTasks::kADC1);
}

/**
  * @brief:  Connect all the required fan parameters to the 'mistepper_handle' ready for
  *          transmission
  * @param:  miStepperUSART pointer
  * @retval: None (void output)
  */
void    fan(miStepperUSART      *mistepper_handle) {
    mistepper_handle->fan_demand                    = *(uint32_t *)
                &_ihal::_ifan::fan_demand.content;
    mistepper_handle->fan_task_time                 = (uint32_t)
                _ihal::_itimer::timeStates(_ihal::TimedTasks::kFAN);
}

/**
  * @brief:  Connect all the required stepper parameters to the 'mistepper_handle' ready for
  *          transmission
  * @param:  miStepperUSART pointer
  * @retval: None (void output)
  */
void    stepper(miStepperUSART  *mistepper_handle) {
    //mistepper_handle->stepper_frequency            =
    //mistepper_handle->stepper_gear                 =
    //mistepper_handle->stepper_calc_position        =
    mistepper_handle->stepper_task_time             = (uint32_t)
                _ihal::_itimer::timeStates(_ihal::TimedTasks::kSTEPPER);
}

/**
  * @brief:  Connect all the required usart1 parameters to the 'mistepper_handle' ready for
  *          transmission
  * @param:  miStepperUSART pointer
  * @retval: None (void output)
  */
void    usart1(miStepperUSART   *mistepper_handle) {
    mistepper_handle->usart1_task_time              = (uint32_t)
                _ihal::_itimer::timeStates(_ihal::TimedTasks::kUSART1);
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
