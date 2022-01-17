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
#include "tasks/1_hardware_arbitration_layer/istepper_hal.hpp"

#include "mistepper_driver/miStepperUSART.h"    // Header for miStepper UART interface

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
    mistepper_handle->angular_position              = (float)
                    _ihal::_ispi1::ang_pos_raw.content.data;

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
    mistepper_handle->internal_temperature_top      = (float)
                    _ihal::_ii2c1::internal_top_temp_raw.content.data;

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
    mistepper_handle->internal_voltage_reference    = (float)
                _ihal::_iadc1::internal_voltage_reference.content.data;
    mistepper_handle->cpu_temperature               = (float)
                _ihal::_iadc1::cpu_temperature.content.data;
    mistepper_handle->fan_voltage                   = (float)
                _ihal::_iadc1::fan_voltage.content.data;
    mistepper_handle->fan_current                   = (float)
                _ihal::_iadc1::fan_current.content.data;
    mistepper_handle->stepper_voltage               = (float)
                _ihal::_iadc1::stepper_voltage.content.data;
    mistepper_handle->stepper_current               = (float)
                _ihal::_iadc1::stepper_current.content.data;
    mistepper_handle->conversion_fault              = (uint8_t)
                _ihal::_iadc1::conversion_flt.content;

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
    mistepper_handle->fan_demand                    = (float)
                _ihal::_ifan::demand.content;
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
    mistepper_handle->stepper_frequency            = (uint16_t)
                _ihal::_istepper::frequency.content;

    uint8_t gear = _ihal::_istepper::microstep.content  & 0x07;
    uint8_t dir  = _ihal::_istepper::direction.content  & 0x01;
    uint8_t enb  = _ihal::_istepper::enable.content     & 0x01;

    uint16_t stepper_state = enb;
    stepper_state |= (uint16_t) (dir << 1);
    stepper_state |= (uint16_t) (gear << 2);

    mistepper_handle->stepper_state                 = (uint16_t) stepper_state;

    mistepper_handle->stepper_calc_position        = (uint32_t)
                _ihal::_istepper::calc_position.content;
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

/**
  * @brief:  If the interface control has been enabled, then this function will trigger the
  *          requested tasks
  * @param:  miStepperUSART pointer
  * @retval: None (void output)
  */
void    userControl(miStepperUSART   *mistepper_handle) {
    if ( (mistepper_handle->reqt_mode & miStepperUSART::kenable_interface) != 0 ) {
        _ihal::pushValue(&_ihal::_ifan::demand,         mistepper_handle->reqt_fan_demand);

        _ihal::pushValue(&_ihal::_istepper::enable,     mistepper_handle->reqt_stepper_enable);
        _ihal::pushValue(&_ihal::_istepper::direction,  mistepper_handle->reqt_stepper_direction);
        _ihal::pushValue(&_ihal::_istepper::microstep,  mistepper_handle->reqt_stepper_gear);
        _ihal::pushValue(&_ihal::_istepper::frequency,  mistepper_handle->reqt_stepper_frequency);
    }
    else {
        _ihal::pushValue(&_ihal::_ifan::demand, 0.00f);

        _ihal::pushValue(&_ihal::_istepper::enable,     (uint8_t)   0);
        _ihal::pushValue(&_ihal::_istepper::direction,  (uint8_t)   0);
        _ihal::pushValue(&_ihal::_istepper::microstep,  (uint8_t)   0);
        _ihal::pushValue(&_ihal::_istepper::frequency,  (uint16_t)  0);

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
