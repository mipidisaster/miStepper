/**************************************************************************************************
 * @file        i2c1_dev_driver.cpp
 * @author      Thomas
 * @brief       Source file for I2C Device Driver task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/i2c_dev_driver/i2c1_dev_driver.h"
// C System Header(s)
// ------------------
#include <stdint.h>
#include "cmsis_os.h"                   // Header for introducing FreeRTOS to device

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
// None

// Project Libraries
// -----------------
#include "stm32l4xx_hal.h"              // Include the HAL library
#include "main.h"                       // Include main header file, as this contains the defines
                                        // for GPIO signals
#include "i2c.h"                        // Include the i2c header file, as this contains the
                                        // hi2dc1 handle

#include "tasks/0_hardware_drivers/i2c_dev_driver/i2c1_dev_driver_parameters.hpp"
#include "tasks/0_hardware_drivers/i2c_dev_driver/i2c1_board_temperature.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ii2c_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_I2CPe__HD               // Include the I2C class handler
#include FilInd_AD741x_HD               // Include the device AD741x handler

//=================================================================================================

namespace _i2c1_dev {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static I2CPeriph    *lc_handle;         // Pointer to the I2C1 class handle, for interrupt use
static uint8_t      comm_buff[3][_param::kbuff_size]    = { 0 };

static I2CPeriph::Form  form[_param::kform_size]        = { 0 };

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None
}
//=================================================================================================

/**
  * @brief:  I2C1 Device Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vI2C1DeviceHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // Create I2C1 class
    // =================
    I2CPeriph   i2c1_device(&hi2c1, &_i2c1_dev::form[0], _i2c1_dev::_param::kform_size);
    _i2c1_dev::lc_handle = &i2c1_device;    // Link I2C1Dev class to global pointer (for ISR)

    // Setup I2C Connected Devices
    // ===========================
    AD741x  top_temperature_sensor =
                                _i2c1_dev::_int_temp::generateInternalTopTemperatureSensor();

    AD741x  bottom_temperature_sensor =
                                _i2c1_dev::_int_temp::generateInternalBottomTemperatureSensor();

/*---------------------------[    Device Fault flag generation    ]---------------------------*/
    I2CPeriph::DevFlt   lc_i2c1_comm_flt = {I2CPeriph::DevFlt::kInitialised};

    _ihal::DevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> lc_top_temp_sensor_comm_state = {
            .IdleCount  = 0,

            .DevFlt     = AD741x::DevFlt::kInitialised,
            .ComFlt     = I2CPeriph::DevFlt::kInitialised
    };

    _ihal::DevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> lc_bottom_temp_sensor_comm_state = {
            .IdleCount  = 0,

            .DevFlt     = AD741x::DevFlt::kInitialised,
            .ComFlt     = I2CPeriph::DevFlt::kInitialised
    };

    // Interface signal initialisation & interrupt setup
    // =================================================
    _ihal::_ii2c1::interfaceInitialise();

    // Enable I2C interrupts:
    i2c1_device.configBusNACKIT(I2CPeriph::InterState::kIT_Enable);
    i2c1_device.configBusSTOPIT(I2CPeriph::InterState::kIT_Enable);
    i2c1_device.configBusErroIT(I2CPeriph::InterState::kIT_Enable);

/*---------------------------[        I2C Device Main Loop        ]---------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t first_pass = 1;     // Indicate that this is the first time going through this task

    uint16_t previous_recorded_time = _ihal::_itimer::tic();
    uint32_t previous_wake_time = osKernelSysTick();    // Capture start time of task within Kernel
                                                        // time
    /* Infinite loop */
    for(;;) {
        uint16_t cal_task_period = _ihal::_itimer::toc(&previous_recorded_time);

        // Setup data communication with the top internal temperature sensor & read back data
        _i2c1_dev::_int_temp::internalTemperatureSensorManagement(
                           &i2c1_device,
                           &top_temperature_sensor,
                           &_i2c1_dev::comm_buff[_i2c1_dev::_param::kwrte_loc][0],
                           &_i2c1_dev::comm_buff[_i2c1_dev::_param::kread_top_temp_sensor][0],
                           &lc_top_temp_sensor_comm_state
                                                                 );
        if ((lc_top_temp_sensor_comm_state.DevFlt  != AD741x::DevFlt::kNone) ||
            (first_pass == 1)) {
            // If the AD7415 device/communication is faulty, OR it has been the first pass through
            // of task then
            _ihal::setFault(&_ihal::_ii2c1::internal_top_temp_raw);
        }
        else
        {   // Otherwise data is good, and this is not the first pass
            _ihal::pushValue(&_ihal::_ii2c1::internal_top_temp_raw,
                             top_temperature_sensor.temp);

            _ihal::clearFault(&_ihal::_ii2c1::internal_top_temp_raw);
        }

        // Setup data communication with the bottom internal temperature sensor & read back data
        _i2c1_dev::_int_temp::internalTemperatureSensorManagement(
                           &i2c1_device,
                           &bottom_temperature_sensor,
                           &_i2c1_dev::comm_buff[_i2c1_dev::_param::kwrte_loc][0],
                           &_i2c1_dev::comm_buff[_i2c1_dev::_param::kread_bottom_temp_sensor][0],
                           &lc_bottom_temp_sensor_comm_state
                                                                 );
        if ((lc_bottom_temp_sensor_comm_state.DevFlt  != AD741x::DevFlt::kNone) ||
            (first_pass == 1)) {
            // If the AD7415 device/communication is faulty, OR it has been the first pass through
            // of task then
            _ihal::setFault(&_ihal::_ii2c1::internal_bottom_temp_raw);
        }
        else
        {   // Otherwise data is good, and this is not the first pass
            _ihal::pushValue(&_ihal::_ii2c1::internal_bottom_temp_raw,
                             bottom_temperature_sensor.temp);

            _ihal::clearFault(&_ihal::_ii2c1::internal_bottom_temp_raw);
        }

        // Link internal signals to output pointers:
        _ihal::pushValue(&_ihal::_ii2c1::comm_flt, lc_i2c1_comm_flt);
        _ihal::pushValue(&_ihal::_ii2c1::top_temp_sensor_flt, lc_top_temp_sensor_comm_state);
        _ihal::pushValue(&_ihal::_ii2c1::bottom_temp_sensor_flt, lc_bottom_temp_sensor_comm_state);

        first_pass = 0;             // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        _ihal::_itimer::recordTimeSheet(  _ihal::TimedTasks::kI2C1, cal_task_period,
                                          _ihal::_itimer::toc(previous_recorded_time)  );

        osDelayUntil(&previous_wake_time, _i2c1_dev::_param::ktask_rate);
        // Put the task in "DELAYED" state for the defined period
    }
}

/**
  * @brief:  I2C1 Event Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void I2C1_EV_IRQHandler(void) { _i2c1_dev::lc_handle->handleEventIRQ(); }

/**
  * @brief:  I2C1 Error Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void I2C1_ER_IRQHandler(void) { _i2c1_dev::lc_handle->handleErrorIRQ(); }

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None
