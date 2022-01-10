/**************************************************************************************************
 * @file        motor_driver.cpp
 * @author      Thomas
 * @brief       Source file for Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/motor_drivers/motor_driver.h"
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
#include "main.h"                       // Include main header file, as this contains the defines

#include "tasks/0_hardware_drivers/motor_drivers/motor_driver_parameters.hpp"

#include "tasks/0_hardware_drivers/motor_drivers/fan_driver/fan_driver.hpp"
#include "tasks/0_hardware_drivers/motor_drivers/stepper_driver/stepper_driver.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ifan_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/istepper_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

//=================================================================================================
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
  * @brief:  Motor Hardware Abstraction Layer task (covering both Fan and Stepper configuration
  *          and monitoring)
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vMotorHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    _motor::_fan::setup();
    _motor::_stepper::setup();

/*---------------------------[    Device Fault flag generation    ]---------------------------*/
    // Interface signal initialisation & interrupt setup
    // =================================================
    _ihal::_ifan::interfaceInitialise();
    _ihal::_istepper::interfaceInitialise();

    uint8_t enable      = _ihal::_istepper::enable.content;
    uint8_t microstep   = _ihal::_istepper::microstep.content;
    uint8_t direction   = _ihal::_istepper::direction.content;
    uint16_t frequency  = _ihal::_istepper::frequency.content;

/*---------------------------[        FAN Device Main Loop        ]---------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t first_pass = 1;     // Indicate that this is the first time going through this task

    uint16_t previous_recorded_time = _ihal::_itimer::tic();
    uint32_t previous_wake_time = osKernelSysTick();    // Capture start time of task within Kernel
                                                        // time
    /* Infinite loop */
    for(;;) {
        uint16_t cal_task_period = _ihal::_itimer::toc(&previous_recorded_time);

        // Capture any new updates to input signals and link to local internals
        if (first_pass == 1) {                  // On first pass of this task, it owns its input
                                                // as well
            _motor::_fan::firstpass();
            _motor::_stepper::firstpass();
        }
        float current_demand = _ihal::getValue(&_ihal::_ifan::demand);
        _motor::_fan::updatespeed(current_demand);

        if ((enable     != _ihal::_istepper::enable.content) ||
            (microstep  != _ihal::_istepper::microstep.content) ||
            (direction  != _ihal::_istepper::direction.content) ||
            (frequency  != _ihal::_istepper::frequency.content))
        {
            _motor::_stepper::updatespeed(_ihal::_istepper::enable.content,
                                          _ihal::_istepper::microstep.content,
                                          _ihal::_istepper::direction.content,
                                          _ihal::_istepper::frequency.content);
        }

        enable      = _ihal::_istepper::enable.content;
        microstep   = _ihal::_istepper::microstep.content;
        direction   = _ihal::_istepper::direction.content;
        frequency  = _ihal::_istepper::frequency.content;
        _motor::_stepper::calposition();

        first_pass = 0;             // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        _ihal::_itimer::recordTimeSheet(  _ihal::TimedTasks::kFAN, cal_task_period,
                                          _ihal::_itimer::toc(previous_recorded_time)  );

        osDelayUntil(&previous_wake_time, _motor::_param::ktask_rate);
        // Put the task in "DELAYED" state for the defined period
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
