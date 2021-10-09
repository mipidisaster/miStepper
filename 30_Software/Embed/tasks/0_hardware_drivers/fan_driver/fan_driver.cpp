/**************************************************************************************************
 * @file        fan_driver.cpp
 * @author      Thomas
 * @brief       Source file for Fan Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/fan_driver/fan_driver.h"
// C System Header(s)
// ------------------
#include <stdint.h>
#include <math.h>
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
#include "tim.h"                        // Include the timer header file, as this contains the
                                        // htim15 handle

#include "tasks/0_hardware_drivers/fan_driver/fan_driver_parameters.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ifan_hal.hpp"
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
  * @brief:  Fan Motor Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vFANMotorHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // No class for a PWM timer, so this section will instead setup the timer and enable it.

    // Setup and kick-off the PWM on channel 2 of timer 2
    TIM_CCxChannelCmd(htim15.Instance,  // Enable TIMER Channel 2
                      TIM_CHANNEL_2,                            // Compare Output Mode
                      TIM_CCx_ENABLE);                          //

    __HAL_TIM_MOE_ENABLE(&htim15);
    // As TIM15 (linked input) contains the break register(s), need to enable full timer
    // functionality

    __HAL_TIM_ENABLE_IT(&htim15,          // Enable TIM15 Update interrupt
                        TIM_IT_UPDATE);   // used for time schedule logging
        // Note the interrupt handling is done within main.ccp and main.h

    __HAL_TIM_ENABLE(&htim15);            // Start timer!
    //====================
    // PWM is now running!

/*---------------------------[    Device Fault flag generation    ]---------------------------*/
    // Interface signal initialisation & interrupt setup
    // =================================================
    _ihal::_ifan::interfaceInitialise();

    uint16_t tempcalc = 0;      // Temporary parameter used to convert float input to actual PWM
                                // supported width

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
            _ihal::pushValue(&_ihal::_ifan::fan_demand, 0.00f);
        }
        float current_demand = _ihal::getValue(&_ihal::_ifan::fan_demand);

        if (current_demand <= 0.00) {        // If Fan demand is 0, i.e. Switch off PWM output
            __HAL_TIM_SET_COMPARE(&htim15,
                                  TIM_CHANNEL_2,
                                  0);
        }
        else {
            if (current_demand >= 100.0)    // If demand is greater than PWM Defined width
                current_demand = 100.0;     // Limit to Defined PWM width

            tempcalc    = (uint16_t) ((current_demand / 100.0 ) * _fan::_param::kpwm_width);
            __HAL_TIM_SET_COMPARE(&htim15,
                                  TIM_CHANNEL_2,
                                  tempcalc);
        }

        // Link internal signals to output pointers:
        // This is a reading task only, no outputs are provided

        first_pass = 0;             // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        _ihal::_itimer::recordTimeSheet(  _ihal::TimedTasks::kFAN, cal_task_period,
                                          _ihal::_itimer::toc(previous_recorded_time)  );

        osDelayUntil(&previous_wake_time, _fan::_param::ktask_rate);
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
