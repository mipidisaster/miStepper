/**************************************************************************************************
 * @file        fan_driver.cpp
 * @author      Thomas
 * @brief       Source file for Fan Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/motor_drivers/fan_driver/fan_driver.hpp"
// C System Header(s)
// ------------------
#include <stdint.h>
#include <math.h>

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

#include "tasks/0_hardware_drivers/motor_drivers/fan_driver/fan_driver_parameters.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ifan_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

//=================================================================================================

namespace _motor::_fan {
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
  * @brief:  Setup and initialise the FAN Timer PWM
  * @param:  void const, not used
  * @retval: None (void output)
  */
void setup(void) {
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
}

/**
  * @brief:  Tasks to do only ONCE in the loop
  * @param:  void const, not used
  * @retval: None (void output)
  */
void firstpass(void) {
    _ihal::pushValue(&_ihal::_ifan::demand, 0.00f);
}

/**
  * @brief:  Update the FAN PWN parameter as per the input 'new_speed'
  * @param:  float value, containing the new fan speed
  * @retval: None (void output)
  */
void updatespeed(float new_speed) {
    if (new_speed <= 0.00) {        // If Fan demand is 0, i.e. Switch off PWM output
        __HAL_TIM_SET_COMPARE(&htim15,
                              TIM_CHANNEL_2,
                              0);
    }
    else {
        if (new_speed >= 100.0)    // If demand is greater than PWM Defined width
            new_speed = 100.0;     // Limit to Defined PWM width

        uint16_t tempcalc   = (uint16_t) ((new_speed / 100.0 ) * _param::kpwm_width);
        __HAL_TIM_SET_COMPARE(&htim15,
                              TIM_CHANNEL_2,
                              tempcalc);
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
