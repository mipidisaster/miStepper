/**************************************************************************************************
 * @file        stepper_driver.cpp
 * @author      Thomas
 * @brief       Source file for Stepper Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/motor_drivers/stepper_driver/stepper_driver.hpp"
#include "tasks/0_hardware_drivers/motor_drivers/motor_driver.h"
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
#include "stm32l4xx_hal.h"              // Include the HAL library
#include "main.h"                       // Include main header file, as this contains the defines
                                        // for GPIO signals
#include "tim.h"                        // Include the timer header file, as this contains the
                                        // htim1 handle

#include "tasks/0_hardware_drivers/motor_drivers/stepper_driver/stepper_driver_parameters.hpp"

#include "tasks/1_hardware_arbitration_layer/istepper_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_GPIO___HD               // Include the GPIO class handler
#include FilInd_StpCOREHD               // Include the Stepper CORE class handler

#ifdef zz__StepperDMAControl__zz
#include FilInd_StpDMA_HD               // Include the Stepper DMA class handler

#else
#include FilInd_StpTIM_HD               // Include the Stepper TIM class handler

#endif
//=================================================================================================

extern DMA_HandleTypeDef hdma_tim1_ch3; // Declare a DMA handle used for the TIM1

namespace _motor::_stepper {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
#ifdef zz__StepperDMAControl__zz
static StepperDMA   *lc_handle;         // Pointer to the StepperDMA class handle, for interrupt
                                        // use

#else
static StepperTIM   *lc_handle;         // Pointer to the StepperTIM class handle, for interrupt
                                        // use

#endif

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
uint32_t determineCountRate(uint8_t microstep);

//=================================================================================================

/**
  * @brief:  Setup and initialise the Stepper Class
  * @param:  void const, not used
  * @retval: None (void output)
  */
void setup(void) {
    // Generate the GPIO pins outside of 'Stepper' handle scope
    // ========================================================
    static GPIO    STP_DIR(    STP_DIR_GPIO_Port,       STP_DIR_Pin,        GPIO::kOutput);
    static GPIO    STP_MS[3]={{STP_MS1_GPIO_Port,       STP_MS1_Pin,        GPIO::kOutput},
                              {STP_MS2_GPIO_Port,       STP_MS2_Pin,        GPIO::kOutput},
                              {STP_MS3_GPIO_Port,       STP_MS3_Pin,        GPIO::kOutput} };

    static GPIO    STP_DECAY(  STP_DECAY_GPIO_Port,     STP_DECAY_Pin,      GPIO::kOutput);
    static GPIO    STP_SLEEP(  STP_NSLEEP_GPIO_Port,    STP_NSLEEP_Pin,     GPIO::kOutput);


    // Disable the Stepper IC, by pulling ENABLE/SLEEP high, and RESET low:
    STP_SLEEP.setValue(GPIO::State::kLow);      // Set Sleep  pin to LOW  '0'   (Device Asleep)
    STP_DECAY.setValue(GPIO::State::kLow);      // Set Decay  pin to Low  '0'   (Mixed Decay mode)
    // Create Stepper(DMA) class
    // =========================
#ifdef zz__StepperDMAControl__zz
    static StepperDMA
#else
    static StepperTIM
#endif
            STSPIN820(&htim1, &hdma_tim1_ch3,
                // Generate the output signal for STEP 'pulse'
            ( TIM_CCxN_ENABLE << TIM_CHANNEL_3 ),   // The output channel pin for enabling the
                                                    // output for 'Capture/Compare 3'
            // Linked GPIO pins - Reset(Sleep), Direction and Microstep
            &STP_SLEEP, &STP_DIR, &STP_MS[0], 3,
            200 * 256,

   /* Setup the 'Stepper' handler so that
    *      [1] The linked DMA used to generate the STEP 'pulse', which is Capture/Compare 3
    *      [2] Provide the BIT position for enabling/disabling in 'DIER' register for TIM linkage
    *          to DMA
    *      [3] Provide the BIT position within TIM 'EGR' to generate a 'Capture/Compare 3' event
    *      [4] Provide the BIT position within TIM 'SR' to check for a 'Capture/Compare 3' event
    *********************************************************************************************/
            StepperCore::HrdSetup {.PulseDMAAddress  = (uint32_t) &htim1.Instance->CCR3,    //[1]
                                   .EnbTIMDMA        = TIM_DMA_CC3,                         //[2]
                                   .PulsEGRBit       = TIM_EVENTSOURCE_CC3,                 //[3]
                                   .PulsSRBit        = TIM_FLAG_CC3                         //[4]
                                  }
            );

    lc_handle = &STSPIN820;         // Link STSPIN820 class to global pointer (for ISR)

}

/**
  * @brief:  Tasks to do only ONCE in the loop
  * @param:  void const, not used
  * @retval: None (void output)
  */
void firstpass(void) {
   // Nothing required currently for initial pass.
}

/**
  * @brief:  Tasks to update the speed demand of the stepper motor
  * @param:  void const, not used
  * @retval: None (void output)
  */
void updatespeed(uint8_t enable, uint8_t microstep, uint8_t direction, uint16_t frequency) {
    if (enable != 0) {
        if ( (frequency == 0) && (lc_handle->getCurrentMode() != StepperCore::Mode::kDisabled) ) {
            lc_handle->forceSTOP();
        }
        else {
        GPIO::State motor_direction = GPIO::State::kLow;
        StepperCore::CountPanel::Polarity motor_polarity
                               = StepperCore::CountPanel::Polarity::kDown;

        if (direction == 0) {
            motor_direction = GPIO::State::kHigh;
            motor_polarity  = StepperCore::CountPanel::Polarity::kUp;
        }

        lc_handle->newVelocity(motor_direction,
                               microstep,
                               frequency,
                               motor_polarity, determineCountRate(microstep));
    }}
    else {
        // If the motor is in motion, then force a stop
        if (lc_handle->getCurrentMode() != StepperCore::Mode::kDisabled) {
            lc_handle->forceSTOP();
        }

        lc_handle->directAccessReset(0);    // For the Reset of the Stepper.
        // With the STSPIN820 this will result in the motor becoming limp (not being actively
        // driven)
    }
}

void calposition(void) {
    _ihal::pushValue(&_ihal::_istepper::calc_position, lc_handle->calc_position);
}

}

#ifdef zz__StepperDMAControl__zz
/**
  * @brief:  DMA1 Channel 7 (linked to TIM1 Channel 3) Interrupt Service Routine handler
  * @param:  None (void input)
  * @retval: None (void output)
  */
void DMA1_Channel7_IRQHandler(void) { _motor::_stepper::lc_handle->handleIRQ();         }
#else
/**
  * @brief:  TIM1 Update Interrupt Service Routine handler (does include TIM16 update as well)
  * @param:  None (void input)
  * @retval: None (void output)
  */
void TIM1_UP_TIM16_IRQHandler(void) { _motor::_stepper::lc_handle->handleIRQ();         }

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
namespace _motor::_stepper {
/**
  * @brief:  Function to take in desired microstep value, and return the rate of count for the
  *          StepperCore calculated position
  *          See datasheet for STSPIN820 for how the below is defined
  *
  * @param:  unsigned 8bit integer defining the microstep value
  * @retval: unsigned 32bit integer value for step rate (absolute)
  */
uint32_t determineCountRate(uint8_t microstep) {
    switch (microstep) {
    case 0:
        return 256;

    case 1:
        return 128;

    case 2:
        return 64;

    case 3:
        return 32;

    case 4:
        return 16;

    case 5:
        return 8;

    case 6:
        return 2;

    case 7:
        return 1;

    default:        // Default returns '0' such that the position calculation is erroneous
        return 0;   // due to an unsupported microstep being selected.
    }

    // Shouldn't get here, but this is to protect against this
    return 0;
}
}
