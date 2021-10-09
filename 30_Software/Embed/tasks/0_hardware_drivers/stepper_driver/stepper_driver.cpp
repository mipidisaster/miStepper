/**************************************************************************************************
 * @file        stepper_driver.cpp
 * @author      Thomas
 * @brief       Source file for Stepper Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/stepper_driver/stepper_driver.h"
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
#include "tim.h"                        // Include the timer header file, as this contains the
                                        // htim1 handle

#include "tasks/0_hardware_drivers/stepper_driver/stepper_driver_parameters.hpp"

#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_GPIO___HD               // Include the GPIO class handler
#include FilInd_StpCOREHD               // Include the Stepper CORE class handler
#include FilInd_StpDMA_HD               // Include the Stepper DMA class handler

//=================================================================================================

extern DMA_HandleTypeDef hdma_tim1_ch3; // Declare a DMA handle used for the TIM1

namespace _stepper {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static StepperDMA   *lc_handle;         // Pointer to the StepperDMA class handle, for interrupt
                                        // use

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None
}
//=================================================================================================

/**
  * @brief:  Stepper Motor Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vSTPMotorHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // Generate the GPIO pins outside of 'Stepper' handle scope
    // ========================================================
    GPIO    STP_DIR(    STP_DIR_GPIO_Port,          STP_DIR_Pin,        GPIO::kOutput);
    GPIO    STP_MS[3]={{STP_MS1_GPIO_Port,          STP_MS1_Pin,        GPIO::kOutput},
                       {STP_MS2_GPIO_Port,          STP_MS2_Pin,        GPIO::kOutput},
                       {STP_MS3_GPIO_Port,          STP_MS3_Pin,        GPIO::kOutput} };

    GPIO    STP_DECAY(  STP_DECAY_GPIO_Port,        STP_DECAY_Pin,      GPIO::kOutput);
    GPIO    STP_SLEEP(  STP_NSLEEP_GPIO_Port,       STP_NSLEEP_Pin,     GPIO::kOutput);

    // Disable the Stepper IC, by pulling ENABLE/SLEEP high, and RESET low:
    STP_SLEEP.setValue(GPIO::State::kHigh);     // Set Sleep  pin to LOW  '0'   (Device Asleep)
    STP_DECAY.setValue(GPIO::State::kHigh);     // Set Decay  pin to HIGH '1'   (Slow Decay mode)

    // Create Hardware setup variable used to configure the 'Stepper' handler:
    StepperCore::HrdSetup  STPConfig;

    /* Setup the 'Stepper' handler so that
     *      [1] The linked DMA used to generate the STEP 'pulse', which is Capture/Compare 3
     *      [2] Provide the BIT position for enabling/disabling in 'DIER' register for TIM linkage
     *          to DMA
     *      [3] Provide the BIT position within TIM 'EGR' to generate a 'Capture/Compare 3' event
     *      [4] Provide the BIT position within TIM 'SR' to check for a 'Capture/Compare 3' event
     *********************************************************************************************/
    STPConfig.PulseDMAAddress  = (uint32_t) &htim1.Instance->CCR3;
                                                                                            //[1]
    STPConfig.EnbTIMDMA        = TIM_DMA_CC3;                                               //[2]
    STPConfig.PulsEGRBit       = TIM_EVENTSOURCE_CC3;                                       //[3]
    STPConfig.PulsSRBit        = TIM_FLAG_CC3;                                              //[4]

    // Create Stepper(DMA) class
    // =========================
    StepperDMA NEMA(&htim1, &hdma_tim1_ch3,
                // Generate the output signal for STEP 'pulse'
            ( TIM_CCxN_ENABLE << TIM_CHANNEL_3 ),   // The output channel pin for enabling the
                                                    // output for 'Capture/Compare 3'
            // Linked GPIO pins - Reset(Sleep), Direction and Microstep
            &STP_SLEEP, &STP_DIR, &STP_MS[0], 3,
            200 * 16, STPConfig
            );
    _stepper::lc_handle = &NEMA;        // Link NEMA class to global pointer (for ISR)

/*---------------------------[    Device Fault flag generation    ]---------------------------*/
        // Interface signal initialisation & interrupt setup
        // =================================================
        //_ihal::_ifan::interfaceInitialise();


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

        if (first_pass == 1) {
            NEMA.newPosition(GPIO::State::kHigh, 0, 20000, 200,
                             StepperCore::CountPanel::Polarity::kUp, 5);

            NEMA.newPosition(GPIO::State::kLow, 7, 200, 51200,
                             StepperCore::CountPanel::Polarity::kUp, 5);
            // Run test cycle - 200steps at 20ms per pulses
            // 1 rotation = 4s
        }

        // Link internal signals to output pointers:
        // This is a reading task only, no outputs are provided

        first_pass = 0;             // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        _ihal::_itimer::recordTimeSheet(  _ihal::TimedTasks::kSTEPPER, cal_task_period,
                                          _ihal::_itimer::toc(previous_recorded_time)  );

        osDelayUntil(&previous_wake_time, _stepper::_param::ktask_rate);
        // Put the task in "DELAYED" state for the defined period
    }
}

/**
  * @brief:  DMA1 Channel 7 (linked to TIM1 Channel 3) Interrupt Service Routine handler
  * @param:  None (void input)
  * @retval: None (void output)
  */
void DMA1_Channel7_IRQHandler(void) {  _stepper::lc_handle->handleIRQ();         }

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None
