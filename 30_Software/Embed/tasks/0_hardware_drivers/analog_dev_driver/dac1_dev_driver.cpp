/**************************************************************************************************
 * @file        dac1_dev_driver.cpp
 * @author      Thomas
 * @brief       Source file for DAC Device Driver task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/analog_dev_driver/dac1_dev_driver.h"
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
#include "dac.h"                        // Include the dac header file, as this contains the
                                        // hdac1 handle
#include "tim.h"                        // Include the timer header file, as this contains the
                                        // htim6 handle

#include "tasks/0_hardware_drivers/analog_dev_driver/dac1_dev_driver_parameters.hpp"
#include "tasks/0_hardware_drivers/analog_dev_driver/dac1_stepper_vref.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ianalog_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

//=================================================================================================

extern DMA_HandleTypeDef hdma_dac_ch1;     // Declare a DMA handle used for the DAC1

namespace _dac1_dev {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static volatile uint16_t dac_dma_register[_param::dma_size] = { 0 };
static volatile _param::DevFlt  dev_flt = _param::DevFlt::kInitialised;     // Fault flag for DAC

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void configureDACDMA(void);
}
//=================================================================================================

/**
  * @brief:  DAC1 Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vDAC1DeviceHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // None

/*---------------------------[     Configure the Internal ADC     ]---------------------------*/
    _dac1_dev::configureDACDMA();               // Configure the DAC1 with the required DMA
        // And also enable the DAC peripheral

/*---------------------------[    Device Fault flag generation    ]---------------------------*/
    _dac1_dev::dev_flt   = _dac1_dev::_param::DevFlt::kNone;
        // Clear fault flags

    // Interface signal initialisation & interrupt setup
    // =================================================
    _ihal::_idac1::interfaceInitialise();

/*---------------------------[        DAC Device Main Loop        ]---------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t first_pass = 1;     // Indicate that this is the first time going through this task

    uint16_t previous_recorded_time = _ihal::_itimer::tic();
    uint32_t previous_wake_time = osKernelSysTick();    // Capture start time of task within Kernel
                                                        // time
    /* Infinite loop */
    for(;;) {
        uint16_t cal_task_period = _ihal::_itimer::toc(&previous_recorded_time);

        if (first_pass == 1) {                  // On first pass of this task, it owns its input
                                                // as well
            _ihal::pushValue(&_ihal::_idac1::stepper_ic_limit_demand, 0.25f);
        }

        float current_demand = _ihal::getValue(&_ihal::_idac1::stepper_ic_limit_demand);

        if (current_demand <= _dac1_dev::_param::low_current) { // If demand is 0, force register
            _dac1_dev::dac_dma_register[0] = 0;                 // to zero
        }
        else {
            // Ensure that device current demand is saturated to the maximum current
            if (current_demand >= _dac1_dev::_param::max_current) {
                current_demand = _dac1_dev::_param::max_current;
            }

            _dac1_dev::dac_dma_register[0] = _dac1_dev::_stepper_vref::calcStepperMotorICLimit(
                                                current_demand);
        }

        // Link internal signals to output pointers:
        _ihal::pushValue(&_ihal::_idac1::realisation_flt, _dac1_dev::dev_flt);

        first_pass = 0;             // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        _ihal::_itimer::recordTimeSheet(  _ihal::TimedTasks::kDAC1, cal_task_period,
                                          _ihal::_itimer::toc(previous_recorded_time)  );

        osDelayUntil(&previous_wake_time, _dac1_dev::_param::ktask_rate);
        // Put the task in "DELAYED" state for the defined period
    }
}

/**
  * @brief:  TIM6, DAC1 Channel 1 & 2 Interrupt Serivce Routine handler
  * @param:  None (void input)
  * @retval: None (void output)
  */
void TIM6_DAC_IRQHandler(void) {
    // Check to see if the interrupt was due to underrun:
    if ( (__HAL_DAC_GET_FLAG(&hdac1, DAC_FLAG_DMAUDR1) != 0) &&
         (__HAL_DAC_GET_IT_SOURCE(&hdac1, DAC_IT_DMAUDR1) != 0) ) {
        __HAL_DAC_CLEAR_FLAG(&hdac1, DAC_FLAG_DMAUDR1);     // Clear Underrun (AUDR) flag

        __HAL_DMA_DISABLE(&hdma_dac_ch1);                   // Disable the linked DMA to ADC
        __HAL_DAC_DISABLE(&hdac1, DAC_CHANNEL_1);           // and disable DAC1 channel 1

        _dac1_dev::dev_flt   = _dac1_dev::_param::DevFlt::kUnder_Run;
            // Set record as faulty
    }
}

/**
  * @brief:  DMA1 Channel 3 (linked to DAC1) Interrupt Service Routine handler
  * @param:  None (void input)
  * @retval: None (void output)
  */
void DMA1_Channel3_IRQHandler(void) {
    // Check to see if the interrupt was due to DMA transmit error
    //      See if Interrupt has been enabled - in combination with - Interrupt being set
    if ( (__HAL_DMA_GET_IT_SOURCE(&hdma_dac_ch1, DMA_IT_TE) != 0) &&
         (__HAL_DMA_GET_FLAG(&hdma_dac_ch1,
                                 __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_dac_ch1)) != 0) ) {
        __HAL_DMA_CLEAR_FLAG(&hdma_dac_ch1, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_dac_ch1));
            // Clear the interrupt (Channel 3)

        __HAL_DMA_DISABLE(&hdma_dac_ch1);                   // Disable the linked DMA to ADC
        __HAL_DAC_DISABLE(&hdac1, DAC_CHANNEL_1);           // and disable DAC1 channel 1
        _dac1_dev::dev_flt   = _dac1_dev::_param::DevFlt::kDMA_Error;
            // Set record as faulty

        // Essentially if there is a DMA error fault, then cancel all DAC conversions!
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @brief:  DAC1 DMA setup
  * @param:  None (void input)
  * @retval: None (void output)
  */
void _dac1_dev::configureDACDMA(void) {
    // Link the DMA DAC channel to the DAC1 device, so as to reduce workload on CPU
    __HAL_DMA_DISABLE(&hdma_dac_ch1);   // Ensure that the DMA channel is disabled

    HAL_DMA_Start(&hdma_dac_ch1,                   // Linked DMA for DAC
                  (uint32_t)&dac_dma_register[0],           // DMA register as source
                  (uint32_t)&hdac1.Instance->DHR12R1,       // 12bit right hand side (destination)

                  _dac1_dev::_param::dma_size);
    // Called function will also enable the DMA peripheral

    SET_BIT(hdac1.Instance->CR, DAC_CR_DMAEN1);                 // Enable the DMA for DAC1

    // As the DMA is now linked to the DAC. It means that any underruns that occur will result in
    // the DAC hardware no longer requesting any DMA requests.
    // Therefore underruns interrupt needs to be enabled, as well as conversion interrupt
    __HAL_DAC_ENABLE_IT(&hdac1, DAC_IT_DMAUDR1);

    // Enable the DMA Error interrupt flag
    __HAL_DMA_ENABLE_IT(&hdma_dac_ch1, DMA_IT_TE);

    __HAL_DAC_ENABLE(&hdac1, DAC_CHANNEL_1);    // Enable the DAC1 channel 1

    // Enable Timer for DAC conversions
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);  // Ensure that update flag is already cleared
    __HAL_TIM_ENABLE(&htim6);       // Then enable timer
}
