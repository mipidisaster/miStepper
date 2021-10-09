/**************************************************************************************************
 * @file        adc1_dev_driver.cpp
 * @author      Thomas
 * @brief       Source file for ADC Device Driver task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/analog_dev_driver/adc1_dev_driver.h"
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
#include "adc.h"                        // Include the adc header file, as this contains the
                                        // hadc1 handle
#include "tim.h"                        // Include the timer header file, as this contains the
                                        // htim6 handle

#include "tasks/0_hardware_drivers/analog_dev_driver/adc1_dev_driver_parameters.hpp"
#include "tasks/0_hardware_drivers/analog_dev_driver/adc1_monitors.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ianalog_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class

//=================================================================================================

extern DMA_HandleTypeDef hdma_adc1;     // Declare a DMA handle used for the ADC1

namespace _adc1_dev {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static uint8_t sequence_counter    = 0;    // Counter for number of ADC runs completed

static _monitors::ADCDMARecord  cur_record  = { 0 };
    // Global 'ADCDMARecord' linked to DMA and used within interrupts, to restart DMA if fault
    // occurs

static GenBuffer<_monitors::ADCDMARecord>  *buffer_handle;
    // Pointer to the GenBuff ADCDMARecord buffer, for interrupt use
static _monitors::ADCDMARecord  form[_param::kform_size]     = { 0 };

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void calibrateDevice(void);
void configureADCDMA(void);
}
//=================================================================================================

/**
  * @brief:  ADC1 Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vADC1DeviceHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // None

/*---------------------------[     Configure the Internal ADC     ]---------------------------*/
    GenBuffer<_adc1_dev::_monitors::ADCDMARecord> adc1_device(&_adc1_dev::form[0],
                                                               _adc1_dev::_param::kform_size);
        // Link ADC Form to GenBuffer

    _adc1_dev::buffer_handle = &adc1_device;    // Link adc1_device to global pointer

    _adc1_dev::calibrateDevice();       // Calibrate the ADC1 device
    _adc1_dev::configureADCDMA();       // Configure the ADC1 with the required DMA

/*---------------------------[    Device Fault flag generation    ]---------------------------*/
    _adc1_dev::_monitors::ADCDMARecord    temp_rec = {
            .Analog = { 0 },
            .Flt    = _adc1_dev::_param::DevFlt::kInitialised,
    };

    // Interface signal initialisation & interrupt setup
    // =================================================
    _ihal::_iadc1::interfaceInitialise();

/*---------------------------[        ADC Device Main Loop        ]---------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t first_pass = 1;     // Indicate that this is the first time going through this task

    uint16_t previous_recorded_time = _ihal::_itimer::tic();
    uint32_t previous_wake_time = osKernelSysTick();    // Capture start time of task within Kernel
                                                        // time
    /* Infinite loop */
    for(;;) {
        uint16_t cal_task_period = _ihal::_itimer::toc(&previous_recorded_time);

        while (adc1_device.state() != kGenBuffer_Empty) {   // Cycle through ADC records which are
                                                            // populated
            adc1_device.outputRead( &temp_rec );            // Read new record, and copy into
                                                            // 'TempRec'

            if ((temp_rec.Flt != _adc1_dev::_param::DevFlt::kNone)||
                (first_pass == 1)) {
                // If there was a conversion problem, OR it has been the first pass through of task
                // then
                _ihal::_iadc1::setADCFaults();  // Set the fault flag for all ADC parameters
            }
            else
            {   // Otherwise data is good, and this is not the first pass
                // Below functions will write data to the HAL boundary
                _adc1_dev::_monitors::internalVoltage(&temp_rec);
                _adc1_dev::_monitors::internalTemperature(&temp_rec);

                _adc1_dev::_monitors::fanMotorVoltage(&temp_rec);
                _adc1_dev::_monitors::fanMotorCurrent(&temp_rec);

                _adc1_dev::_monitors::stepperMotorVoltage(&temp_rec);
                _adc1_dev::_monitors::stepperMotorCurrent(&temp_rec);
                _adc1_dev::_monitors::stepperMotorICLimit(&temp_rec);

                _ihal::_iadc1::clearADCFaults();    // Clear any faults
            }
        }

        // Link internal signals to output pointers:
        _ihal::pushValue(&_ihal::_iadc1::conversion_flt, temp_rec.Flt);

        first_pass = 0;             // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        _ihal::_itimer::recordTimeSheet(  _ihal::TimedTasks::kADC1, cal_task_period,
                                          _ihal::_itimer::toc(previous_recorded_time)  );

        osDelayUntil(&previous_wake_time, _adc1_dev::_param::ktask_rate);
        // Put the task in "DELAYED" state for the defined period
    }
}

/**
  * @brief:  ADC1 Interrupt Service Routine handler
  * @param:  None (void input)
  * @retval: None (void output)
  */
void ADC1_IRQHandler(void) {
#warning "Modify this, as currently its not really doing anything - within DMA_START it locks the HAL!"
    // Check to see if the interrupt was due to completed sequence:
    if ( (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOS) != 0) &&
         (__HAL_ADC_GET_IT_SOURCE(&hadc1, ADC_IT_EOS) != 0) ) {
        // If complete sequence interrupt triggered, then clear:
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOS);  // Clear End of Sequence (EOS) flag

        _adc1_dev::sequence_counter++;          // Increment the ADC sequence count.
        if (_adc1_dev::sequence_counter >= _adc1_dev::_monitors::ksequences) {
                // If sequence count is equal/greater than number of runs in record.
            _adc1_dev::sequence_counter = 0;    // Reset counter

            _adc1_dev::buffer_handle->inputWrite(_adc1_dev::cur_record);
                    // Add the current record into buffer

            if (_adc1_dev::cur_record.Flt != _adc1_dev::_param::DevFlt::kNone) {
                // If record is faulty, then DMA has been disabled. Therefore needs to be re-setup
                // and enabled
                __HAL_UNLOCK(&hdma_adc1);
                HAL_DMA_Start(
                      &hdma_adc1,                       // Linked DMA for ADC
                      (uint32_t)&hadc1.Instance->DR,    // ADC Data Register
                      (uint32_t)&_adc1_dev::cur_record.Analog[0],       // Pointer to global
                                                                        // "Current" ADC Record
                                                                        // which always contains
                                                                        // the live value(s)
                      _adc1_dev::_monitors::kconversions * _adc1_dev::_monitors::ksequences);
                            // Contains ADC1_ConvPSeq number of conversions per sequence
                            //          ADC1_NumSeq sequences in record
                // Called function will also enable the DMA peripheral
            }

            _adc1_dev::cur_record.Flt   = _adc1_dev::_param::DevFlt::kNone;
            // Clear any faults on current record
            // DMA is *expected to be* in circular mode, so will continue to run
        }
    }

    // Check to see if the interrupt was due to overrun:
    if ( (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR) != 0) &&
         (__HAL_ADC_GET_IT_SOURCE(&hadc1, ADC_IT_OVR) != 0) ) {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);  // Clear Overrun (OVR) flag

        __HAL_DMA_DISABLE(&hdma_adc1);                // Disable the linked DMA to ADC
        ADC_Disable(&hadc1);                          // And disable the ADC
        _adc1_dev::cur_record.Flt   = _adc1_dev::_param::DevFlt::kOver_Run;
            // Set record as faulty
    }
}

/**
  * @brief:  DMA1 Channel 1 (linked to ADC1) Interrupt Service Routine handler
  * @param:  None (void input)
  * @retval: None (void output)
  */
void DMA1_Channel1_IRQHandler(void) {
    // Check to see if the interrupt was due to DMA transmit error
    //      See if Interrupt has been enabled - in combination with - Interrupt being set
    if ( (__HAL_DMA_GET_IT_SOURCE(&hdma_adc1, DMA_IT_TE) != 0) &&
         (__HAL_DMA_GET_FLAG(&hdma_adc1,
                                 __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_adc1)) != 0) ) {
        __HAL_DMA_CLEAR_FLAG(&hdma_adc1, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_adc1));
            // Clear the interrupt (Channel 1)

        __HAL_DMA_DISABLE(&hdma_adc1);                      // Disable the linked DMA to ADC
        ADC_Disable(&hadc1);                                // And disable the ADC
        _adc1_dev::cur_record.Flt   = _adc1_dev::_param::DevFlt::kDMA_Error;
            // Set record as faulty

        // Essentially if there is a DMA error fault, then cancel all ADC conversions!
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
  * @brief:  ADC1 calibration setup
  * @param:  void const, not used
  * @retval: None (void output)
  */
void _adc1_dev::calibrateDevice(void) {
    uint32_t calibration = 0;           // Variable for the calibrated return value

    // Start calibration run for Single Ended signals (only these are used within miStepper)
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Retrieve the read calibration value
    calibration = HAL_ADCEx_Calibration_GetValue(&hadc1,
                                                 ADC_SINGLE_ENDED);

    // Now put into Calibration register of ADC (Single Ended)
    HAL_ADCEx_Calibration_SetValue(&hadc1,
                                   ADC_SINGLE_ENDED, calibration);
}

/**
  * @brief:  ADC1 DMA setup
  * @param:  void const, not used
  * @retval: None (void output)
  */
void _adc1_dev::configureADCDMA(void) {
    // Link the DMA ADC channel to ADC 1 Device, so as to capture all the conversions triggered
    __HAL_DMA_DISABLE(&hdma_adc1);    // Ensure that the DMA channel is disabled
    HAL_DMA_Start(&hdma_adc1,                         // Linked DMA for ADC
                  (uint32_t)&hadc1.Instance->DR,   // ADC Data Register
                  (uint32_t)&_adc1_dev::cur_record.Analog[0],   // Pointer to global "Current" ADC
                                                                // Record which always contains the
                                                                // live value(s)
                  _adc1_dev::_monitors::kconversions * _adc1_dev::_monitors::ksequences);
                    // Contains ADC1_ConvPSeq number of conversions per sequence
                    //          ADC1_NumSeq sequences in record
    // Called function will also enable the DMA peripheral

    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMAEN);              // Link ADC1 to the DMA
    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMACFG);             // Enable circular conversions

    // As the DMA is now linked to the ADC. It means that any overruns that occur will result in
    // the ADC hardware no longer requesting any DMA requests.
    // Therefore OVR interrupt needs to be enabled, as well as conversion interrupt
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_OVR);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOS);

    // Enable the DMA Error interrupt flag
    __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TE);

    // The strategy for fault accommodation is as follows:
    //  A OVRRun fault, or DMA fault
    //      DMA is disabled, and any ADC conversions are ignored (data to be considered faulty)
    //      Once the correct number of ADC conversions have completed then:
    //          Any existing ADC faults will be cleared
    //          DMA will be restarted, and enabled

    ADC_Enable(&hadc1);   // Enable the ADC

    // Enable Timer for ADC conversions
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);  // Ensure that update flag is already cleared
    __HAL_TIM_ENABLE(&htim6);       // Then enable timer

    LL_ADC_REG_StartConversion(hadc1.Instance);    // Start conversions!
}
