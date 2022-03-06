/**************************************************************************************************
 * @file        spi1_dev_driver.cpp
 * @author      Thomas
 * @brief       Source file for SPI Device Driver task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/spi_dev_driver/spi1_dev_driver.h"
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
#include "spi.h"                        // Include the spi header file, as this contains the
                                        // hspi1 handle

#include "tasks/0_hardware_drivers/spi_dev_driver/spi1_dev_driver_parameters.hpp"
#include "tasks/0_hardware_drivers/spi_dev_driver/spi1_angle_sensor.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/ispi_hal.hpp"
#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_GPIO___HD               // Include the GPIO class handler
#include FilInd_SPIPe__HD               // Include the SPI class handler
#include FilInd_AS5x4x_HD               // Include the device AS5x4 handler

//=================================================================================================

namespace _spi1_dev {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static SPIPeriph    *lc_handle;         // Pointer to the SPI1 class handle, for interrupt use

static SPIPeriph::Form  form[_param::kspi_form_size]    = { 0 };

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None
}
//=================================================================================================

/**
  * @brief:  SPI1 Device Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vSPI1DeviceHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // Create SPI1 class
    // =================
    SPIPeriph   spi1_device(&hspi1, &_spi1_dev::form[0], _spi1_dev::_param::kspi_form_size);
    _spi1_dev::lc_handle = &spi1_device;    // Link SPI1Dev class to global pointer (for ISR)

    // Setup SPI Connected Devices
    // ===========================
    AS5x4x angle_sensor = _spi1_dev::_ext_angle::generateAngleSensor();

    // Link AS5047D device to Daisy chain structure for simpler interrupt handling.
    AS5x4x::Daisy   as5_daisy = AS5x4x::constructDaisy(&angle_sensor, 1);

    // Create SPI Device(s) Chip Select
    // ================================
    GPIO    angle_chipselect(CS_ANG_SENSOR_GPIO_Port,  CS_ANG_SENSOR_Pin,  GPIO::kOutput);

/*---------------------------[    Device Fault flag generation    ]---------------------------*/
    SPIPeriph::DevFlt   lc_spi1_comm_flt = {SPIPeriph::DevFlt::kInitialised};

    _ihal::DevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt> lc_angle_sensor_comm_state = {
            .IdleCount  = 0,

            .DevFlt     = AS5x4x::DevFlt::kInitialised,
            .ComFlt     = SPIPeriph::DevFlt::kInitialised
    };

    // Interface signal initialisation & interrupt setup
    // =================================================
    _ihal::_ispi1::interfaceInitialise();

    // Enable SPI interrupts:
    spi1_device.configBusErroIT(SPIPeriph::InterState::kIT_Enable);
        // Enable Bus Error fault interrupt

/*---------------------------[        SPI Device Main Loop        ]---------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t first_pass = 1;     // Indicate that this is the first time going through this task

    uint16_t previous_recorded_time = _ihal::_itimer::tic();
    uint32_t previous_wake_time = osKernelSysTick();    // Capture start time of task within Kernel
                                                        // time
    /* Infinite loop */
    for(;;) {
        uint16_t cal_task_period = _ihal::_itimer::toc(&previous_recorded_time);

        // Check the status of the AS5047D communication
        // Will handle the connection to the HAL layer for the read parameter
        _spi1_dev::_ext_angle::manageAngleSensor(
                                    &spi1_device,
                                    &angle_sensor,
                                    &as5_daisy,
                                    &angle_chipselect,
                                    &lc_angle_sensor_comm_state,
                                    &first_pass
                                                  );

        // As this is only 1 device connected to the SPI, its fault indication for the
        // communication bus needs to be copied into the "lc_spi1_comm_flt"
        lc_spi1_comm_flt = lc_angle_sensor_comm_state.ComFlt;

        // Link internal signals to output pointers:
        _ihal::pushValue(&_ihal::_ispi1::comm_flt,
                         lc_spi1_comm_flt);
        _ihal::pushValue(&_ihal::_ispi1::angle_sensor_flt,
                         lc_angle_sensor_comm_state);

        first_pass = 0;             // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        _ihal::_itimer::recordTimeSheet(  _ihal::TimedTasks::kSPI1, cal_task_period,
                                          _ihal::_itimer::toc(previous_recorded_time)  );

        osDelayUntil(&previous_wake_time, _spi1_dev::_param::ktask_rate);
        // Put the task in "DELAYED" state for the defined period
    }
}

/**
  * @brief:  SPI1 Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void SPI1_IRQHandler(void) { _spi1_dev::lc_handle->handleIRQ(); };

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None
