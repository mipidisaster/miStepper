/**************************************************************************************************
 * @file        usart1_dev_driver.cpp
 * @author      Thomas
 * @brief       Source file for USART communication task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include "tasks/0_hardware_drivers/usart_dev_driver/usart1_dev_driver.h"
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
#include "usart.h"                      // Include the usart header file, as this contains the
                                        // huart1 handle

#include "tasks/0_hardware_drivers/usart_dev_driver/usart1_dev_driver_parameters.hpp"
#include "tasks/0_hardware_drivers/usart_dev_driver/usart1_interfaces.hpp"

#include "tasks/1_hardware_arbitration_layer/ihal_management.hpp"

#include "tasks/1_hardware_arbitration_layer/itimer_hal.hpp"

#include "mistepper_driver/miStepperUSART.h"    // Header for miStepper UART interface

#include "FileIndex.h"
//~~~~~~~~~~~~~~~~~~~~
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class

#include FilInd_USART__HD               // Include the USART Class handler
#include FilIndUSARTDMAHD               // Include the USART DMA specific class

#include FilInd_DATMngrHD               // Provide the function set for Data Manipulation

//=================================================================================================

extern DMA_HandleTypeDef hdma_usart1_rx;    // Declare a DMA handle used for the USART1 receive
extern DMA_HandleTypeDef hdma_usart1_tx;    // Declare a DMA handle used for the USART1 transmit

namespace _usart1_dev {
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static UARTDMAPeriph *lc_handle;        // Pointer to the USART1 class handle, for interrupt use
static uint8_t      comm_buff[2][_param::kbuff_size]    = { 0 };

static UARTPeriph::Form  form[2][_param::kform_size]    = { 0 };
//#error "The above has been modified this needs to be checked at next run"
/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None
}
//=================================================================================================

/**
  * @brief:  USART1 Device Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vUSART1DeviceHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // Create USART1 class
    // ===================
    UARTDMAPeriph   usart1_device(&huart1,
        &_usart1_dev::form[_usart1_dev::_param::kwrte_loc][0], _usart1_dev::_param::kform_size,
        &_usart1_dev::form[_usart1_dev::_param::kread_loc][0], _usart1_dev::_param::kform_size,
        &hdma_usart1_rx, &hdma_usart1_tx);
    _usart1_dev::lc_handle = &usart1_device;    // Link USART1Dev class to global pointer (for ISR)

    // Setup Communication buffer array
    // ================================
    /*  This buffer array is to be used to contain the data transmitted via USART to and from this
     *  device.
     *  The first  entry is to be the data being SENT from this embedded device
     *  The second entry is to be the data being READ from this embedded device
     */
    miStepperUSART usart_protocol(
            miStepperUSART::DeviceSource::kmiStepper,

            &_usart1_dev::comm_buff[_usart1_dev::_param::kwrte_loc][0],
                                        _usart1_dev::_param::kbuff_size,
            &_usart1_dev::comm_buff[_usart1_dev::_param::kread_loc][0],
                                        _usart1_dev::_param::kbuff_size
            );

/*---------------------------[    Device Fault flag generation    ]---------------------------*/
    volatile UARTPeriph::DevFlt read_back_fault = UARTPeriph::DevFlt::kNone;
    volatile uint16_t count_read_back = 0;

    volatile UARTPeriph::DevFlt wrte_back_fault = UARTPeriph::DevFlt::kNone;
    volatile uint16_t count_wrte_back = 0;

    // Interface signal initialisation & interrupt setup
    // =================================================
    //_ihal::_ispi1::interfaceInitialise();

/*---------------------------[       USART Device Main Loop       ]---------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t first_pass = 1;     // Indicate that this is the first time going through this task

    uint16_t previous_recorded_time = _ihal::_itimer::tic();
    uint32_t previous_wake_time = osKernelSysTick();    // Capture start time of task within Kernel
                                                        // time
    /* Infinite loop */
    for(;;) {
        uint16_t cal_task_period = _ihal::_itimer::toc(&previous_recorded_time);

        usart1_device.readGenBufferLock(&usart_protocol.message_in,
                                        &read_back_fault,
                                        &count_read_back);
        usart_protocol.decodeMessage();

        _usart1_dev::_interfaces::spi1(&usart_protocol);
        _usart1_dev::_interfaces::i2c1(&usart_protocol);
        _usart1_dev::_interfaces::adc1(&usart_protocol);
        _usart1_dev::_interfaces::fan(&usart_protocol);
        _usart1_dev::_interfaces::stepper(&usart_protocol);
        _usart1_dev::_interfaces::usart1(&usart_protocol);

        if ( (usart_protocol.reqt_mode & miStepperUSART::kreset_packetcount) != 0 ) {
            usart_protocol.packet_count = 0;

            usart_protocol.reqt_mode &= ~(miStepperUSART::kreset_packetcount);
        }

        _usart1_dev::_interfaces::userControl(&usart_protocol);

        if ( (usart_protocol.reqt_mode & miStepperUSART::kenable_transmit) != 0 ) {
            usart_protocol.miStepperOut();

            usart1_device.intWrtePacket(&usart_protocol.message_out.pa[0],
                                        usart_protocol.message_out.input_pointer,
                                        &wrte_back_fault,
                                        &count_wrte_back);

            usart_protocol.reqt_mode &= ~(miStepperUSART::kenable_transmit);
            usart_protocol.packet_count++;     // Increase packet counter
        }

        // Link internal signals to output pointers:
        // None

        first_pass = 0;             // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        _ihal::_itimer::recordTimeSheet(  _ihal::TimedTasks::kUSART1, cal_task_period,
                                          _ihal::_itimer::toc(previous_recorded_time)  );

        osDelayUntil(&previous_wake_time, _usart1_dev::_param::ktask_rate);
        // Put the task in "DELAYED" state for the defined period
    }
}

/**
  * @brief:  USART1 Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void USART1_IRQHandler(void) { _usart1_dev::lc_handle->handleIRQ(); };

/**
  * @brief:  DMA1 Channel 4 USART1 (Transmit) Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void DMA1_Channel4_IRQHandler(void) { _usart1_dev::lc_handle->handleDMATxIRQ(); };

/**
  * @brief:  DMA1 Channel 5 USART1 (Receive) Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void DMA1_Channel5_IRQHandler(void) { _usart1_dev::lc_handle->handleDMARxIRQ(); };

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

