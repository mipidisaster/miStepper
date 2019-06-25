/**************************************************************************************************
 * @file        i2c_dev_hal.cpp
 * @author      Thomas
 * @version     V1.1
 * @date        25 Jun 2019
 * @brief       Source file for I2C Device task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/

#include "EmbedIndex.h"
#include EMBD_I2CTask

/**************************************************************************************************
 * Define any local global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static I2CPeriph    *I2C1_Handle;       // Pointer to the I2C1 class handle, for interrupt only

/**************************************************************************************************
 * Define any local functions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void AD74151_IntrManage(I2CPeriph *hi2c, AD741x *device, GenBuffer<uint8_t> *wrBuff,
                                                         GenBuffer<uint8_t> *rdBuff,
                        HALDevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> *CommState);
// Prototype for AD7415_1 Packet setup and management.


/**
  * @brief:  I2C1 Device Hardware Abstraction Layer task
  * @param:  _taskI2C -> cast to a void pointer
  * @retval: None (void output)
  */
void vI2C1DeviceHAL(void const * pvParameters) {
    _taskI2C1 pxParameters;
    pxParameters = * (_taskI2C1 *) pvParameters;
    // pxParameters is to include the parameters required to configure and interface this task
    // with other tasks within the OS - see header file for parameters (config, input, output).
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // Create locally used variables within task:
    I2CPeriph::Form     I2CForm[I2C1_FormBuffer]        = { 0 };    // I2C1 Form initialised
    uint8_t             I2CCommBuff[2][I2C1_CommBuff]   = { 0 };    // I2C Communication array

    GenBuffer<uint8_t>  I2CWriteBuff(&I2CCommBuff[I2CWt_Arry][0], I2C1_CommBuff);
        // Create GenBuffer linked to generic write transmit array. This will be used by all
        // I2C attached device write operations.

    // Create I2C1 class
    // =================
    I2CPeriph   I2C1Dev(pxParameters.config.i2c1_handle, &I2CForm[0], I2C1_FormBuffer);
    I2C1_Handle = &I2C1Dev;         // Link I2C1Dev class to global pointer (for ISR)

/*---------------------------[ Setup I2C Connected Device Classes ]---------------------------*/
    AD741x::Form        AD7415Form[I2C1_AD7415Form]     = { 0 };    // AD7415 Form initialised

    AD741x  AD7415Dev(AD741x::DevPart::AD7415_1,        // Construct a AD7415_1 class handle
                      AD741x::AddrBit::GND,             // Indicate that the Address pin is GND
                      &AD7415Form[0],                   // Link to form buffer
                      I2C1_AD7415Form);                 // Define size of buffer

    GenBuffer<uint8_t>  AD7415ReadBuff(&I2CCommBuff[I2C_AD7415_G][0], I2C1_CommBuff);

    // Create local version of linked signals (prefix with "lc")
    // #=======================================================#
    I2CPeriph::DevFlt   lcI2C1CommFlt;              // Local version of I2C1 communication fault

    lcI2C1CommFlt   = I2CPeriph::DevFlt::Initialised;   // Initialise fault state

    HALDevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> lcAD74151CommState;
        // Local version of AD7415_1 Communication status indication
    lcAD74151CommState.ComFlt   = I2CPeriph::DevFlt::Initialised;   // Initialise fault state
    lcAD74151CommState.DevFlt   = AD741x::DevFlt::Initialised;      // Initialise fault state
    lcAD74151CommState.IdleCount= 0;                                // Initialise fault

    // Local version of value of External device temperature
    _HALParam   lcExtTmp    = {.data = -999.0,             // Initialise signal with default value
                               .flt  = _HALParam::Faulty };// Indicate data as fault


    // Enable I2C interrupts:
    I2C1Dev.configBusNACKIT(I2CPeriph::ITEnable);       // Enable Bus NACK fault interrupt
    I2C1Dev.configBusSTOPIT(I2CPeriph::ITEnable);       // Enable Bus STOP fault interrupt
    I2C1Dev.configBusErroIT(I2CPeriph::ITEnable);       // Enable Bus Error fault interrupt

    /* Following contains the main aspects of this task-------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t FirstPass = 1;      // Indicate that this is the first time going through this task

    uint32_t PreviousWakeTime = osKernelSysTick();  // Capture start time of task within Kernel
                                                    // time
    /* Infinite loop */
    for(;;) {
        ticStartTask(miI2C1__Task);     // Capture time of start of task

        // Check the status of the AD7415_1 communication
        AD74151_IntrManage(  &I2C1Dev,
                             &AD7415Dev,
                             &I2CWriteBuff,
                             &AD7415ReadBuff,
                             &lcAD74151CommState
                          );
        if ((lcAD74151CommState.DevFlt  != AD741x::DevFlt::None) || (FirstPass == 1))
            // If the AD7415 device/communication is faulty, OR it has been the first pass through
            // of task then
            lcExtTmp.flt    = _HALParam::Faulty;        // Indicate data is faulty
        else
        {   // Otherwise data is good, and this is not the first pass
            lcExtTmp.flt    = _HALParam::NoFault;       // Indicate data is fault free
            lcExtTmp.data   = AD7415Dev.Temp;           // Capture the recorded angle position
        }

        // Link internal signals to output pointers:
        *(pxParameters.output.I2C1CommFlt)  = lcI2C1CommFlt;
        *(pxParameters.output.AD74151Flt)   = lcAD74151CommState;
        *(pxParameters.output.ExtTmp)       = lcExtTmp;


        FirstPass = 0;              // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        tocStopTask(miI2C1__Task);  // Capture time task completed
        osDelayUntil(&PreviousWakeTime, I2C___DEV_Time);
        // Put the task in "DELAYED" state for the defined period
        // see - miStepperCONFIG.h for the timings
    }
}

/**
  * @brief:  I2C1 Event Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void I2C1_EV_IRQHandler(void) { I2C1_Handle->IRQEventHandle(); }

/**
  * @brief:  I2C1 Error Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void I2C1_ER_IRQHandler(void) { I2C1_Handle->IRQErrorHandle(); }

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @brief:  Will request multiple communication session with AD7415_1, and will be queued into
  *          the I2C1 device
  * @param:  I2C Peripheral pointer
  * @param:  AD741x device pointer
  * @param:  GenBuffer pointer to the generic write buffer
  * @param:  GenBuffer pointer to the AD741x specific read buffer
  * @retval: None (void output)
  */
static void AD74151_PacketSetup(I2CPeriph *hi2c, AD741x *device, GenBuffer<uint8_t> *wrBuff,
                                                                 GenBuffer<uint8_t> *rdBuff)
{
    device->intConfigWrite(hi2c,                        // Construct a write request to AD7415
                           AD741x::PwrState::StandBy,   // device, putting it into 'Standby' mode
                           AD741x::FiltState::Enabled,  // Filter is enabled
                           AD741x::OneShot::TrigConv,   // and trigger a conversion request
                           // (in 'Standby' conversion will only occur upon request)
                           wrBuff);                     // Provide pointer to array/buffer to
                                                        // request data from

    device->intConfigRead(hi2c, rdBuff, wrBuff);        // Read back the status of AD7415
                                                        // configuration
    device->intTempRead(hi2c, rdBuff, wrBuff);          // Read back temperature
}

/**
  * @brief:  Determines whether there is new data available to be decoded from the AD741x device.
  *          Will also determine fault status of device communication.
  * @param:  I2C Peripheral pointer
  * @param:  AD741x device pointer
  * @param:  GenBuffer pointer to the generic write buffer
  * @param:  GenBuffer pointer to the AD741x specific read buffer
  * @param:  Pointer to the HALDevComFlt specific for the AD741x device, so includes the AD741x
  *          fault type, and I2C Peripheral fault type
  * @retval: None (void output)
  */
void AD74151_IntrManage(I2CPeriph *hi2c, AD741x *device, GenBuffer<uint8_t> *wrBuff,
                                                         GenBuffer<uint8_t> *rdBuff,
                        HALDevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> *CommState)
{
    /* AD7415_1 I2C communication interrupt management steps:
     * ***********
     *  1 =     If there is no fault with the I2C interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then decode data and bring contents into
     *          the AD741x class type. No faults triggered, and all communication counts cleared.
     *          Next communication is organised, and requested of the hardware.
     *
     *  2 =     If there is no fault with the I2C interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then de-code data and bring contents into
     *          the AD741x class type. If the class determines a fault with the data, then
     *          indicate that the external device is faulty. Re-initialise the AD741x class type
     *          internals, and organised a new communication - and sent request to hardware.
     *
     *  3 =     If there is no fault with the I2C interrupt communication, and the correct number
     *          of bytes/packets have NOT been transmitted. Then increment a count of "blackout"
     *          iterations.
     *          If this count exceeds the threshold 'I2C1_FaultCount', then indicate a device no
     *          communication fault.
     *
     *  4 =     If there is a I2C interrupt communication fault, then indicate a I2C communication
     *          fault, setup device fault as parent comm fault.
     *          Re-initialise the AD741x class type internals, clear the I2C interrupt
     *          communication fault, and send a new transmit request to hardware - in effort to
     *          re-establish communication.
     *********************************************************************************************/
    if ( (device->I2CWFlt == I2CPeriph::DevFlt::None) &&
         (device->I2CRFlt == I2CPeriph::DevFlt::None) ) {   // If there is no fault with the I2C
                                                            // communication then...
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if ( (device->rdcmpTarget == device->rdcmpFlag) &&
             (device->wtcmpTarget == device->wtcmpFlag) ) { // If the expected number of bytes
                                                            // been written/read then
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            device->intCheckCommStatus(rdBuff, device->rdcmpTarget);
                // Read all available from selected AD741x device

            // Clear communication count fault flags
            device->wtcmpTarget     = 0;            // Clear the communication write target count
            device->rdcmpTarget     = 0;            // Clear the communication  read target count

            device->wtcmpFlag       = 0;            // Clear the communication write flag (actual
                                                    // count)
            device->rdcmpFlag       = 0;            // Clear the communication  read flag (actual
                                                    // count)

            // Set device communication state to None for I2C communication fault, and device
            // fault to be equal to output of AD741x class
            CommState->ComFlt   = I2CPeriph::DevFlt::None;
            CommState->DevFlt   = device->Flt;

            if (device->Flt != AD741x::DevFlt::None)    // If there is a detected fault with the
                                                        // device, then
                device->reInitialise();                 // Re-initialise the class internals

            CommState->IdleCount    = 0;                // Clear communication "blackout" count

            AD74151_PacketSetup(hi2c, device, wrBuff, rdBuff);  // Setup a new set of packets to
                                                                // transmit.
        }
        else {  // If the number of bytes have not been written/read yet, then...
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            if (CommState->IdleCount >= I2C1_FaultCount) {  // If the count exceeds the limit,
                                                            // then
                CommState->DevFlt   = AD741x::DevFlt::NoCommunication;
                // Indicate that there is no communication with device
            }
            else                            // Only if the counter is less then threshold
                CommState->IdleCount++;     // Increment "blackout" counter
        }
    }
    else {  // If there is a detected bus (I2C) fault during communication then
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if (device->I2CRFlt != I2CPeriph::DevFlt::None)     // If there is a read fault
            CommState->ComFlt = device->I2CRFlt;            // then capture
        else                                                // If the read is fault free
            CommState->ComFlt = device->I2CWFlt;            // then captured write fault

        CommState->DevFlt   = AD741x::DevFlt::ParentCommFlt;    // Indicate parent comm fault

        device->reInitialise();                 // Reinitialise the internal class

        device->I2CRFlt   = I2CPeriph::DevFlt::None;        // Clear communication faults
        device->I2CWFlt   = I2CPeriph::DevFlt::None;        // Clear communication faults

        AD74151_PacketSetup(hi2c, device, wrBuff, rdBuff);      // Setup a new set of packets to
                                                                // transmit.
        // This is to allow for any temporary hardware fault to be cleared, and communication to
        // be re-established
    }
}
