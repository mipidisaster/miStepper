/**************************************************************************************************
 * @file        spi_dev_hal.cpp
 * @author      Thomas
 * @version     V2.1
 * @date        15 Sept 2019
 * @brief       Source file for SPI Device task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/

#include "EmbedIndex.h"
#include EMBD_SPITask

/**************************************************************************************************
 * Define any local global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static SPIPeriph    *SPI1_Handle;       // Pointer to the SPI1 class handle, for interrupt use


/**************************************************************************************************
 * Define any local functions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void AS5048A_IntrManage(SPIPeriph *hspi, AS5x4x *device, AS5x4x::Daisy *daisy, GPIO *CS,
                        uint8_t *wtBuff, uint8_t *rdBuff,
                        HALDevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt> *CommState);
// Prototype for AS5048A Packet setup and management.


/**
  * @brief:  SPI1 Device Hardware Abstraction Layer task
  * @param:  _taskSPI -> cast to a void pointer
  * @retval: None (void output)
  */
void vSPI1DeviceHAL(void const * pvParameters) {
    _taskSPI1 pxParameters;
    pxParameters = * (_taskSPI1 *) pvParameters;
    // pxParameters is to include the parameters required to configure and interface this task
    // with other tasks within the OS - see header file for parameters (config, input, output).
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // Create locally used variables within task:
    SPIPeriph::Form     SPIForm[SPI1_FormBuffer]        = { 0 };    // SPI1 Form initialised
    uint8_t             SPICommBuff[2][SPI1_CommBuff]   = { 0 };    // SPI Communication array

    // Create SPI1 class
    // =================
    SPIPeriph   SPI1Dev(pxParameters.config.spi1_handle, &SPIForm[0], SPI1_FormBuffer);
    SPI1_Handle = &SPI1Dev;         // Link SPI1Dev class to global pointer (for ISR)

/*---------------------------[ Setup SPI Connected Device Classes ]---------------------------*/
    uint16_t AS5048Arr[2][SPI1_AS5048AForm] = { 0 };            // AS5048 internal class buffer

    AS5x4x  AS5048ADev(AS5x4x::DevPart::AS5048A,        // Construct a AS5048A class handle
                       &AS5048Arr[SPIWt_Arry][0],       // Link internal write form buffer
                       &AS5048Arr[SPIRd_Arry][0],       // Link internal read from buffer
                       SPI1_AS5048AForm);               // Define size of buffer

    // Link AS5048A device to Daisy chain structure for simpler interrupt handling.
    AS5x4x::Daisy   AS5Daisy = AS5x4x::constructDaisy(&AS5048ADev, 1);

    // Create SPI Device(s) Chip Select
    // ================================
    GPIO    AS5048CS(AS5048_GPIO_Port,  AS5048_Pin,  GPIO::OUTPUT);

    // Create local version of linked signals (prefix with "lc")
    // #=======================================================#
    SPIPeriph::DevFlt   lcSPI1CommFlt;              // Local version of SPI1 communication fault

    lcSPI1CommFlt    = SPIPeriph::DevFlt::Initialised;   // Initialise fault state

    HALDevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt> lcAS5048ACommState;
        // Local version of AS5048 Communication status indication
    lcAS5048ACommState.ComFlt   = SPIPeriph::DevFlt::Initialised;   // Initialise fault state
    lcAS5048ACommState.DevFlt   = AS5x4x::DevFlt::Initialised;      // Initialise fault state
    lcAS5048ACommState.IdleCount= 0;                                // Initialise fault state

    // Local version of value of Angle Position
    _HALParam   lcAngPos    = {.data = -999.0,             // Initialise signal with default value
                               .flt  = _HALParam::Faulty };// Indicate data as fault


    // Enable SPI interrupts:
    SPI1Dev.configBusErroIT(SPIPeriph::ITEnable);       // Enable Bus communication fault interrupt

    /* Following contains the main aspects of this task-------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t FirstPass = 1;      // Indicate that this is the first time going through this task

    uint32_t PreviousWakeTime = osKernelSysTick();  // Capture start time of task within Kernel
                                                    // time
    /* Infinite loop */
    for(;;) {
        ticStartTask(miSPI1__Task);     // Capture time of start of task

        // Check the status of the AS5048A communication
        AS5048A_IntrManage(  &SPI1Dev,
                             &AS5048ADev,
                             &AS5Daisy,
                             &AS5048CS,
                             &SPICommBuff[SPIWt_Arry][0], &SPICommBuff[SPIRd_Arry][0],
                             &lcAS5048ACommState
                          );
        if ((lcAS5048ACommState.DevFlt  != AS5x4x::DevFlt::None) || (FirstPass == 1))
            // If the AS5048 device/communication is faulty, OR it has been the first pass through
            // of task then
            lcAngPos.flt    = _HALParam::Faulty;        // Indicate data is faulty
        else
        {   // Otherwise data is good, and this is not the first pass
            lcAngPos.flt    = _HALParam::NoFault;       // Indicate data is fault free
            lcAngPos.data   = AS5048ADev.Angle;         // Capture the recorded angle position
        }

        // Link internal signals to output pointers:
        *(pxParameters.output.SPI1CommFlt)  = lcSPI1CommFlt;
        *(pxParameters.output.AS5048AFlt)   = lcAS5048ACommState;

        *(pxParameters.output.AngPos)       = lcAngPos;


        FirstPass = 0;              // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        tocStopTask(miSPI1__Task);  // Capture time task completed
        osDelayUntil(&PreviousWakeTime, SPI___DEV_Time);
        // Put the task in "DELAYED" state for the defined period
        // see - miStepperCONFIG.h for the timings
    }
}

/**
  * @brief:  SPI1 Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void SPI1_IRQHandler(void) { SPI1_Handle->IRQHandle(); };

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @brief:  Will request multiple communication sessions with AS5048, and will be queued into
  *          the SPI1 device
  * @param:  SPI Peripheral pointer
  * @param:  AS5x4x device pointer
  * @param:  Daisy chain structure pointer
  * @param:  Chip Select GPIO pointer
  * @param:  Array pointer to where data is read from and written to, expects this is an array
  *          with 'SPI1_CommBuff' columns
  * @retval: None (void output)
  */
static void AS5048_PacketSetup(SPIPeriph *hspi, AS5x4x *device, AS5x4x::Daisy *daisy, GPIO *CS,
                               uint8_t *wtBuff, uint8_t *rdBuff)
{
    device->constructCEF();         // Construct the Clear Error Flags request
    device->constructAGC();         // Construct the Automatic Gain Control request
    device->constructMag();         // Construct the CORDIC Magnitude
    device->constructAng();         // Construct the Angular position
    device->constructNOP();         // Construct NOP

    // Clear communication count fault flags
    daisy->Trgt     = 0;                    // Clear the communication target count
    daisy->Cmplt    = 0;                    // Clear the communication actual count

    AS5x4x::intSingleTransmit(hspi,                 // Add data requests to target SPI peripheral
                              CS,                   // Chip Select to use
                              daisy,                // Daisy Chain setup
                              rdBuff,               // Pointer to where read back data is to be
                                                    // stored
                              wtBuff);              // Pointer to where data to be written to
                                                    // AS5048 is to be taken from.
}

/**
  * @brief:  Determines whether there is new data available to be decoded from the AS5048 device.
  *          Will also determine fault status of device communication.
  * @param:  SPI Peripheral pointer
  * @param:  AS5x4x device pointer
  * @param:  Daisy chain structure pointer
  * @param:  Chip Select GPIO pointer
  * @param:  Array pointer to where data is read from and written to, expects this is an array
  *          with 'SPI1_CommBuff' columns
  * @param:  Pointer to the HALDevComFlt specific for the AS5x4x device, so includes the AS5x4x
  *          fault type, and SPI Peripheral fault type
  * @retval: None (void output)
  */
void AS5048A_IntrManage(SPIPeriph *hspi, AS5x4x *device, AS5x4x::Daisy *daisy, GPIO *CS,
                        uint8_t *wtBuff, uint8_t *rdBuff,
                        HALDevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt> *CommState)
{
    /* AS5048 SPI communication interrupt management steps:
     * ***********
     *  1 =     If there is no fault with the SPI interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then decode data and bring contents into
     *          the AS5x4x class type. No faults triggered, and all communication counts cleared.
     *          Next communication is organised, and requested of the hardware.
     *
     *  2 =     If there is no fault with the SPI interrupt communication, and the correct number
     *          of bytes/packets have been transmitted, then de-code data and bring contents into
     *          the AS5x4x class type. If the class determines a fault with the data, then
     *          indicate that the external device is faulty. Re-initialise the AS5x4x class type
     *          internals, and organised a new communication - and sent request to hardware.
     *
     *  3 =     If there is no fault with the SPI interrupt communication, and the correct number
     *          of bytes/packets have NOT been transmitted. Then increment a count of "blackout"
     *          iterations.
     *          If this count exceeds the threshold 'SPI1_FaultCount', then indicate a device no
     *          communication fault.
     *
     *  4 =     If there is a SPI interrupt communication fault, then indicate a SPI communication
     *          fault, setup device fault as parent comm fault.
     *          Re-initialise the AS5x4x class type internals, clear the SPI interrupt
     *          communication fault, and send a new transmit request to hardware - in effort to
     *          re-establish communication.
     *********************************************************************************************/
    if (daisy->Flt == SPIPeriph::DevFlt::None) {    // If there is no fault with the SPI
                                                    // communication then...
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if ( daisy->Trgt == daisy->Cmplt ) {            // If the expected number of bytes have
                                                        // been transmitted then
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            AS5x4x::SPIReadChain(daisy->Devices,        // Read all available data from every
                                 daisy->numDevices,     // AS5x4x supported device within daisy
                                 &rdBuff[0],            // chain.
                                 daisy->Trgt);
            //####
            // Currently only supports use of 1 connected AS5x4x device.
            //####
            AS5x4x::readDaisyPackets(daisy);            // Deconstruct all data, and integrate
                                                        // into each AS5x4x class instance.

            // Set device communication state to None for SPI communication fault, and device
            // fault to be equal to output of AS5x4x class
            CommState->ComFlt   = SPIPeriph::DevFlt::None;
            CommState->DevFlt   = device->Flt;

            if (device->Flt != AS5x4x::DevFlt::None)    // If there is a detected fault with the
                                                        // device, then
                device->reInitialise();                 // Re-initialise the class internals

            CommState->IdleCount    = 0;                // Clear communication "blackout" count

            AS5048_PacketSetup(hspi, device, daisy, CS, wtBuff, rdBuff);
            // Setup a new set of packets to transmit.
        }
        else {  // If the number of bytes have not been transmitted yet, then...
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            if (CommState->IdleCount >= SPI1_FaultCount) {  // If the count exceeds the limit,
                                                            // then
                CommState->DevFlt   = AS5x4x::DevFlt::NoCommunication;
                // Indicate that there is no communication with device
            }
            else                            // Only if the counter is less then threshold
                CommState->IdleCount++;     // Increment "blackout" counter
        }
    }
    else {  // If there is a detected bus (SPI) fault during communication then
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        CommState->ComFlt   = daisy->Flt;       // Copy across the communication fault
        CommState->DevFlt   = AS5x4x::DevFlt::ParentCommFlt;    // Indicate parent comm fault

        device->reInitialise();                 // Reinitialise the internal class

        daisy->Flt          = SPIPeriph::DevFlt::None;          // Clear the daisy chain
                                                                // communication fault
        AS5048_PacketSetup(hspi, device, daisy, CS, wtBuff, rdBuff);
        // Setup a new set of packets to transmit.
        // This is to allow for any temporary hardware fault to be cleared, and communication to
        // be re-established
    }
}
