/**************************************************************************************************
 * @file        usart_dev_hal.cpp
 * @author      Thomas
 * @version     V1.1
 * @date        25 Jun 2019
 * @brief       Source file for USART communication task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/

#include "EmbedIndex.h"
#include EMBD_USARTTask

/**************************************************************************************************
 * Define any local global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
uint8_t             USART1Arr[2][USART1_CommBuff];      // Array used to contain USART comm data
        // Defined outside of the RTOS function so isn't added to the stack
static UARTPeriph   *USART1_Handle;     // Pointer to the USART1 class handle, for interrupt use

typedef struct _miStpRdStatus{
    enum  RdStatus  {   Idle        = 0,
                        Listening   = 1

                    }   _state;

    enum Captured   {   DataRead    = 0,
                        NewData     = 1
                    }   _capt;

    uint8_t     rdPointer;
    uint8_t     rdData[16];

}   _miStpRd;

_miStpRd    RdState = { ._state     = _miStpRdStatus::Idle,
                        ._capt      = _miStpRdStatus::DataRead,
                        .rdPointer  = 0,
                        .rdData     = { 0 } };

/**************************************************************************************************
 * Define any local functions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void Transmit16bit(UARTPeriph *huart, uint16_t data);
void Transmit32bit(UARTPeriph *huart, uint32_t data);


/**
  * @brief:  USART1 Device Hardware Abstraction Layer task
  * @param:  _taskUSART -> cast to a void pointer
  * @retval: None (void output)
  */
void vUSART1DeviceHAL(void const * pvParameters) {
    _taskUSART1 pxParameters;
    pxParameters = * (_taskUSART1 *) pvParameters;
    // pxParameters is to include the parameters required to configure and interface this task
    // with other tasks within the OS - see header file for parameters (config, input, output).
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // Create locally used variables within task:
    uint32_t i = 0;                                 // Define a variable used to count loops
    for (i = 0; i != USART1_CommBuff; i++) {        // Loop through all data within "USART1Arr"
        USART1Arr[0][i] = 0;                        //
        USART1Arr[1][i] = 0;                        // Set all data point to "0"
    }
    GenBuffer<uint8_t>  USARTGenBuff[2] = {                         // Link USRAT Comm array to
                                                                    // GenBuffer
                {&USART1Arr[0][0], USART1_CommBuff},                // 1st = Write Buffer
                {&USART1Arr[1][0], USART1_CommBuff}                 // 2nd = Read  Buffer
    };

    // Create USART1 class
    // ===================
    UARTPeriph  USART1Dev(pxParameters.config.usart1_handle, &USARTGenBuff[1], &USARTGenBuff[0]);
    USART1_Handle = &USART1Dev;     // Link USART1Dev class to global pointer (for ISR)

    // Create local version of linked signals (prefix with "lc")
    // #=======================================================#

    // Input to this task
    // #=================#
    _HALParam   lcAngPos;           // Local version of value of Angle Position
    SPIPeriph::DevFlt   lcSPI1Flt;  // Local version of SPI1 Bus Fault
    HALDevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt> lcAS5048Flt;    // Local version of AS5048
                                                                    // Fault

    _HALParam   lcExtTmp;           // Local version of value of External device temperature
    I2CPeriph::DevFlt   lcI2C1Flt;  // Local version of I2C1 Bus Fault
    HALDevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt> lcAD7415Flt;    // Local version of AD7415
                                                                    // Fault

    _HALParam   lcIntVrf;           // Local version of Voltage Reference
    _HALParam   lcIntTmp;           // Local version of Internal Temperature

    _HALParam   lcFanVlt;           // Local version of Fan Motor Voltage
    _HALParam   lcFanCur;           // Local version of Fan Motor Current
    _HALParam   lcFanAct;           // Local version of Fan Motor Actual Demand

    _HALParam   lcStpVlt;           // Local version of Stepper Motor Voltage
    _HALParam   lcStpCur;           // Local version of Stepper Motor Current
    uint16_t    lcStpfrq;           // Local version of Stepper Frequency
    uint16_t    lcStpste;           // Local version of Stepper state
    uint32_t    lcStpcPs;           // Local version of Stepper calcPosition

    // Output to this task
    // #=================#
    float       lcFanDmd        = 0.0;  // Local parameter for Fan Demand
    uint8_t     lcStpEnable     = 0;    // Local parameter for Stepper Enable
    uint8_t     lcStpGear       = 0;    // Local parameter for Stepper Gear
    uint8_t     lcStpDirct      = 0;    // Local parameter for Stepper Direction
    uint16_t    lcStpFreqDmd    = 0;    // Local parameter for Stepper Frequency Demand

    // Local parameters
    uint16_t    PckCount = 0;       // Communication Packet count
    uint8_t     TransmitMode = 0;   // Mode for transmitting data
    uint8_t     readback  = 0;      // Variable to store read data from USART

    // Enable USART interrupts:
    USART1Dev.ReceiveIT(UART_Enable);                   // Enable Receive interrupts

    /* Following contains the main aspects of this task-------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t FirstPass = 1;      // Indicate that this is the first time going through this task

    uint32_t PreviousWakeTime = osKernelSysTick();  // Capture start time of task within Kernel
                                                    // time
    /* Infinite loop */
    for(;;) {
        ticStartTask(miUSART1Task);     // Capture time of start of task

        // Capture any new updates to input signals and link to local internals
        lcAngPos    = *(pxParameters.input.AngPos);
        lcSPI1Flt   = *(pxParameters.input.SPI1CommFlt);
        lcAS5048Flt = *(pxParameters.input.AS5048AFlt);

        lcExtTmp    = *(pxParameters.input.ExtTmp);
        lcI2C1Flt   = *(pxParameters.input.I2C1CommFlt);
        lcAD7415Flt = *(pxParameters.input.AD74151Flt);

        lcIntVrf    = *(pxParameters.input.IntVrf);
        lcIntTmp    = *(pxParameters.input.IntTmp);

        lcFanVlt    = *(pxParameters.input.FanVlt);
        lcFanCur    = *(pxParameters.input.FanCur);
        lcFanAct    = *(pxParameters.input.FanAct);

        lcStpVlt    = *(pxParameters.input.StpVlt);
        lcStpCur    = *(pxParameters.input.StpCur);
        lcStpfrq    = *(pxParameters.input.StpFreqAct);
        lcStpste    = *(pxParameters.input.StpStatAct);
        lcStpcPs    = *(pxParameters.input.StpcalPost);

        while (USART1Dev.SingleRead_IT(&readback)  != GenBuffer_Empty) {
            if (RdState._state == _miStpRdStatus::Idle) {
                if ( ( RdState.rdPointer  == 0 ) && ( readback == 0xFF ) ) {
                    RdState.rdPointer++;
                }
                else if ( ( RdState.rdPointer  == 1 ) && ( readback == 0xA5 ) ) {
                    RdState.rdPointer = 2;
                    RdState._state    = _miStpRdStatus::Listening;
                }
            }
            else if (RdState._state == _miStpRdStatus::Listening) {
                RdState.rdData[RdState.rdPointer] = readback;

                if (RdState.rdPointer >= 15) {
                    RdState.rdPointer   = 0;
                    RdState._state      = _miStpRdStatus::Idle;
                    RdState._capt       = _miStpRdStatus::NewData;
                }
                else {
                    RdState.rdPointer++;
                }
            }
        }

        if (RdState._capt == _miStpRdStatus::NewData) {
            RdState._capt = _miStpRdStatus::DataRead;

            // Receive the USART package(s):
            // Layout of data in alignment with "PacketTransmission.xlsx" Sheet
            //          "USARTPacket_miStpIn"                                Version 1.0

            TransmitMode    = RdState.rdData[3];

            if ( (TransmitMode & (1 << EnableInputBit)) != 0) {

                lcFanDmd        = DataManip::_4x8bit_2_float(&RdState.rdData[0x04]);

                lcStpEnable     = RdState.rdData[0x08];
                lcStpGear       = RdState.rdData[0x09];
                lcStpDirct      = RdState.rdData[0x0A];

                lcStpFreqDmd    = DataManip::_2x8bit_2_16bit(&RdState.rdData[0x0C]);
            }
            else {
                lcFanDmd        = 0;
                lcStpEnable     = 0;
                lcStpGear       = 0;
                lcStpDirct      = 0;
                lcStpFreqDmd    = 0;
            }

        }


/*
        while (USART1Dev.SingleRead_IT(&readback)  != GenBuffer_Empty) {
            if      (  (readback == 'D') || (readback == 'd') )  {
                TransmitMode    |= (1 << TransmitDataBit);
            }
            else if (  (readback == 'R') || (readback == 'r') )  {
                TransmitMode    |= (1 << ResetPktCountBt);
            }
        }
        uint8     InterfaceReg

        # Fan Motor request value
        float32   FanDmd

        # Stepper Requests
        uint8     STPEnable
        uint8     STPGear
        uint8     STPDir
        uint16    STPFreq
*/

        if ( (TransmitMode & (1 << ResetPktCountBt)) != 0) {
            PckCount        = 0;        // Reset the package counter
            TransmitMode    &= ~(1 << ResetPktCountBt);
        }

        if ( (TransmitMode & (1 << TransmitDataBit)) != 0) {
            // Transmit the USART package(s):
            // Layout of data in alignment with "PacketTransmission.xlsx" Sheet
            //          "USARTPacket_miStpOut"                               Version 1.1
            // Initially transmit new package identifier:
            USART1Dev.SingleTransmit_IT(    0xFF                                );
            USART1Dev.SingleTransmit_IT(    0x5A                                );
            Transmit16bit(&USART1Dev,       PckCount                            );

            /* SPI1 packets:                                                    */
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcAngPos.data        );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcSPI1Flt                 );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcAS5048Flt.ComFlt        );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcAS5048Flt.DevFlt        );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcAS5048Flt.IdleCount     );

            Transmit32bit(&USART1Dev,       miTaskData(miSPI1__Task)            );

            /* I2C1 packets:                                                    */
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcExtTmp.data        );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcI2C1Flt                 );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcAD7415Flt.ComFlt        );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcAD7415Flt.DevFlt        );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcAD7415Flt.IdleCount     );

            Transmit32bit(&USART1Dev,       miTaskData(miI2C1__Task)            );

            /* ADC1 packets:                                                    */
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcIntVrf.data        );
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcIntTmp.data        );
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcFanVlt.data        );
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcFanCur.data        );
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcStpVlt.data        );
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcStpCur.data        );

            USART1Dev.SingleTransmit_IT(    0x00                                );
            USART1Dev.SingleTransmit_IT(    0x00                                );
            USART1Dev.SingleTransmit_IT(    0x00                                );
            USART1Dev.SingleTransmit_IT(    (uint8_t) lcIntVrf.flt              );

            Transmit32bit(&USART1Dev,       miTaskData(miADC1__Task)            );

            /* FAN packets:                                                     */
            Transmit32bit(&USART1Dev,       *(uint32_t *) &lcFanAct.data        );

            Transmit32bit(&USART1Dev,       miTaskData(miFan___Task)            );

            /* STEPPER packets:                                                 */
            Transmit16bit(&USART1Dev,       lcStpfrq                            );
            Transmit16bit(&USART1Dev,       lcStpste                            );
            Transmit32bit(&USART1Dev,       lcStpcPs                            );

            Transmit32bit(&USART1Dev,       miTaskData(miSteperTask)            );

            /* USART1 packets:                                                  */
            Transmit32bit(&USART1Dev,       miTaskData(miUSART1Task)            );

            PckCount++;     // Increase packet counter

            TransmitMode    &= ~(1 << TransmitDataBit);
        }

        // Link internal signals to output pointers:
        *(pxParameters.output.FanDmd)       = lcFanDmd;
        *(pxParameters.output.StpEnable)    = lcStpEnable;
        *(pxParameters.output.StpGear)      = lcStpGear;
        *(pxParameters.output.StpDirct)     = lcStpDirct;
        *(pxParameters.output.StpFreqDmd)   = lcStpFreqDmd;

        FirstPass = 0;              // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        tocStopTask(miUSART1Task);  // Capture time task completed
        osDelayUntil(&PreviousWakeTime, USART_DEV_Time);
        // Put the task in "DELAYED" state for the defined period
        // see - miStepperCONFIG.h for the timings
    }
}

/**
  * @brief:  USART1 Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void USART1_IRQHandler(void) { USART1_Handle->IRQHandle(); };

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

void Transmit16bit(UARTPeriph *huart, uint16_t data) {
    uint8_t tmparray[2] = {  0  };

    DataManip::_16bit_2_2x8bit(data,  &tmparray[0]);

    huart->SingleTransmit_IT( tmparray[0] );
    huart->SingleTransmit_IT( tmparray[1] );
}

void Transmit32bit(UARTPeriph *huart, uint32_t data) {
    uint8_t tmparray[4] = { 0 };

    DataManip::_32bit_2_4x8bit(data,  &tmparray[0]);

    huart->SingleTransmit_IT( tmparray[0] );
    huart->SingleTransmit_IT( tmparray[1] );
    huart->SingleTransmit_IT( tmparray[2] );
    huart->SingleTransmit_IT( tmparray[3] );
}
