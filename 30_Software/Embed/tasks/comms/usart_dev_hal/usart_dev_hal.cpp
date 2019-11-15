/**************************************************************************************************
 * @file        usart_dev_hal.cpp
 * @author      Thomas
 * @version     V2.1
 * @date        28 Sept 2019
 * @brief       Source file for USART communication task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/

#include "EmbedIndex.h"
#include EMBD_USARTTask

/**************************************************************************************************
 * Globally defined variables used GLOBALLY throughout system (not just this task)
 *************************************************************************************************/
float                     FanDmd = 0;
uint8_t                   StpEnable   = 0;
uint8_t                   StpGear     = 0;
uint8_t                   StpDirct    = 0;
uint16_t                  StpFreqDmd  = 0;

/**************************************************************************************************
 * Define any externally consumed global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// Hardware parameters
extern UART_HandleTypeDef huart1;           // Defined within 'main.cpp'

extern DMA_HandleTypeDef hdma_usart1_rx;    // Defined within 'main.cpp'
extern DMA_HandleTypeDef hdma_usart1_tx;    // Defined within 'main.cpp'

// Task inputs
extern _HALParam                                           AngPos;
extern SPIPeriph::DevFlt                                   SPI1CommFlt;
extern HALDevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt>     AS5048AFlt;

extern _HALParam                                           ExtTmp;
extern I2CPeriph::DevFlt                                   I2C1CommFlt;
extern HALDevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt>     AD74151Flt;

extern _HALParam                                           IntVrf;
extern _HALParam                                           IntTmp;

extern _HALParam                                           FanVlt;
extern _HALParam                                           FanCur;
extern _HALParam                                           FanAct;

extern _HALParam                                           StpVlt;
extern _HALParam                                           StpCur;
extern uint16_t                                            StpFreqAct;
extern uint16_t                                            StpStatAct;
extern uint32_t                                            StpcalPost;

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * Define any local global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
UARTPeriph::Form    lcReadBuffer[2]  = { 0 };
UARTPeriph::Form    lcWrteBuffer[64] = { 0 };

uint8_t             lcUSART1Arr[2][USART1_CommBuff] = { 0 };// Array used to contain USART comm
                                                            // data
        // Defined outside of the RTOS function so isn't added to the stack
static UARTDMAPeriph    *lcUSART1_Handle;// Pointer to the USART1 class handle, for interrupt use

typedef struct _miStpRdStatus{
    enum  RdStatus  {   Idle        = 0,
                        Listening   = 1

                    }   _state;

    enum Captured   {   DataRead    = 0,
                        NewData     = 1
                    }   _capt;

    uint8_t     rdPointer;
    uint8_t     rdData[MISTEPPER_RECEIVE];

}   _miStpRd;

_miStpRd    lcRdState = { ._state     = _miStpRdStatus::Idle,
                          ._capt      = _miStpRdStatus::DataRead,
                          .rdPointer  = 0,
                          .rdData     = { 0 } };

uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

/**************************************************************************************************
 * Define any local functions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void Transmit16bit(GenBuffer<uint8_t> *buffer, uint16_t data);
void Transmit32bit(GenBuffer<uint8_t> *buffer, uint32_t data);
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

/**
  * @brief:  USART1 Device Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vUSART1DeviceHAL(void const * argument) {
/*---------------------------[  Setup HAL based classes for H/W   ]---------------------------*/
    // Create locally used variables within task:
    uint32_t i = 0;                                 // Define a variable used to count loops

    GenBuffer<uint8_t>  USARTGenBuff[2] = {                         // Link USRAT Comm array to
                                                                    // GenBuffer
                {&lcUSART1Arr[0][0], USART1_CommBuff},              // 1st = Write Buffer
                {&lcUSART1Arr[1][0], USART1_CommBuff}               // 2nd = Read  Buffer
    };

    // Create USART1 class
    // ===================
    UARTDMAPeriph  USART1Dev(&huart1, &lcWrteBuffer[0], 64,
                                      &lcReadBuffer[0],  2,
                                      &hdma_usart1_rx,  &hdma_usart1_tx);
    lcUSART1_Handle = &USART1Dev;   // Link USART1Dev class to global pointer (for ISR)

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

    volatile UARTPeriph::DevFlt ReadBackFault = UARTPeriph::DevFlt::None;
    volatile uint16_t countReadBack = 0;

    volatile UARTPeriph::DevFlt WrteBackFault = UARTPeriph::DevFlt::None;
    volatile uint16_t countWrteBack = 0;

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
        lcAngPos    = AngPos;
        lcSPI1Flt   = SPI1CommFlt;
        lcAS5048Flt = AS5048AFlt;

        lcExtTmp    = ExtTmp;
        lcI2C1Flt   = I2C1CommFlt;
        lcAD7415Flt = AD74151Flt;

        lcIntVrf    = IntVrf;
        lcIntTmp    = IntTmp;

        lcFanVlt    = FanVlt;
        lcFanCur    = FanCur;
        lcFanAct    = FanAct;

        lcStpVlt    = StpVlt;
        lcStpCur    = StpCur;
        lcStpfrq    = StpFreqAct;
        lcStpste    = StpStatAct;
        lcStpcPs    = StpcalPost;

        USART1Dev.Read_GenBufferLock(&USARTGenBuff[1], &ReadBackFault, &countReadBack);

        while (USARTGenBuff[1].OutputRead(&readback)  != GenBuffer_Empty) {
            if (lcRdState._state == _miStpRdStatus::Idle) {
                if ( ( lcRdState.rdPointer  == 0 ) && ( readback == 0xFF ) ) {
                    lcRdState.rdData[lcRdState.rdPointer] = readback;
                    lcRdState.rdPointer = 1;
                }
                else if ( ( lcRdState.rdPointer  == 1 ) && ( readback == 0xA5 ) ) {
                    lcRdState.rdData[lcRdState.rdPointer] = readback;
                    lcRdState.rdPointer = 2;
                    lcRdState._state    = _miStpRdStatus::Listening;
                }
                else {  // If sequence above not achieved, then restart!
                    lcRdState.rdPointer = 0;
                }
            }
            else if (lcRdState._state == _miStpRdStatus::Listening) {
                lcRdState.rdData[lcRdState.rdPointer++] = readback;

                if (lcRdState.rdPointer >= MISTEPPER_RECEIVE) {
                    lcRdState.rdPointer   = 0;
                    lcRdState._state      = _miStpRdStatus::Idle;
                    ///////////////////////////////////////////////////////////////////////////////
                    // Receive the USART package(s):
                    // Layout of data in alignment with "PacketTransmission.xlsx" Sheet
                    //          "USARTPacket_miStpIn"                                Version 1.1
                    //
                    if (update_crc(0, &lcRdState.rdData[0x00], MISTEPPER_RECEIVE) == 0) {
                        TransmitMode    = lcRdState.rdData[0x03];

                        if ( (TransmitMode & (1 << EnableInputBit)) != 0) {

                            lcFanDmd        = DataManip::_4x8bit_2_float(&lcRdState.rdData[0x04]);

                            lcStpEnable     = lcRdState.rdData[0x08];
                            lcStpGear       = lcRdState.rdData[0x09];
                            lcStpDirct      = lcRdState.rdData[0x0A];

                            lcStpFreqDmd    = DataManip::_2x8bit_2_16bit(&lcRdState.rdData[0x0C]);
                        }
                        else {
                            lcFanDmd        = 0;
                            lcStpEnable     = 0;
                            lcStpGear       = 0;
                            lcStpDirct      = 0;
                            lcStpFreqDmd    = 0;
                        }
                    }
                    ///////////////////////////////////////////////////////////////////////////////
                }
            }
        }

        if ( (TransmitMode & (1 << ResetPktCountBt)) != 0) {
            PckCount        = 0;        // Reset the package counter
            TransmitMode    &= ~(1 << ResetPktCountBt);
        }

        if ( (TransmitMode & (1 << TransmitDataBit)) != 0) {
            USARTGenBuff[0].QFlush();   // Clear the GenBuffer (return points to start)

            // Transmit the USART package(s):
            // Layout of data in alignment with "PacketTransmission.xlsx" Sheet
            //          "USARTPacket_miStpOut"                               Version 1.2
            // Initially transmit new package identifier:
            USARTGenBuff[0].InputWrite(     0xFF                                );
            USARTGenBuff[0].InputWrite(     0x5A                                );
            Transmit16bit(&USARTGenBuff[0], PckCount                            );

            /* SPI1 packets:                                                    */
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcAngPos.data        );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcSPI1Flt                 );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcAS5048Flt.ComFlt        );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcAS5048Flt.DevFlt        );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcAS5048Flt.IdleCount     );

            Transmit32bit(&USARTGenBuff[0], miTaskData(miSPI1__Task)            );

            /* I2C1 packets:                                                    */
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcExtTmp.data        );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcI2C1Flt                 );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcAD7415Flt.ComFlt        );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcAD7415Flt.DevFlt        );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcAD7415Flt.IdleCount     );

            Transmit32bit(&USARTGenBuff[0], miTaskData(miI2C1__Task)            );

            /* ADC1 packets:                                                    */
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcIntVrf.data        );
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcIntTmp.data        );
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcFanVlt.data        );
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcFanCur.data        );
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcStpVlt.data        );
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcStpCur.data        );

            USARTGenBuff[0].InputWrite(     0x00                                );
            USARTGenBuff[0].InputWrite(     0x00                                );
            USARTGenBuff[0].InputWrite(     0x00                                );
            USARTGenBuff[0].InputWrite(     (uint8_t) lcIntVrf.flt              );

            Transmit32bit(&USARTGenBuff[0], miTaskData(miADC1__Task)            );

            /* FAN packets:                                                     */
            Transmit32bit(&USARTGenBuff[0], *(uint32_t *) &lcFanAct.data        );

            Transmit32bit(&USARTGenBuff[0], miTaskData(miFan___Task)            );

            /* STEPPER packets:                                                 */
            Transmit16bit(&USARTGenBuff[0], lcStpfrq                            );
            Transmit16bit(&USARTGenBuff[0], lcStpste                            );
            Transmit32bit(&USARTGenBuff[0], lcStpcPs                            );

            Transmit32bit(&USARTGenBuff[0], miTaskData(miSteperTask)            );

            /* USART1 packets:                                                  */
            Transmit32bit(&USARTGenBuff[0], miTaskData(miUSART1Task)            );

            WrteBackFault = UARTPeriph::DevFlt::None;
            countWrteBack = 0;

            uint16_t temp = 0;
            temp = update_crc(0, USARTGenBuff[0].pa, USARTGenBuff[0].input_pointer);

            /* CRC    packets:                                                  */
            Transmit16bit(&USARTGenBuff[0], temp                                 );

            USART1Dev.intWrtePacket(USARTGenBuff[0].pa, USARTGenBuff[0].input_pointer,
                                    &WrteBackFault, &countWrteBack);

            PckCount++;     // Increase packet counter

            TransmitMode    &= ~(1 << TransmitDataBit);
        }

        // Link internal signals to output pointers:
        FanDmd          = lcFanDmd;
        StpEnable       = lcStpEnable;
        StpGear         = lcStpGear;
        StpDirct        = lcStpDirct;
        StpFreqDmd      = lcStpFreqDmd;

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
void USART1_IRQHandler(void) { lcUSART1_Handle->IRQHandle(); };

/**
  * @brief:  DMA1 Channel 4 USART1 (Transmit) Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void DMA1_Channel4_IRQHandler(void) { lcUSART1_Handle->IRQDMATxHandle(); };

/**
  * @brief:  DMA1 Channel 5 USART1 (Receive) Interrupt Service Routine handler.
  * @param:  None (void input)
  * @retval: None (void output)
  */
void DMA1_Channel5_IRQHandler(void) { lcUSART1_Handle->IRQDMARxHandle(); };

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

void Transmit16bit(GenBuffer<uint8_t> *buffer, uint16_t data) {
    uint8_t tmparray[2] = {  0  };

    DataManip::_16bit_2_2x8bit(data,  &tmparray[0]);

    buffer->QuickWrite(&tmparray[0], 2);
}

void Transmit32bit(GenBuffer<uint8_t> *buffer, uint32_t data) {
    uint8_t tmparray[4] = { 0 };

    DataManip::_32bit_2_4x8bit(data,  &tmparray[0]);

    buffer->QuickWrite(&tmparray[0], 4);
}

uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{

    uint16_t i, j;

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
