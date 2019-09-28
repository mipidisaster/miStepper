/**************************************************************************************************
 * @file        adc_hal.cpp
 * @author      Thomas
 * @version     V2.1
 * @date        28 Sept 2019
 * @brief       Source file for ADC input task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/

#include "EmbedIndex.h"
#include EMBD_ADCTask

/**************************************************************************************************
 * Define any externally consumed global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
extern ADC_HandleTypeDef hadc1;             // Defined within'main.cpp'
extern DMA_HandleTypeDef hdma_adc1;         // Defined within'main.cpp'

extern TIM_HandleTypeDef htim6;             // Defined within'main.cpp'

/**************************************************************************************************
 * Define any local global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static volatile uint8_t ADC1SequCount = 0;          // Counter for number of ADC runs completed

struct ADCDMARecord {                                   // ADC Record structure
    uint16_t    Analog[ADC1_NumSeq * ADC1_ConvPSeq];    // Array to contain all conversions for
                                                        // defined number of runs
    enum RecordFault : uint8_t {NoFault = 0, Fault = 1} Flt;    // Fault status
};

static ADCDMARecord             CurADC1Record = { 0 };  // Global 'ADCDMARecord' linked to DMA
                                                        // and used within interrupts, to restart
                                                        // DMA if fault occurs
static GenBuffer<ADCDMARecord>  *ADC1Buff_Handle;       // Pointer to the GenBuff ADCDMARecord
                                                        // buffer, for interrupt use
static ADC_HandleTypeDef        *ADC1_Handle;           // Pointer to ADC hardware register and
static DMA_HandleTypeDef        *ADC1DMA_Handle;        // linked DMA register, for interrupt use


/**************************************************************************************************
 * Define any local functions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void ADC1_InternalVoltage(ADCDMARecord *Record, float *IntVRef);
void ADC1_InternalTemperature(ADCDMARecord *Record, float *IntTmp, float *IntVRef);
void ADC1_FANMotorVoltage(ADCDMARecord *Record, float *FanVlt, float *IntVRef);
void ADC1_FANMotorCurrent(ADCDMARecord *Record, float *FanCur, float *IntVRef);
void ADC1_STPMotorVoltage(ADCDMARecord *Record, float *StpVlt, float *IntVRef);
void ADC1_STPMotorCurrent(ADCDMARecord *Record, float *StpCur, float *IntVRef);
void ADC1_AverageParameter(float *param);
// Prototype(s) for conversion of the ADC data to usable parameters

/**
  * @brief:  ADC1 Hardware Abstraction Layer task
  * @param:  _taskADC -> cast to a void pointer
  * @retval: None (void output)
  */
void vADC1DeviceHAL(void const * pvParameters) {
    _taskADC1 pxParameters;
    pxParameters = * (_taskADC1 *) pvParameters;
    // pxParameters is to include the parameters required to configure and interface this task
    // with other tasks within the OS - see header file for parameters (config, input, output).
/*---------------------------[     Configure the Internal ADC     ]---------------------------*/
    ADCDMARecord        ADCForm[ADC1_FormBuffer]        = { 0 };  // ADC1 Form initialised

    GenBuffer<ADCDMARecord> ADC1GenBuff(&ADCForm[0], ADC1_FormBuffer);
        // Link ADC Form to GenBuffer

    ADC1Buff_Handle = &ADC1GenBuff;                     // Link ADCGenBuff to global pointer
    ADC1_Handle     = &hadc1;  // Link ADC input to global pointer
    ADC1DMA_Handle  = &hdma_adc1;     // Link ADC linked DMA to global pointer
                                                        // all for ISR

    uint32_t calibration = 0;           // Variable for the calibrated return value

    // Start calibration run for Single Ended signals (only these are used within miStepper)
    HAL_ADCEx_Calibration_Start(ADC1_Handle, ADC_SINGLE_ENDED);

    // Retrieve the read calibration value
    calibration = HAL_ADCEx_Calibration_GetValue(ADC1_Handle,
                                                 ADC_SINGLE_ENDED);

    // Now put into Calibration register of ADC (Single Ended)
    HAL_ADCEx_Calibration_SetValue(ADC1_Handle,
                                   ADC_SINGLE_ENDED, calibration);


    // Link the DMA ADC channel to ADC 1 Device, so as to capture all the conversions triggered
    __HAL_DMA_DISABLE(ADC1DMA_Handle);      // Ensure that the DMA channel is disabled
    HAL_DMA_Start(ADC1DMA_Handle,                       // Linked DMA for ADC
                  (uint32_t)&ADC1_Handle->Instance->DR, // ADC Data Register
                  (uint32_t)&CurADC1Record.Analog[0],   // Pointer to global "Current" ADC Record
                                                        // which always contains the live value(s)
                  ADC1_NumSeq * ADC1_ConvPSeq);         // Size of record
                    // Contains ADC1_ConvPSeq number of conversions per sequence
                    //          ADC1_NumSeq sequences in record

    SET_BIT(ADC1_Handle->Instance->CFGR,     //
            ADC_CFGR_DMAEN);                 // Link ADC1 to the DMA
    SET_BIT(ADC1_Handle->Instance->CFGR,     //
            ADC_CFGR_DMACFG);                // Enable circular conversions

    // As the DMA is now linked to the ADC. It means that any overruns that occur will result in
    // the ADC hardware no longer requesting any DMA requests.
    // Therefore OVR interrupt needs to be enabled, as well as conversion interrupt
    __HAL_ADC_ENABLE_IT(ADC1_Handle, ADC_IT_OVR);
    __HAL_ADC_ENABLE_IT(ADC1_Handle, ADC_IT_EOS);

    // Enable the DMA Error interrupt flag
    __HAL_DMA_ENABLE_IT(ADC1DMA_Handle, DMA_IT_TE);

    // The strategy for fault accommodation is as follows:
    //  A OVRRun fault, or DMA fault
    //      DMA is disabled, and any ADC conversions are ignored (data to be considered faulty)
    //      Once the correct number of ADC conversions have completed then:
    //          Any existing ADC faults will be cleared
    //          DMA will be restarted, and enabled

    ADC_Enable(ADC1_Handle);     // Enable the ADC

    // Enable Timer for ADC conversions
    __HAL_TIM_CLEAR_FLAG(&htim6,    // Ensure that update flag is already
                         TIM_FLAG_UPDATE);                  // cleared
    __HAL_TIM_ENABLE(&htim6);       // Then enable timer

    LL_ADC_REG_StartConversion(ADC1_Handle->Instance);      // Start conversions!

    // Create local version of linked signals (prefix with "lc")
    // #=======================================================#
    float TempFloatArray[ADC1_ConvPSeq][ADC1_NumSeq + 1] = { 0 };
    // Temporary array which will contain the 32bit float version of ADC conversion data
    // [ TimeSlice ] [ SpecificData ]

    ADCDMARecord    TempRec = { 0 };        // Temporary record

    _HALParam   lcParams[ADC1_ConvPSeq];    // Array to contain the local versions of all outputs
                                            // from this task

    for (calibration = 0; calibration != ADC1_ConvPSeq; calibration++) {
            // Cycle through all parameters
        lcParams[calibration].data  = -999.0;           // Initialise signal with default value
        lcParams[calibration].flt   = _HALParam::Faulty;// Indicate data as fault.
    }

    /* Following contains the main aspects of this task-------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t FirstPass = 1;      // Indicate that this is the first time going through this task

    uint32_t PreviousWakeTime = osKernelSysTick();  // Capture start time of task within Kernel
                                                    // time
    /* Infinite loop */
    for(;;) {
        ticStartTask(miADC1__Task);     // Capture time of start of task

        while (ADC1GenBuff.State() != GenBuffer_Empty) {    // Cycle through ADC records which are
                                                            // populated
            ADC1GenBuff.OutputRead( &TempRec );             // Read new record, and copy into
                                                            // 'TempRec'

            if ((TempRec.Flt == ADCDMARecord::Fault) || (FirstPass == 1)) {
                // If there was a conversion problem, OR it has been the first pass through of task
                // then
                for (calibration = 0; calibration != ADC1_ConvPSeq; calibration++) {
                    // Loop through all output parameters
                    lcParams[calibration].flt   = _HALParam::Faulty;    // Indicate data is faulty
                }
            }
            else
            {   // Otherwise data is good, and this is not the first pass
                ADC1_InternalVoltage(&TempRec,  &TempFloatArray[VRefLoc][1]);
                    // Convert ADC data to get the Voltage Reference

                ADC1_InternalTemperature(&TempRec, &TempFloatArray[ITmpLoc][1],
                                                   &TempFloatArray[VRefLoc][1]);
                    // Calculate the internal temperature reading

                ADC1_FANMotorVoltage(&TempRec, &TempFloatArray[FANVLoc][1],
                                               &TempFloatArray[VRefLoc][1]);
                ADC1_FANMotorCurrent(&TempRec, &TempFloatArray[FANILoc][1],
                                               &TempFloatArray[VRefLoc][1]);
                    // Calculate the FAN Motor Voltage and Current

                ADC1_STPMotorVoltage(&TempRec, &TempFloatArray[STPVLoc][1],
                                               &TempFloatArray[VRefLoc][1]);
                ADC1_STPMotorCurrent(&TempRec, &TempFloatArray[STPILoc][1],
                                               &TempFloatArray[VRefLoc][1]);
                    // Calculate the FAN Motor Voltage and Current

                for (calibration = 0; calibration != ADC1_ConvPSeq; calibration++) {
                    ADC1_AverageParameter(&TempFloatArray[calibration][0]); // Calculate Average

                    // Loop through all output parameters
                    lcParams[calibration].data  = TempFloatArray[calibration][0];
                        // Link '_HALParam' .data value to calculated average value
                    lcParams[calibration].flt   = _HALParam::NoFault;
                        // Indicate data is fault free
                }
            }
        }

        // Link internal signals to output pointers:
        *(pxParameters.output.IntVrf)       = lcParams[VRefLoc];
        *(pxParameters.output.IntTmp)       = lcParams[ITmpLoc];

        *(pxParameters.output.FanVlt)       = lcParams[FANVLoc];
        *(pxParameters.output.FanCur)       = lcParams[FANILoc];

        *(pxParameters.output.StpVlt)       = lcParams[STPVLoc];
        *(pxParameters.output.StpCur)       = lcParams[STPILoc];


        FirstPass = 0;              // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        tocStopTask(miADC1__Task);  // Capture time task completed
        osDelayUntil(&PreviousWakeTime, ADC___DEV_Time);
        // Put the task in "DELAYED" state for the defined period
        // see - miStepperCONFIG.h for the timings
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
    if ( (__HAL_ADC_GET_FLAG(ADC1_Handle, ADC_FLAG_EOS) != 0) &&
         (__HAL_ADC_GET_IT_SOURCE(ADC1_Handle, ADC_IT_EOS) != 0) ) {
        // If complete sequence interrupt triggered, then clear:
        __HAL_ADC_CLEAR_FLAG(ADC1_Handle, ADC_FLAG_EOS);    // Clear End of Sequence (EOS) flag

        ADC1SequCount++;    // Increment the ADC sequence count.
        if (ADC1SequCount >= ADC1_NumSeq) {     // If sequence count is equal/greater than
                                                // number of runs in record.
            ADC1SequCount = 0;                  // Reset counter

            ADC1Buff_Handle->InputWrite(CurADC1Record); // Add the current record into buffer

            if (CurADC1Record.Flt != ADCDMARecord::NoFault) {
                // If record is faulty, then DMA has been disabled. Therefore needs to be re-setup
                // and enabled

                HAL_DMA_Start(ADC1DMA_Handle,                       // Linked DMA for ADC
                              (uint32_t)&ADC1_Handle->Instance->DR, // ADC Data Register
                              (uint32_t)&CurADC1Record.Analog[0],   // Pointer to global "Current"
                                                                    // ADC Record which always
                                                                    // contains the live value(s)
                              ADC1_NumSeq * ADC1_ConvPSeq);         // Size of record
                            // Contains ADC1_ConvPSeq number of conversions per sequence
                            //          ADC1_NumSeq sequences in record
            }

            CurADC1Record.Flt    = ADCDMARecord::NoFault;   // Clear any faults on current record
            // DMA is in circular mode, so will continue to run
        }
    }

    // Check to see if the interrupt was due to overrun:
    if ( (__HAL_ADC_GET_FLAG(ADC1_Handle, ADC_FLAG_OVR) != 0) &&
         (__HAL_ADC_GET_IT_SOURCE(ADC1_Handle, ADC_IT_OVR) != 0) ) {
        __HAL_ADC_CLEAR_FLAG(ADC1_Handle, ADC_FLAG_OVR);    // Clear Overrun (OVR) flag

        __HAL_DMA_DISABLE(ADC1DMA_Handle);                  // Disable the linked DMA to ADC
        CurADC1Record.Flt        = ADCDMARecord::Fault;     // Set record as faulty
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
    if ( (__HAL_DMA_GET_IT_SOURCE(ADC1DMA_Handle, DMA_IT_TE) != 0) &&
         (__HAL_DMA_GET_FLAG(ADC1DMA_Handle, __HAL_DMA_GET_TE_FLAG_INDEX(ADC1DMA_Handle)) != 0) ) {
        __HAL_DMA_CLEAR_FLAG(ADC1DMA_Handle, DMA_FLAG_TE1); // Clear the interrupt (Channel 1)

        __HAL_DMA_DISABLE(ADC1DMA_Handle);                  // Disable the linked DMA to ADC
        CurADC1Record.Flt        = ADCDMARecord::Fault;     // Set record as faulty

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
  * @brief:  Calculates the Internal voltage reference of STM32 device (this needs to be
  *          determined first for other conversions
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @param:  float pointer to the array to contain the Internal Voltage Reference value
  * @param:  float pointer to the array containing VRef converted values
  * @retval: None (void output)
  */
void ADC1_InternalVoltage(ADCDMARecord *Record, float *IntVRef) {
    uint8_t i = 0;          // Variable to loop through number of sequences run
    uint8_t dataloc = 0;    // Location within array for V

    for (i = 0; i != ADC1_NumSeq; i++) {            // Loop through copies of data within record
        dataloc = (ADC1_ConvPSeq * i) + VRefLoc;    // Calculate position of data within array

        if (Record->Analog[dataloc] == 0)           // If the data is 0
            IntVRef[i]  = 0.00L;                    // Set Vref to 0, to protect against divide by
                                                    // 0
        else
            IntVRef[i]   = 3.0L * (VREFINT / ((float) Record->Analog[dataloc]));
            // Convert the ADC entry to voltage reference
    }
}

/**
  * @brief:  Calculates the Temperature reading internal to STM32 device
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @param:  float pointer to the array to contain the Internal Temperature value
  * @param:  float pointer to the array containing VRef converted values
  * @retval: None (void output)
  */
void ADC1_InternalTemperature(ADCDMARecord *Record, float *IntTmp, float *IntVRef) {
    uint8_t i = 0;          // Variable to loop through number of sequences run
    uint8_t dataloc = 0;    // Location within array for V

    // Following variables are used to convert internal calibration registers to voltages (V) in
    // float format.
    float CAL1Volt = ( 3.0L   *   TS_CAL1 ) / ADC_Resolution;
    float CAL2Volt = ( 3.0L   *   TS_CAL2 ) / ADC_Resolution;

    for (i = 0; i != ADC1_NumSeq; i++) {            // Loop through copies of data within record
        dataloc = (ADC1_ConvPSeq * i) + ITmpLoc;    // Calculate position of data within array

        IntTmp[i]   = ( IntVRef[i] * (float)Record->Analog[dataloc] ) / ADC_Resolution;
            // Get the current voltage from Temperature sensor
        IntTmp[i]   -=  CAL1Volt;   // Account for first calibration voltage
        IntTmp[i]   *=  (TSCAL2Tmp  -  TSCAL1Tmp) / (CAL2Volt  -  CAL1Volt);
            // Multiply value by conversion slope
        IntTmp[i]   +=  TSCAL1Tmp;  // Again offset based upon first calibration temperature
    }
}

/**
  * @brief:  Calculates the Fan Voltage being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @param:  float pointer to the array to contain the Fan Voltage value
  * @param:  float pointer to the array containing VRef converted values
  * @retval: None (void output)
  */
void ADC1_FANMotorVoltage(ADCDMARecord *Record, float *FanVlt, float *IntVRef) {
    uint8_t i = 0;          // Variable to loop through number of sequences run
    uint8_t dataloc = 0;    // Location within array for V

    for (i = 0; i != ADC1_NumSeq; i++) {            // Loop through copies of data within record
        dataloc = (ADC1_ConvPSeq * i) + FANVLoc;    // Calculate position of data within array

        FanVlt[i] = ( IntVRef[i] * (float)Record->Analog[dataloc] ) / ADC_Resolution;
        FanVlt[i] *= (  (FAN_RUpper + FAN_RLower)  /  FAN_RLower  );
        // Convert the ADC entry to Fan Voltage
    }
}

/**
  * @brief:  Calculates the Stepper Voltage being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @param:  float pointer to the array to contain the Stepper Voltage value
  * @param:  float pointer to the array containing VRef converted values
  * @retval: None (void output)
  */
void ADC1_STPMotorVoltage(ADCDMARecord *Record, float *StpVlt, float *IntVRef) {
    uint8_t i = 0;          // Variable to loop through number of sequences run
    uint8_t dataloc = 0;    // Location within array for V

    for (i = 0; i != ADC1_NumSeq; i++) {            // Loop through copies of data within record
        dataloc = (ADC1_ConvPSeq * i) + STPVLoc;    // Calculate position of data within array

        StpVlt[i] = ( IntVRef[i] * (float)Record->Analog[dataloc] ) / ADC_Resolution;
        StpVlt[i] *= (  (STP_RUpper + STP_RLower)  /  STP_RLower  );
        // Convert the ADC entry to Stepper Voltage
    }
}

/**
  * @brief:  Calculates the Fan Current being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @param:  float pointer to the array to contain the Fan Current value
  * @param:  float pointer to the array containing VRef converted values
  * @retval: None (void output)
  */
void ADC1_FANMotorCurrent(ADCDMARecord *Record, float *FanCur, float *IntVRef) {
    uint8_t i = 0;          // Variable to loop through number of sequences run
    uint8_t dataloc = 0;    // Location within array for V

    for (i = 0; i != ADC1_NumSeq; i++) {            // Loop through copies of data within record
        dataloc = (ADC1_ConvPSeq * i) + FANILoc;    // Calculate position of data within array

        FanCur[i] = ( IntVRef[i] * (float)Record->Analog[dataloc] ) / ADC_Resolution;
        FanCur[i] *= (  1  / (FAN_CS30 * FAN_Rsense)  );
        // Convert the ADC entry to Fan Current
    }
}

/**
  * @brief:  Calculates the Stepper Current being drawn
  * @param:  ADCDMARecord pointer for the entire ADC record run
  * @param:  float pointer to the array to contain the Stepper Current value
  * @param:  float pointer to the array containing VRef converted values
  * @retval: None (void output)
  */
void ADC1_STPMotorCurrent(ADCDMARecord *Record, float *StpCur, float *IntVRef) {
    uint8_t i = 0;          // Variable to loop through number of sequences run
    uint8_t dataloc = 0;    // Location within array for V

    for (i = 0; i != ADC1_NumSeq; i++) {            // Loop through copies of data within record
        dataloc = (ADC1_ConvPSeq * i) + STPILoc;    // Calculate position of data within array

        StpCur[i] = ( IntVRef[i] * (float)Record->Analog[dataloc] ) / ADC_Resolution;
        StpCur[i] *= (  1  / (STP_CS30 * STP_Rsense)  );
        // Convert the ADC entry to Stepper Current
    }
}

/**
  * @brief:  Calculates the average of the parameter array, and puts output into first entry.
  *          Starts the averaging from the second entry.
  * @retval: None (void output)
  */
void ADC1_AverageParameter(float *param) {
    uint8_t i = 0;          // Variable to loop through number of sequences run

    param[0] = param[1];    // Initialise the  average output to the first ready parameter
                            // (contained within the second array point)

    for (i = 1; i != ADC1_NumSeq; i++) {    // Loop through copies of data within record
        param[0] += param[i + 1];           // Add the next parameter to accumulator
    }

    param[0] = param[0] / ((float) ADC1_NumSeq);    // Divide accumulation by number of parameters
                                                    // so as to get the average
}
