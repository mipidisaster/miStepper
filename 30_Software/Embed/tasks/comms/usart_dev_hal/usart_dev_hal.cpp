/**************************************************************************************************
 * @file        usart_dev_hal.cpp
 * @author      Thomas
 * @version     V0.1
 * @date        09 Mar 2019
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


/**************************************************************************************************
 * Define any local functions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
void WriteDatatoScreen(UARTPeriph *husart, char *pstorage, _HALParam *datapoint, char *xtrText,
                                                                                 char unit);
void WriteTimeToScreen(UARTPeriph *husart, char *pstorage, uint8_t curTask,
                                           char *durText, char *delText);
// Prototype(s) for function which puts data and string onto screen

void QNewLine(UARTPeriph *husart);
// Prototype for requesting a new line, and cursor to be returned to start of line


/**
  * @brief:  USART Device Hardware Abstraction Layer task
  * @param:  _taskUSART -> cast to a void pointer
  * @retval: None (void output)
  */
void vUSARTDeviceHAL(void const * pvParameters) {
    _taskUSART pxParameters;
    pxParameters = * (_taskUSART *) pvParameters;
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
    UARTPeriph  USART1Dev(pxParameters.config.dev_handle, &USARTGenBuff[1], &USARTGenBuff[0]);
    USART1_Handle = &USART1Dev;     // Link USART1Dev class to global pointer (for ISR)

    // Create local version of linked signals (prefix with "lc")
    // #=======================================================#
    _HALParam   lcAngPos;           // Local version of value of Angle Position
    _HALParam   lcExtTmp;           // Local version of value of External device temperature

    _HALParam   lcIntVrf;           // Local version of Voltage Reference
    _HALParam   lcIntTmp;           // Local version of Internal Temperature

    _HALParam   lcFanVlt;           // Local version of Fan Motor Voltage
    _HALParam   lcFanCur;           // Local version of Fan Motor Current
    _HALParam   lcFanAct;           // Local version of Fan Motor Actual Demand
    _HALParam   lcFanDmd;           // Local version of Fan Motor Demand

    _HALParam   lcStpVlt;           // Local version of Stepper Motor Voltage
    _HALParam   lcStpCur;           // Local version of Stepper Motor Current
    _HALParam   movement;           // Local version of Stepper Movement

    uint8_t     lcStpEnableReq;     // Request the motor to be enabled
    uint8_t     lcStpMoveReq;       // Request the motor to move

    // Parameters used to contain ASCII characters to be displayed via USART
    char textarray[MaxCharinLine] = { 0 };  // This array will be passed into lower functions to
                                            // limit the impact on stack size
    int USARTsize = 0;                      // Variable returned from 'snprintf' for size of
                                            // package
    int looper = 0;                         // Variable to loop through the characters in
                                            // 'textarray'
    i = 0;      // Re-config the counter variable to be set to "0", as this is displayed to show
                // USART task loop count

    uint8_t  HMIString = 0;
        // Variable used to enable/disable USART communication, Fan motor, stepper motor

    uint8_t readback = 0;       // Variable used to store any characters returned from USART


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

        while (USART1Dev.SingleRead_IT(&readback) != GenBuffer_Empty) {
            if      ( (readback == 'D') || (readback == 'd') ) {
                HMIString   ^= (1 << TransmitData);
            }
            else if ( (readback == 'F') || (readback == 'f') ) {
                HMIString   ^= (1 << FanEnable);
            }
            else if ( (readback == 'E') || (readback == 'e') ) {
                HMIString   ^= (1 << StpEnable);
            }
            else if ( (readback == 'M') || (readback == 'm') ) {
                HMIString   ^= (1 << StpMove);
            }
        }

        // Capture any new updates to input signals and link to local internals
        lcAngPos    = *(pxParameters.input.AngPos);
        lcExtTmp    = *(pxParameters.input.ExtTmp);

        lcIntVrf    = *(pxParameters.input.IntVrf);
        lcIntTmp    = *(pxParameters.input.IntTmp);

        lcFanVlt    = *(pxParameters.input.FanVlt);
        lcFanCur    = *(pxParameters.input.FanCur);
        lcFanAct    = *(pxParameters.input.FanAct);

        lcStpVlt    = *(pxParameters.input.StpVlt);
        lcStpCur    = *(pxParameters.input.StpCur);
        movement    = *(pxParameters.input.movement);

        if ((HMIString & (1 << TransmitData)) != 0) {
            // Only if the transmit bit has been enabled, send the following values to the display
            // Current loop count (of USART only)
            // Values of Angular Position, Internal and External Temperature, Voltage Reference,
            //           Fan Voltage, Current and demand, Stepper Motor Voltage Current
            //           Then (after first pass of task) transmit each task time duration and
            //           delay
            USART1Dev.SingleTransmit_IT( 12 );   // Clear the screen

            USARTsize = snprintf(&textarray[0], MaxCharinLine,
                                 "Currently on count -> %10d", (int) i);

            for (looper = 0; looper != USARTsize; looper++) {
                    USART1Dev.SingleTransmit_IT( (uint8_t) textarray[looper] );
            }

            QNewLine(&USART1Dev);
            QNewLine(&USART1Dev);


            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcAngPos, (char*)"Angle  ->", 'r');
            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcExtTmp, (char*)"E Temp ->", 'C');
            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcIntTmp, (char*)"I Temp ->", 'C');
            QNewLine(&USART1Dev);
            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcIntVrf, (char*)"Vref   ->", 'V');
            QNewLine(&USART1Dev);
            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcFanVlt, (char*)"Fan V  ->", 'V');
            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcFanCur, (char*)"Fan I  ->", 'A');
            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcFanAct, (char*)"Fan Act->", '%');
            QNewLine(&USART1Dev);
            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcStpVlt, (char*)"Stp V  ->", 'V');
            WriteDatatoScreen(&USART1Dev, &textarray[0], &lcStpCur, (char*)"Stp I  ->", 'A');
            WriteDatatoScreen(&USART1Dev, &textarray[0], &movement, (char*)"Stp Act->", '%');

            QNewLine(&USART1Dev);
            QNewLine(&USART1Dev);

            if (FirstPass != 1) {
                WriteTimeToScreen(&USART1Dev, &textarray[0], miUSART1Task,
                                                             (char *) "USART Task  ->",
                                                             (char *) "            ->");
                WriteTimeToScreen(&USART1Dev, &textarray[0], miSPI1__Task,
                                                             (char *) "SPI1  Task  ->",
                                                             (char *) "            ->");
                WriteTimeToScreen(&USART1Dev, &textarray[0], miI2C1__Task,
                                                             (char *) "I2C1  Task  ->",
                                                             (char *) "            ->");
                WriteTimeToScreen(&USART1Dev, &textarray[0], miADC1__Task,
                                                             (char *) "ADC1  Task  ->",
                                                             (char *) "            ->");
                WriteTimeToScreen(&USART1Dev, &textarray[0], miFan___Task,
                                                             (char *) "Fan   Task  ->",
                                                             (char *) "            ->");
                WriteTimeToScreen(&USART1Dev, &textarray[0], miSteperTask,
                                                             (char *) "Stp   Task  ->",
                                                             (char *) "            ->");
            }

            // If the Fan Enable bit has been set, then demand the Fan to FULL POWER!
            if ((HMIString & (1 << FanEnable)) != 0) {  // Check for 'FanEnable' bit to be set
                USART1Dev.SingleTransmit_IT('F');       // Transmit indication of Fan enabled,
                                                        // single character 'F' at end of string
                lcFanDmd.data   = 100.0;                // Set Fan Demand to 100%
            }
            // If the bit has not been set, then ensure demand is set to 0%
            else {  lcFanDmd.data   = 0.0;  }

            // If the Stepper Enable bit has been set, then request the motor to be enabled
            if ((HMIString & (1 << StpEnable)) != 0) {  // Check for 'StpEnable' bit to be set
                USART1Dev.SingleTransmit_IT('E');       // Transmit indication of Stepper enabled,
                                                        // single character 'E' at end of string
                lcStpEnableReq    = 1;                  // Set request to 1
            }
            // If the bit has not been set, then ensure demand is set to disabled
            else {  lcStpEnableReq    = 0;  }

            // If the stepper movement bit has been set, then request the motor to move
            if ((HMIString & (1 << StpMove)) != 0) {    // Check for 'StpMove' bit to be set
                USART1Dev.SingleTransmit_IT('M');       // Transmit indication of Stepper movement
                                                        // single character 'M' at end of string
                lcStpMoveReq      = 1;                  // Set request to 1
            }
            // If the bit has not been set, then ensure demand is set to no movement
            else {  lcStpMoveReq      = 0;  }
        }
        // If transmit data has not been requested, then ensure that all demands are set to 0
        // and drive the 'HMIString' to a value of 0; so as to ensure that all requested demands
        // are reset
        else {
            lcFanDmd.data   = 0.0;      // Set Fan demand to 0%
            lcStpEnableReq    = 0;      // Ensure that all motor requests are set to OFF
            lcStpMoveReq      = 0;      //

            HMIString       = 0;        // Ensure that any motor conditions which have been
                                        // requested are now reset
        }

        i++;

        // Link internal signals to output pointers:
        *(pxParameters.output.FanDmd)   = lcFanDmd;
        *(pxParameters.output.enable)   = lcStpEnableReq;
        *(pxParameters.output.move)     = lcStpMoveReq;


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

#define     ASCIIFloatMAXchar   10      // Maximum number of characters reserved for float
                                        // conversion
                                        /*******************************************
                                         * Float number     = +  12.0123
                                         *                    ^|  |^|  |
                                         */

/**
  * @brief:  Will convert the input floating point number into ASCII array of size
  *          'ASCIIFloatMASchar'
  * @param:  char pointer to array to store ASCII text of float data
  * @param:  floating point value of data to be converted into text
  * @retval: None (void output)
  */
void ASCIIFloat(char *chararray, float value) {
    uint8_t i = 0;                                  // Loop variable declared and defined

    for (i = 0; i != ASCIIFloatMAXchar; i++)        // Cycle through data within array
        chararray[i] = '\0';                        // and set to 0 (not the value)

    // Function only works correctly on parameters which are within +/-9,999, therefore if value
    // exceeds this display limit message
    if (value < -9999.0000)                                         // If less than -9,999
        snprintf(&chararray[0], ASCIIFloatMAXchar+1, "---LIMIT##"); // Show -ve limit

    else if (value > +9999.0000)                                    // If greater than +9,999
        snprintf(&chararray[0], ASCIIFloatMAXchar+1, "+++LIMIT##"); // Show +ve limit

    else {  // Otherwise value is within range
        int decval = (int) (value * 10000L);        // Multiple float by 10,000 (to get 4 numbers
                                                    // past decimal point.
                                                    // Then convert to integer
        snprintf(&chararray[0], ASCIIFloatMAXchar+1, "%+010d", decval);
            // Convert the input decimal value into ASCII formal, filling the start with '0', and
            // including sign indication (for both '+' and '-')
        for (i = 1; i != 5; i++) {                  // Loop through the first 4 entries (ignoring
                                                    // sign)
            chararray[i] = chararray[i + 1];        // Shift it up (so have space for decimal
        }                                           // point)

        chararray[5] = '.';                         // Add decimal point at 'free' space

        for (i = 1; i != 4; i++) {                  // Again loop through first entries (ignoring
                                                    // sign) and stop short of decimal
            if (chararray[i] == '0')                // If the value is '0' (filler)
                chararray[i] = ' ';                 // turn into a space
            else                                    // If it is not '0', then its a valid number
                break;                              // exit loop + function
        }
    }
}

/**
  * @brief:  Will put onto the USART transmit queue text entries to display the time delay and
  *          duration of the task entry provided as 'curTask'
  * @param:  USART peripheral pointer
  * @param:  char pointer for storage of complete text string prior to USART queue
  * @param:  integer value for the task time data to be retrieved
  * @param:  char pointer for text to be put prior to data value for task duration
  * @param:  char pointer for text to be put prior to data value for task delay
  * @retval: None (void output)
  */
void WriteTimeToScreen(UARTPeriph *husart, char *pstorage, uint8_t curTask,
                                           char *durText, char *delText) {
    int USARTsize = 0;      // Local variable to store returned number of characters
    int looper = 0;         // Looper for ASCII text
    char floatTEXT[11];     // Create temp array for containing ASCII float conversion
    ASCIIFloat(&floatTEXT[0], miTaskDuration(curTask)); // Convert Task duration to ASCII

    USARTsize = snprintf(pstorage, MaxCharinLine, "%s   %sms", durText, floatTEXT);
        // Populate ASCII array with text string

    for (looper = 0; looper != MaxCharinLine; looper++) {   // Put data into the Transmit buffer
        if (looper > USARTsize)                             // If loop exceeds written data
            husart->SingleTransmit_IT( ' ' );               // Populate with spaces

        else    // If within the limits of 'snprintf' output, then add entry of array to transmit
                // queue
            husart->SingleTransmit_IT( (uint8_t) pstorage[looper] );
    }

    QNewLine(husart);   // Put a space between the task duration text and task period text

    ASCIIFloat(&floatTEXT[0], miTaskPeriod(curTask));   // Convert Period duration to ASCII
    USARTsize = snprintf(pstorage, MaxCharinLine, "%s   %sms", delText, floatTEXT);
        // Populate ASCII array with text string

    for (looper = 0; looper != MaxCharinLine; looper++) {   // Put data into the Transmit buffer
        if (looper > USARTsize)                             // If loop exceeds written data
            husart->SingleTransmit_IT( ' ' );               // Populate with spaces

        else    // If within the limits of 'snprintf' output, then add entry of array to transmit
                // queue
            husart->SingleTransmit_IT( (uint8_t) pstorage[looper] );
    }

    QNewLine(husart);   // Put a space between the task duration text and task period text
}

/**
  * @brief:  Will put onto the USART transmit queue text entries to display the value pointed
  *          too via "datapoint"
  * @param:  USART peripheral pointer
  * @param:  char pointer for storage of complete text string prior to USART queue
  * @param:  _HALParam pointer, which contains the data and fault state to be displayed
  * @param:  char pointer for text to be put prior to data values
  * @param:  single character to contain the unit of data point (i.e. degrees = 'C')
  * @retval: None (void output)
  */
void WriteDatatoScreen(UARTPeriph *husart, char *pstorage, _HALParam *datapoint, char *xtrText,
                                                                                 char unit) {
    char fltstate = 0;      // Local veraion to store character value for valid data

    int USARTsize = 0;      // Local variable to store returned number of characters
    int looper = 0;         // Looper for ASCII text

    if (datapoint->flt == _HALParam::NoFault) { // If data is valid
        fltstate = ' ';                         // Provide no fault status
    } else                                      // Otherwise data is invalid
        fltstate = 'X';                         // Then indicate data as bad "X"

    char floatTEXT[11];     // Create temp array for containing ASCII float conversion
    ASCIIFloat(&floatTEXT[0], datapoint->data); // Convert input data to ASCII

    USARTsize = snprintf(pstorage, MaxCharinLine, "%s   %s%c  %c", xtrText, floatTEXT,
                                                                            unit, fltstate);
        // Populate ASCII array with text string
    for (looper = 0; looper != MaxCharinLine; looper++) {   // Put data into the Transmit buffer
        if (looper > USARTsize)                             // If loop exceeds written data
            husart->SingleTransmit_IT( ' ' );               // Populate with spaces

        else    // If within the limits of 'snprintf' output, then add entry of array to transmit
                // queue
            husart->SingleTransmit_IT( (uint8_t) pstorage[looper] );
    }

    QNewLine(husart);   // Put a space between the task duration text and task period text
}

/**
  * @brief:  Will put onto the USART transmit queue a new line request, and return cursor to the
  *          next line
  * @param:  USART peripheral pointer
  * @retval: None (void output)
  */
void QNewLine(UARTPeriph *husart) {
    husart->SingleTransmit_IT( '\r' );  // Return to start
    husart->SingleTransmit_IT( '\n' );  // New line
}
