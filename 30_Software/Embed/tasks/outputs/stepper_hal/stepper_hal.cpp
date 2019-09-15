/**************************************************************************************************
 * @file        stepper_hal.cpp
 * @author      Thomas
 * @version     V1.1
 * @date        25 Jun 2019
 * @brief       Source file for Stepper Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/

#include "EmbedIndex.h"
#include EMBD_STPTask

/**************************************************************************************************
 * Define any local global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
static Stepper      *Stepper_Handle;    // Pointer to the Stepper class handle, for interrupt use

/**************************************************************************************************
 * Define any local functions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**
  * @brief:  Stepper Motor Hardware Abstraction Layer task
  * @param:  _taskSTP -> cast to a void pointer
  * @retval: None (void output)
  */
void vSTPMotorHAL(void const * pvParameters) {
    _taskSTP pxParameters;
    pxParameters = * (_taskSTP *) pvParameters;
    // pxParameters is to include the parameters required to configure and interface this task
    // with other tasks within the OS - see header file for parameters (config, input, output).
/*---------------------------[     Setup Stepper GPIO handles     ]---------------------------*/
    // Generate the GPIO pins outside of 'Stepper' handle scope
    // ========================================================
    GPIO    STP_ENABLE( A4988_NENABLE_GPIO_Port,    A4988_NENABLE_Pin,  GPIO::OUTPUT);
    GPIO    STP_SLEEP(  A4988_NSLEEP_GPIO_Port,     A4988_NSLEEP_Pin,   GPIO::OUTPUT);
    GPIO    STP_DIR(    A4988_DIR_GPIO_Port,        A4988_DIR_Pin,      GPIO::OUTPUT);
    GPIO    STP_RESET(  A4988_NRESET_GPIO_Port,     A4988_NRESET_Pin,   GPIO::OUTPUT);
    GPIO    STP_MS[3]={{A4988_MS1_GPIO_Port,        A4988_MS1_Pin,      GPIO::OUTPUT},
                       {A4988_MS2_GPIO_Port,        A4988_MS2_Pin,      GPIO::OUTPUT},
                       {A4988_MS3_GPIO_Port,        A4988_MS3_Pin,      GPIO::OUTPUT} };

    // Disable the Stepper IC, by pulling ENABLE/SLEEP high, and RESET low:
    STP_ENABLE.setValue(GPIO::State::HIGH);     // Set Enable pin to HIGH '1'   (Device Disabled)
    STP_SLEEP.setValue(GPIO::State::HIGH);      // Set Sleep  pin to HIGH '1'   (Device Awake)
    STP_RESET.setValue(GPIO::State::LOW);       // Set Reset  pin to LOW  '0'   (Reset)

    // Create Hardware setup variable used to configure the 'Stepper' handler:
    Stepper::HrdSetup  STPConfig;

    /* Setup the 'Stepper' handler so that
     *      [1] The linked DMA used to generate the STEP 'pulse', which is Capture/Compare 3
     *      [2] Provide the BIT position for enabling/disabling in 'DIER' register for TIM linkage
     *          to DMA
     *      [3] Provide the BIT position within TIM 'EGR' to generate a 'Capture/Compare 3' event
     *      [4] Provide the BIT position within TIM 'SR' to check for a 'Capture/Compare 3' event
     *      ##NOTE-> [3] and [4] do NOT result in an interrupt, as linked to DMA
     *
     *      [5] Provide the BIT position within TIM 'EGR' to generate a 'Capture/Compare 2' event
     *      [6] Provide the BIT position within TIM 'SR' to check for a 'Capture/Compare 2' event
     *      [7] Provide the BIT position within TIM 'DIER' to enable/disable interrupt of a
     *          'Capture/Compare 2' event
     *      ##NOTE-> [5..7] Result in an interrupt being triggered, which is used to count the
     *                      number of STEPs that has been taken
     *********************************************************************************************/

    STPConfig.PulseDMAAddress   = (uint32_t) &pxParameters.config.stepper_timer->Instance->CCR3;
                                                                                            //[1]
    STPConfig.EnbTIMDMA        = TIM_DMA_CC3;                                               //[2]
    STPConfig.PulsEGRBit       = TIM_EVENTSOURCE_CC3;                                       //[3]
    STPConfig.PulsSRBit        = TIM_FLAG_CC3;                                              //[4]

    STPConfig.CounEGRBit       = TIM_EVENTSOURCE_CC2;                                       //[5]
    STPConfig.CounSRBit        = TIM_FLAG_CC2;                                              //[6]
    STPConfig.CounIntBit       = TIM_IT_CC2;                                                //[7]

    Stepper NEMA(pxParameters.config.stepper_timer, pxParameters.config.stepper_dma,
                // Generate the output signal for STEP 'pulse'
            ( TIM_CCxN_ENABLE << TIM_CHANNEL_3 ),   // The output channel pin for enabling the
                                                    // output for 'Capture/Compare 3'
            TIM_CHANNEL_2,                  // The Capture/Compare channel for the STEP counter
                                            // interrupt
            // Linked GPIO pins - Reset, Direction and Microstep
            &STP_RESET, &STP_DIR, &STP_MS[0], 3,
            200 * 16, STPConfig
            );
    Stepper_Handle  = &NEMA;        // Link NEMA class to global pointer (for ISR)

/*---------------------------[  Stepper is now ready for control  ]---------------------------*/

    // Create local version of linked signals (prefix with "lc")
    // #=======================================================#
    uint8_t     lcStpEnable;    // Local copy of input enabling flag
    uint8_t     lcStpGear;      // Local copy of input Stepper gear selection
    uint8_t     lcStpDirct;     // Local copy of input Stepper direction
    uint16_t    lcStpFreqDmd;   // Local copy of input Stepper Frequency

    // Output to this task
    // #=================#
    uint16_t    StpStatAct  = 0;// Output register for Stepper State
    uint16_t    StpFreqAct  = 0;// Output value for Actual Stepper Frequency

    uint8_t     StpCurDir   = 0;// Captured selected direction of motor
    uint8_t     StpCurGer   = 0;// Captured selected gearing of motor

    uint32_t    PolCountVal = 0;// Set value for internal calculation of position based on STEP
                                // count


    Stepper::CountPanel::Polarity   MtrPol = Stepper::CountPanel::Polarity::UP;
    GPIO::State   MtrDir = GPIO::State::LOW;    // Variable used to store the direction of the
                                                // motor


    /* Following contains the main aspects of this task-------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t FirstPass = 1;      // Indicate that this is the first time going through this task

    uint32_t PreviousWakeTime = osKernelSysTick();  // Capture start time of task within Kernel
                                                    // time
    /* Infinite loop */
    for(;;) {
        ticStartTask(miSteperTask);     // Capture time of start of task

        // This task is going to be running quite slow, just so as to determine that interfaces
        // are all working within the FreeRTOS environment.
        // So will read in the signals -
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Capture any new updates to input signals and link to local internals
        lcStpEnable     = *(pxParameters.input.StpEnable);
        lcStpGear       = *(pxParameters.input.StpGear);
        lcStpDirct      = *(pxParameters.input.StpDirct);
        lcStpFreqDmd    = *(pxParameters.input.StpFreqDmd);

        // Ensure that all provided data is within acceptable limits:
        if (lcStpGear > STP_MaxGear) {      // If gear selection is greater than amount of gears
            lcStpGear = STP_MaxGear;        // Limit to maximum gear
        }

        if (lcStpFreqDmd > STP_LowFrq) {        // If demand is set at slower than can be managed
            lcStpFreqDmd    = 0;                // default to "0" - OFF
        } else if (lcStpFreqDmd == 0) {         // If demand is zero, do nothing

        } else if (lcStpFreqDmd < STP_MaxFrq) { // If demand is set at faster than can be managed
            lcStpFreqDmd    = STP_MaxFrq;       // Saturate at maximum speed
        }


        // Check if the request to ENABLE the stepper has been made (i.e. StpEnable is not zero)
        if (lcStpEnable != 0) {                     // If motor has been enabled
            STP_ENABLE.setValue(GPIO::State::LOW);  // Set the ENABLE pin to LOW (enable)
            StpStatAct  |= (1 << STP_StateEnable);  // Setup Stepper Status to indicate enabled

            // First need to check to see if the requested demand is "0", i.e. off, and that this
            // has NOT been captured yet
            if (lcStpFreqDmd   == 0) {  // If new demand is for no motion
                if (StpFreqAct != 0) {  // and this has NOT been captured yet
                    StpFreqAct  = 0;    // Ensure the stationary request is captured (so not run
                                        // again)
                    StpCurGer   = 0;    // Capture gear is zero
                    StpCurDir   = 0;    // Capture Direction is zero

                    if (NEMA.ShdForm.cMode != Stepper::Mode::Disabled) {
                        // If the motor is in motion, then force the motor to stop
                        NEMA.forceSTOP();
                    }
            } }
            else {
                // Now request is for any other speed (and has been limited to acceptable
                // parameters)
                // Check to see if any of the parameters have actually changed, i.e. frequency,
                // gear ratio, or direction
                if ( (lcStpFreqDmd != StpFreqAct)  ||  (lcStpGear    != StpCurGer)   ||
                     (lcStpDirct   != StpCurDir) )  {

                    if (lcStpDirct == 0) {          // If direction is "0"
                        MtrDir = GPIO::State::HIGH; // Set DIR pin HIGH
                        MtrPol = Stepper::CountPanel::Polarity::UP;     // Set polarity to "UP"
                    } else {                        // Otherwise
                        MtrDir = GPIO::State::LOW;  // Set DIR pin LOW
                        MtrPol = Stepper::CountPanel::Polarity::DOWN;   // Set polarity to "DOWN"
                    }

                    // Based upon the selected Gear, select the desired STEP count
                    if          (lcStpGear == 0) {      // If at "0" gear
                        PolCountVal     =   16;         // Increment by 16 counts (1    full step)
                    } else if   (lcStpGear == 1) {      // If at "1" gear
                        PolCountVal     =    8;         // Increment by  8 counts (1/2  full step)
                    } else if   (lcStpGear == 2) {      // If at "2" gear
                        PolCountVal     =    4;         // Increment by  4 counts (1/4  full step)
                    } else if   (lcStpGear == 3) {      // If at "3" gear
                        PolCountVal     =    2;         // Increment by  2 counts (1/8  full step)
                    //-----------------------------------------------------------------------------
                    // If selected gear is between 3 and 7, then force the gear to be 3
                    } else if   (lcStpGear == 4) {
                        lcStpGear       =    3;
                        PolCountVal     =    2;
                    } else if   (lcStpGear == 5) {
                        lcStpGear       =    3;
                        PolCountVal     =    2;
                    } else if   (lcStpGear == 6) {
                        lcStpGear       =    3;
                        PolCountVal     =    2;
                    //-----------------------------------------------------------------------------
                    } else if   (lcStpGear == 7) {      // If at "7" gear
                        PolCountVal     =    1;         // Increment by  1 counts (1/16 full step)
                    }

                    // Put new speed into buffer
                    NEMA.newVelocity(MtrDir, lcStpGear, lcStpFreqDmd, MtrPol, PolCountVal);

                    // Now capture new values:
                    StpFreqAct  = lcStpFreqDmd;
                    StpCurGer   = lcStpGear;
                    StpCurDir   = lcStpDirct;
                }
            }
        } else {        // If the motor has not been enabled then
            if (NEMA.ShdForm.cMode != Stepper::Mode::Disabled) {    // If the motor is in motion
                NEMA.forceSTOP();           // Force stop of the motor
            }
            // Otherwise motor is stationary
            STP_ENABLE.setValue(GPIO::State::HIGH); // Set the ENABLE pin to HIGH (disabled)

            StpFreqAct  = 0;    // Capture that the motor is now stationary
            StpCurGer   = 0;    // Capture gear is zero
            StpCurDir   = 0;    // Capture Direction is zero
            StpStatAct  = 0;    // Setup Status to indicate all off
        }

        if (StpCurDir == 0) {                       // If direction is set to "0"
            StpStatAct  &= ~(1 << STP_DirectionFl); // Clear bit in register for Direction
        } else {                                    // Otherwise
            StpStatAct  |= (1 << STP_DirectionFl);  // Set bit in register for Direction
        }

        StpStatAct      &= ~(STP_MaxGear << STP_GearStart); // Clear position(s) for gearing
        StpStatAct      |= (StpCurGer   << STP_GearStart);  // Now set with current gearing

        // Link internal signals to output pointers:
        *(pxParameters.output.StpFreqAct)   = StpFreqAct;
        *(pxParameters.output.StpStatAct)   = StpStatAct;
        *(pxParameters.output.StpcalPost)   = NEMA.calcPos;


        FirstPass = 0;              // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        tocStopTask(miSteperTask);  // Capture time task completed
        osDelayUntil(&PreviousWakeTime, STP___HAL_Time);
        // Put the task in "DELAYED" state for the defined period
        // see - miStepperCONFIG.h for the timings
    }
}

/**
  * @brief:  Timer 1 Event or Timer 16 Interrupt Service Routine handler
  * @param:  None (void input)
  * @retval: None (void output)
  */
void TIM1_UP_TIM16_IRQHandler(void) {  Stepper_Handle->IRQUPHandler();         }

/**
  * @brief:  Timer 1 Capture/Compare Interrupt Service Routine handler
  * @param:  None (void input)
  * @retval: None (void output)
  */
void TIM1_CC_IRQHandler(void)       {  Stepper_Handle->IRQCounterCCHandler();  }

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
// None
