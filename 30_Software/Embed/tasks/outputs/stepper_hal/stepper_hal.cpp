/**************************************************************************************************
 * @file        stepper_hal.cpp
 * @author      Thomas
 * @version     V0.1
 * @date        09 Mar 2019
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

    STPConfig.PulseDMAAddress   = (uint32_t) &pxParameters.config.dev_timer->Instance->CCR3;//[1]
    STPConfig.EnbTIMDMA        = TIM_DMA_CC3;                                               //[2]
    STPConfig.PulsEGRBit       = TIM_EVENTSOURCE_CC3;                                       //[3]
    STPConfig.PulsSRBit        = TIM_FLAG_CC3;                                              //[4]

    STPConfig.CounEGRBit       = TIM_EVENTSOURCE_CC2;                                       //[5]
    STPConfig.CounSRBit        = TIM_FLAG_CC2;                                              //[6]
    STPConfig.CounIntBit       = TIM_IT_CC2;                                                //[7]

    Stepper NEMA(pxParameters.config.dev_timer, pxParameters.config.dev_dma,
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
    uint8_t lcenable;           // Local copy of input enable flag
    uint8_t lcmove;             // Local copy of input movement flag
    uint8_t counter = 0;        // Counter to get desired movement
    _HALParam   curmovement = {.data = -999.0,             // Initialise signal with default value
                               .flt  = _HALParam::Faulty };// Indicate data as fault

#define UPPERCOUNT          3   // To make movement occur every 1s, this task runs at 250ms

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
        lcenable    = *(pxParameters.input.enable);
        lcmove      = *(pxParameters.input.move);

        // Then check if a request to ENABLE the stepper has been made (i.e. enable is not zero)
        if (lcenable != 0) {    // If there has been a request to enable then...
            STP_ENABLE.setValue(GPIO::State::LOW); // Set the ENABLE pin to LOW (enable)

            curmovement.flt = _HALParam::NoFault;
            // If the motor is enabled, so movement to non-faulty

            // Now that the Stepper is enabled, check to see a movement request has been made,
            // (i.e. move is not zero)
            if (lcmove != 0) {  // If there has been a request to move then ...
                if (counter == 0) {         // Only when counter is zero
                    counter = UPPERCOUNT;   // reset counter for next loop

                    if (MtrDir == GPIO::State::LOW) {       // If Motor direction is to be LOW
                        curmovement.data= +10.00;           // Indicate a position movement

                        NEMA.newPosition(MtrDir, 0, 20000, 10,
                                         Stepper::CountPanel::Polarity::UP, 1);
                        // Put in a request for the Stepper interrupt handler, to move the motor
                        // by 10 steps at 20000 counts - (50Hz, each step is 0.02s)

                        MtrDir = GPIO::State::HIGH;         // Change direction for next loop
                    } else {
                        curmovement.data= -10.00;           // Indicate a position movement

                        NEMA.newPosition(MtrDir, 0, 20000, 10,
                                         Stepper::CountPanel::Polarity::DOWN, 1);
                        // Put in a request for the Stepper interrupt handler, to move the motor
                        // by 10 steps at 20000 counts - (50Hz, each step is 0.02s)
                        MtrDir = GPIO::State::LOW;          // Change direction for next loop
                    }

                }
                else {
                    counter--;      // Decrement counter
                }
            }

            // If there is no movement request then
            else {
                // set return movement indication to '0'
                curmovement.data= 0.00;     // Set 'curmovement' to zero
                // If the motor is enabled, so movement to non-faulty
                counter         = 0;        // Reset counter

                MtrDir = GPIO::State::LOW;  // Return the motor direction flag to 'LOW'
                STP_RESET.setValue(GPIO::State::LOW);
            }

        }

        // If there has been no request then, indicate that no movement is to occur.
        // and disable the Stepper IC
        else {
            curmovement.data= 0.00;     // Set 'curmovement' to zero
            curmovement.flt = _HALParam::Faulty;
            // If the motor is disabled, set as faulty

            counter     = 0;    // Reset counter

            STP_ENABLE.setValue(GPIO::State::HIGH); // Set the ENABLE pin to HIGH (disabled)
        }

        // Link internal signals to output pointers:
        *(pxParameters.output.movement)   = curmovement;


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

