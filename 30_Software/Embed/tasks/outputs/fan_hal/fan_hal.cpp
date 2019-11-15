/**************************************************************************************************
 * @file        fan_hal.cpp
 * @author      Thomas
 * @version     V2.1
 * @date        28 Sept 2019
 * @brief       Source file for Fan Motor task handler
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/

#include "EmbedIndex.h"
#include EMBD_FANTask

/**************************************************************************************************
 * Globally defined variables used GLOBALLY throughout system (not just this task)
 *************************************************************************************************/
_HALParam                FanAct;

/**************************************************************************************************
 * Define any externally consumed global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// Hardware parameters
extern TIM_HandleTypeDef htim15;            // Defined within 'main.cpp'

// Task inputs
extern float             FanDmd;

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * Define any local global signals
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Define any local functions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**
  * @brief:  Fan Motor Hardware Abstraction Layer task
  * @param:  void const, not used
  * @retval: None (void output)
  */
void vFANMotorHAL(void const * argument) {
/*---------------------------[     Setup Timer for PWM control    ]---------------------------*/

    // Setup and kick-off the PWM on channel 2 of timer 2
    TIM_CCxChannelCmd(htim15.Instance,  // Enable TIMER Channel 2
                      TIM_CHANNEL_2,                            // Compare Output Mode
                      TIM_CCx_ENABLE);                          //

    __HAL_TIM_MOE_ENABLE(&htim15);
    // As TIM15 (linked input) contains the break register(s), need to enable full timer
    // functionality

    __HAL_TIM_ENABLE_IT(&htim15,          // Enable TIM15 Update interrupt
                        TIM_IT_UPDATE);                         // used for time schedule logging
        // Note the interrupt handling is done within main.ccp and main.h

    __HAL_TIM_ENABLE(&htim15);            // Start timer!


/*---------------------------[      PWM Timer is now running      ]---------------------------*/

    // Create local version of linked signals (prefix with "lc")
    // #=======================================================#
    _HALParam   lcFanDmd    = {.data = -999.0,             // Initialise signal with default value
                               .flt  = _HALParam::Faulty };// Indicate data as fault

    uint16_t tempcalc = 0;      // Temporary parameter used to convert float input to actual PWM
                                // supported width

    /* Following contains the main aspects of this task-------------------------*/
    /****************************************************************************/
    /****************************************************************************/
    uint8_t FirstPass = 1;      // Indicate that this is the first time going through this task

    uint32_t PreviousWakeTime = osKernelSysTick();  // Capture start time of task within Kernel
                                                    // time
    /* Infinite loop */
    for(;;) {
        ticStartTask(miFan___Task);     // Capture time of start of task

        // Capture any new updates to input signals and link to local internals
        if (FirstPass == 1) {                   // On first pass of this task, it owns its input
                                                // as well
            lcFanDmd.data   = 0.00;                 // Drive the value to zero
            lcFanDmd.flt    = _HALParam::NoFault;   // and indicate no fault (will NEVER get set
                                                    // true)

            FanDmd          = lcFanDmd.data;
            // Now link this value to the boundary
        }
        else                                // Any other time
            lcFanDmd.data   = FanDmd;       // Task reads in the Fan Demand

        if (lcFanDmd.data <= 0.00) {        // If Fan demand is 0, i.e. Switch off PWM output
            __HAL_TIM_SET_COMPARE(&htim15,
                                  TIM_CHANNEL_2,
                                  0);

            lcFanDmd.data   = 0;
        }
        else {
            if (lcFanDmd.data >= 100.0)     // If demand is greater than PWM Defined width
                lcFanDmd.data = 100.0;      // Limit to Defined PWM width

            tempcalc    = (uint16_t) ((lcFanDmd.data / 100.0 ) * PWMWidth);
            __HAL_TIM_SET_COMPARE(&htim15,
                                  TIM_CHANNEL_2,
                                  tempcalc);
        }

        lcFanDmd.flt                    = _HALParam::NoFault;   // Ensure local Fan Demand is
                                                                // fault free

        // Link internal signals to output pointers:
        FanAct          = lcFanDmd;


        FirstPass = 0;              // Update this flag such that it now indicates that first
                                    // pass has completed
        /****************************************************************************/
        /****************************************************************************/
        /* End of task, will now wait defined period--------------------------------*/
        tocStopTask(miFan___Task);  // Capture time task completed
        osDelayUntil(&PreviousWakeTime, FAN___HAL_Time);
        // Put the task in "DELAYED" state for the defined period
        // see - miStepperCONFIG.h for the timings
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None
