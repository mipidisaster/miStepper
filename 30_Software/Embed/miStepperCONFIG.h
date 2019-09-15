/**************************************************************************************************
 * @file        miStepperCONFIG.h
 * @author      Thomas
 * @version     V0.3
 * @date        25 Jun 2019
 * @brief       Configuration parameters for miStepper functions
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This header contains a majority of configuration parameters which can be used to change
 * sizes/timing/etc of the tasks within the 'miStepper' Embedded device.
 *************************************************************************************************/
#ifndef MISTEPPERCONFIG_H_
#define MISTEPPERCONFIG_H_

#include <stdint.h>

/*-----------------------------------------------------------
 * Typedef's used within this embedded device
 *---------------------------------------------------------*/
// Following structure defines a global type for passing data between tasks, and declaring the
// state of value.
typedef struct {
    float   data;
    enum FltState : uint8_t {NoFault = 0x00, Faulty = 0x5A} flt;
}   _HALParam;

// Following structure defines a global type for containing the status of various
// devices/communication systems used within device
template <typename Dev, typename Comm>
struct HALDevComFlt {
    uint8_t     IdleCount;

    Dev         DevFlt;
    Comm        ComFlt;
};

/*-----------------------------------------------------------
 * Hardware Abstraction Layer, buffer sizes:
 *---------------------------------------------------------*/
//## SPI1 Device HAL
#define SPI1_FormBuffer     16          // Queue size of SPI class forms    (SPIPeriph::Form)
#define SPI1_CommBuff       64          // Define buffer size for the Transmit and Receive for SPI
                                        // communication
#define SPI1_AS5048AForm    16          // Queue size for AS5048A forms     (16bits)
#define SPI1_FaultCount     6           // Maximum count of no data, before fault action taken

//## I2C1 Device HAL
#define I2C1_FormBuffer     16          // Queue size of I2C class forms    (I2CPeriph::Form)
#define I2C1_CommBuff       64          // Define buffer size for the Transmit and Receive for I2C
                                        // communication
#define I2C1_AD7415Form     16          // Queue size for AD7415 forms      (AD741x::Form)
#define I2C1_FaultCount     6           // Maximum count of no data, before fault action taken

//## ADC1 Device HAL
#define ADC1_FormBuffer     4           // Queue size for ADC data forms    (ADCDMARecord)
#define ADC1_ConvPSeq       6           // Define the number of conversions per sequence
#define ADC1_NumSeq         5           // Define the number of ADC runs to be contained within
                                        // single DMA record

//## USART1 Device HAL
#define USART1_CommBuff     2048        // Define buffer size for the Transmit and Receive for
                                        // USART communication

/*-----------------------------------------------------------
 * Task timing
 *---------------------------------------------------------*/
#define USART_DEV_Time         5 //ms   -- Iteration rate for the USART Device/HAL task

#define SPI___DEV_Time         5 //ms   -- Iteration rate for the SPI Device/HAL task
#define I2C___DEV_Time         5 //ms   -- Iteration rate for the I2C Device/HAL task
#define ADC___DEV_Time         5 //ms   -- Iteration rate for the ADC Device/HAL task
/** READ ME BEFORE CHANGING 'ADC___DEV_Time'
  *
  * Within STM32CubeMX need to configure TIM6 so as to have a rate fastes then this
  *     Current configuration   = fcpu      = 80MHz
  *     TIM6 Prescaler          = 3999      = 80MHz / (3999 + 1)    = 20kHz       50us
  *         Counter Period      = 3999      = 20kHz / (  19 + 1)    = 1 kHz        1ms
  *         ADC1_NumSeq         = 5         = 1 kHz / 5             = 200Hz        5ms
  ************************************************************************************************/

#define FAN___HAL_Time       100 //ms   -- Iteration rate for the FAN HAL task
/** READ ME BEFORE CHANGING 'FAN___HAL_Time'
  *
  * Currently the TIMER linked to the STEPPER (TIM1) is configured to give a resolution of 1us
  * (Prescaler = 79 -> 1MHz). With the size of hardware register limited to 16bits (65535), this
  * gives the slowest STEP pulse train of ~65ms (15Hz).
  * So the FAN HAL task cannot go any faster than this, if slowest speeds are expected.
  * Therefore the task, is limited to allow a minimum speed of HALF the iteration speed (software
  * limited at compilation time).
  *
  * With the current iteration rate of 100ms, this gives the slowest STEP pulse train of 50ms
  * (20Hz).
  ************************************************************************************************/
#define STP___HAL_Time       100 //ms   -- Iteration rate for the Stepper HAL task


/*-----------------------------------------------------------
 * Task timing stats
 *---------------------------------------------------------*/

#define MiTasksStats    6           // Number of tasks that require time statistics

#define miUSART1Task    0           // Array entry which contains the USART1 task stats
#define miSPI1__Task    1           // Array entry which contains the SPI1   task stats
#define miI2C1__Task    2           // Array entry which contains the I2C1   task stats
#define miADC1__Task    3           // Array entry which contains the ADC1   task stats
#define miFan___Task    4           // Array entry which contains the Fan    task stats
#define miSteperTask    5           // Array entry which contains the Fan    task stats

typedef     uint16_t    _miTimeTrack;   // Typedef the variable type used for counter task period
                                        // and duration

struct _taskTime {
    _miTimeTrack    ActStrtTm;      // Actual start time of the task (So current task)

    _miTimeTrack    TaskPrid;
    _miTimeTrack    TaskDurt;
};

#define TimeDomainChng  20L         // Multiplier to convert the 'ms' task times stated above
                                    // to 'counts' based upon the Time stats counter
/** READ ME BEFORE CHANGING 'TimeDomainChng'
  *
  * Currently the counter used to observe the task duration and periods is based upon TIM15,
  * which currently set to:
  *     fcpu input          =   80MHz
  *     TIM15 Prescaler     =   3       = 80MHz  / (3    + 1)  = 20MHz          50ns
  *     Count Period        =   999     - 20MHz  / (999  + 1)  = 20KHz          50us
  *                                                                             0.05ms
  *     Therefore there are 20 counts to 1ms
  ************************************************************************************************/

#define TimeMAXCount    65535       // Define the maximum number which can be contained within
                                    // counter variable (2^16) - 1

void ticStartTask(uint8_t curTask);     // Global prototype for capturing the start time of task
void tocStopTask(uint8_t curTask);      // Global prototype for capturing the stop time of task
float miTaskDuration(uint8_t curTask);  // Global prototype for returning the Task Duration (ms)
float miTaskPeriod(uint8_t curTask);    // Global prototype for returning the Task Period (ms)
uint32_t miTaskData(uint8_t curTask);   // Global prototype for returning the Task Data
                                        // Duration and Period (ms)

#endif /* MISTEPPERCONFIG_H_ */
