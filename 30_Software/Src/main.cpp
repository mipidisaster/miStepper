/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EmbedIndex.h"
#include <stdio.h>

#include EMBD_SPITask           // Contains the task for handling SPI1 and
                                // attached devices
#include EMBD_I2CTask           // Contains the task for handling I2C1 and
                                // attached devices
#include EMBD_ADCTask           // Contains the task for handling ADC1 and
                                // attached devices

#include EMBD_USARTTask         // Contains the task for handling USART1

#include EMBD_FANTask           // Contains the task for handling the FAN
                                // motor
#include EMBD_STPTask           // Contains the task for handling the Stepper
                                // motor

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim1_ch3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId SPIDeviceManageHandle;
osThreadId USARTDeviceManageHandle;
osThreadId I2CDeviceManageHandle;
osThreadId ADCDeviceManageHandle;
osThreadId FANMotorManageHandle;
osThreadId STPMotorManageHandle;
/* USER CODE BEGIN PV */
volatile _miTimeTrack   TimeKeeper = 0;
volatile _taskTime      TimeSheet[MiTasksStats] = { 0 };

_taskSPI1   xSPI1pass;
_taskI2C1   xI2C1pass;
_taskADC1   xADC1pass;

_taskUSART1 xUSART1pass;

_taskFAN    xFANpass;
_taskSTP    xSTPpass;

// Linkage signals
// ~~~~~~~~~~~~~~~
_HALParam                                           AngPos;
_HALParam                                           ExtTmp;

_HALParam                                           IntVrf;
_HALParam                                           IntTmp;

_HALParam                                           FanVlt;
_HALParam                                           FanCur;
_HALParam                                           FanAct;

float                                               FanDmd = 0;

_HALParam                                           StpVlt;
_HALParam                                           StpCur;
uint16_t                                            StpFreqAct  = 0;
uint16_t                                            StpStatAct  = 0;
uint32_t                                            StpcalPost  = 0;

uint8_t                                             StpEnable   = 0;
uint8_t                                             StpGear     = 0;
uint8_t                                             StpDirct    = 0;
uint16_t                                            StpFreqDmd  = 0;
                    // Dmd of 18751, with gear of 7, should get 1rpm

// Maintenance fault flags
//~~~~~~~~~~~~~~~~~~~~~~~~~
SPIPeriph::DevFlt                                   SPI1CommFlt;
I2CPeriph::DevFlt                                   I2C1CommFlt;
HALDevComFlt<AS5x4x::DevFlt, SPIPeriph::DevFlt>     AS5048AFlt;
HALDevComFlt<AD741x::DevFlt, I2CPeriph::DevFlt>     AD74151Flt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
extern void vSPI1DeviceHAL(void const * argument);
extern void vUSART1DeviceHAL(void const * argument);
extern void vI2C1DeviceHAL(void const * argument);
extern void vADC1DeviceHAL(void const * argument);
extern void vFANMotorHAL(void const * argument);
extern void vSTPMotorHAL(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* RTOS Linkage Configuration-----------------------------------------------*/
  // Setup signal linkage for the SPI task
  xSPI1pass.config.spi1_handle      = &hspi1;

  xSPI1pass.output.SPI1CommFlt      = &SPI1CommFlt;
  xSPI1pass.output.AS5048AFlt       = &AS5048AFlt;
  xSPI1pass.output.AngPos           = &AngPos;

  xUSART1pass.input.AngPos          = xSPI1pass.output.AngPos;      // Ang Position     to USART
  xUSART1pass.input.SPI1CommFlt     = xSPI1pass.output.SPI1CommFlt; // Faults           to USART
  xUSART1pass.input.AS5048AFlt      = xSPI1pass.output.AS5048AFlt;  // Faults           to USART

  // Setup signal linkage for the I2C task
  xI2C1pass.config.i2c1_handle      = &hi2c1;

  xI2C1pass.output.I2C1CommFlt      = &I2C1CommFlt;
  xI2C1pass.output.AD74151Flt       = &AD74151Flt;
  xI2C1pass.output.ExtTmp           = &ExtTmp;

  xUSART1pass.input.ExtTmp          = xI2C1pass.output.ExtTmp;      // Ext Temperature  to USART
  xUSART1pass.input.I2C1CommFlt     = xI2C1pass.output.I2C1CommFlt; // Faults           to USART
  xUSART1pass.input.AD74151Flt      = xI2C1pass.output.AD74151Flt;  // Faults           to USART

  // Setup signal linkage for the ADC task
  xADC1pass.config.adc1_handle      = &hadc1;
  xADC1pass.config.adc1_dma         = &hdma_adc1;
  xADC1pass.config.adc1_timer       = &htim6;

  xADC1pass.output.IntVrf           = &IntVrf;
  xADC1pass.output.IntTmp           = &IntTmp;

  xADC1pass.output.FanVlt           = &FanVlt;
  xADC1pass.output.FanCur           = &FanCur;

  xADC1pass.output.StpVlt           = &StpVlt;
  xADC1pass.output.StpCur           = &StpCur;

  xUSART1pass.input.IntVrf          = xADC1pass.output.IntVrf;      // Int Voltage      to USART
  xUSART1pass.input.IntTmp          = xADC1pass.output.IntTmp;      // Int Temperature  to USART

  xUSART1pass.input.FanVlt          = xADC1pass.output.FanVlt;      // Fan Voltage      to USART
  xUSART1pass.input.FanCur          = xADC1pass.output.FanCur;      // Fan Current      to USART

  xUSART1pass.input.StpVlt          = xADC1pass.output.StpVlt;      // STP Voltage      to USART
  xUSART1pass.input.StpCur          = xADC1pass.output.StpCur;      // STP Current      to USART

  // Setup signal linkage for the USART task
  xUSART1pass.config.usart1_handle  = &huart1;

  xUSART1pass.output.FanDmd         = &FanDmd;

  xFANpass.input.FanDmd             = xUSART1pass.output.FanDmd;    // Fan Demand       to USART

  xUSART1pass.output.StpEnable      = &StpEnable;
  xUSART1pass.output.StpGear        = &StpGear;
  xUSART1pass.output.StpDirct       = &StpDirct;
  xUSART1pass.output.StpFreqDmd     = &StpFreqAct;

  xSTPpass.input.StpEnable          = xUSART1pass.output.StpEnable;
  xSTPpass.input.StpGear            = xUSART1pass.output.StpGear;
  xSTPpass.input.StpDirct           = xUSART1pass.output.StpDirct;
  xSTPpass.input.StpFreqDmd         = xUSART1pass.output.StpFreqDmd;

  // Setup signal linkage for the FAN task
  xFANpass.config.fan_timer         = &htim15;

  xFANpass.output.FanAct            = &FanAct;
  xUSART1pass.input.FanAct          = xFANpass.output.FanAct;


  // Setup signal linkage for the STEPPER task
  xSTPpass.config.stepper_timer     = &htim1;
  xSTPpass.config.stepper_dma       = &hdma_tim1_ch3;

  xSTPpass.output.StpFreqAct        = &StpFreqAct;
  xSTPpass.output.StpStatAct        = &StpStatAct;
  xSTPpass.output.StpcalPost        = &StpcalPost;

  xUSART1pass.input.StpFreqAct      = xSTPpass.output.StpFreqAct;   // STP Frequency    to USART
  xUSART1pass.input.StpStatAct      = xSTPpass.output.StpStatAct;   // STP Status       to USART
  xUSART1pass.input.StpcalPost      = xSTPpass.output.StpcalPost;   // STP calc Posit   to USART

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SPIDeviceManage */
  osThreadDef(SPIDeviceManage, vSPI1DeviceHAL, osPriorityNormal, 0, 384);
  SPIDeviceManageHandle = osThreadCreate(osThread(SPIDeviceManage), (void *) &xSPI1pass);

  /* definition and creation of USARTDeviceManage */
  osThreadDef(USARTDeviceManage, vUSART1DeviceHAL, osPriorityBelowNormal, 0, 384);
  USARTDeviceManageHandle = osThreadCreate(osThread(USARTDeviceManage), (void *) &xUSART1pass);

  /* definition and creation of I2CDeviceManage */
  osThreadDef(I2CDeviceManage, vI2C1DeviceHAL, osPriorityNormal, 0, 384);
  I2CDeviceManageHandle = osThreadCreate(osThread(I2CDeviceManage), (void *) &xI2C1pass);

  /* definition and creation of ADCDeviceManage */
  osThreadDef(ADCDeviceManage, vADC1DeviceHAL, osPriorityNormal, 0, 384);
  ADCDeviceManageHandle = osThreadCreate(osThread(ADCDeviceManage), (void *) &xADC1pass);

  /* definition and creation of FANMotorManage */
  osThreadDef(FANMotorManage, vFANMotorHAL, osPriorityAboveNormal, 0, 128);
  FANMotorManageHandle = osThreadCreate(osThread(FANMotorManage), (void *) &xFANpass);

  /* definition and creation of STPMotorManage */
  osThreadDef(STPMotorManage, vSTPMotorHAL, osPriorityHigh, 0, 384);
  STPMotorManageHandle = osThreadCreate(osThread(STPMotorManage), (void *) &xSTPpass);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B02989;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 3;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AS5048_GPIO_Port, AS5048_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A4988_DIR_Pin|A4988_NSLEEP_Pin|A4988_NRESET_Pin|A4988_MS3_Pin 
                          |A4988_MS2_Pin|A4988_MS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A4988_NENABLE_GPIO_Port, A4988_NENABLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : AS5048_Pin */
  GPIO_InitStruct.Pin = AS5048_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AS5048_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A4988_DIR_Pin A4988_NSLEEP_Pin A4988_NRESET_Pin A4988_MS3_Pin 
                           A4988_MS2_Pin A4988_MS1_Pin A4988_NENABLE_Pin */
  GPIO_InitStruct.Pin = A4988_DIR_Pin|A4988_NSLEEP_Pin|A4988_NRESET_Pin|A4988_MS3_Pin 
                          |A4988_MS2_Pin|A4988_MS1_Pin|A4988_NENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* Functions used to calculate task timing information -----------------------*/

/**
  * @brief:  Calculate the time difference between two time snapshots, function takes into account
  *          counter overflow
  * @param:  First Time snapshot in the '_miTimeTrack' type (which is likely uint16_t just
  *          typedef'd)
  * @param:  Second Time snapshot in the '_miTimeTrack' type (which is likely uint16_t just
  *          typedef'd)
  * @retval: Difference between the time snapshots in the '_miTimeTrack' type
  */
_miTimeTrack TimeDiff(_miTimeTrack FirstTime, _miTimeTrack SecondTime) {
    /*  Normally the 'SecondTime' will be larger then 'FirstTime', so if this is the case then can
     *  just do difference and return this.
     *
     *  Otherwise the timer has overflow, therefore the 'FirstTime' will be larger. In this case
     *  take a delta against the largest number that _miTimeTrack can store, then add the
     *  'SecondTime'.
     */
    _miTimeTrack tempValue = 0;                 // Variable of same type as input/output set to
                                                // '0'
    if (SecondTime >= FirstTime) {              // If the Second time is greater then or equal to
                                                // first then...
        tempValue = SecondTime - FirstTime;     // Straight difference between times
    }
    else {                                      // Otherwise, First time is greater (overflow)
        tempValue = TimeMAXCount - FirstTime;   // Calculate difference of First time to overrun
        tempValue += SecondTime + 1;            // then add Second time, and add 1 (to account for
    }                                           // overflow count

    return (tempValue);                         // Return calculated difference
}

/**
  * @brief:  Capture the new start time of task, and calculate the time since last task time
  * @param:  Integer entry to select the correct time entry within the '_taskTime' array (local to
  *          this code)
  * @retval: None (void output)
  */
void ticStartTask(uint8_t curTask) {
    _miTimeTrack tempTime = TimeKeeper;     // Capture the time of function start

    TimeSheet[curTask].TaskPrid     = TimeDiff(TimeSheet[curTask].ActStrtTm, tempTime);
        // Calculate difference between the last start time, and current time

    TimeSheet[curTask].ActStrtTm    = tempTime; // Update the start time to current time
}

/**
  * @brief:  Calculate the duration of the task, based upon the current time (function will only
  *          be called at the end of task.
  * @param:  Integer entry to select the correct time entry within the '_taskTime' array (local to
  *          this code)
  * @retval: None (void output)
  */
void tocStopTask(uint8_t curTask) {
    TimeSheet[curTask].TaskDurt     = TimeDiff(TimeSheet[curTask].ActStrtTm, TimeKeeper);
        // Calculate difference between the start time of task, and current time
}

/**
  * @brief:  Convert the '_taskTime' entry for calculated task duration from 'counts' to
  *          'milliseconds'
  * @param:  Integer entry to select the correct time entry within the '_taskTime' array (local to
  *          this code)
  * @retval: float value of task duration in 'milliseconds
  */
float miTaskDuration(uint8_t curTask) {
    return (((float) TimeSheet[curTask].TaskDurt / TimeDomainChng ));
}

/**
  * @brief:  Convert the '_taskTime' entry for calculated task period from 'counts' to
  *          'milliseconds'
  * @param:  Integer entry to select the correct time entry within the '_taskTime' array (local to
  *          this code)
  * @retval: float value of task period in 'milliseconds
  */
float miTaskPeriod(uint8_t curTask) {
    return (  (  (float) TimeSheet[curTask].TaskPrid / TimeDomainChng )  );
}

/**
  * @brief:  Return a concatenated word (32bit) containing the Task Period and Task Duration
  *          in 'counts'
  * @param:  Integer entry to select the correct time entry within the '_taskTime' array (local to
  *          this code)
  * @retval: float value of task period in 'milliseconds
  */
uint32_t miTaskData(uint8_t curTask) {
    uint32_t tempvariable = 0;      // 32bit Variable to temporary store Task period data

    tempvariable  = ( (uint16_t) TimeSheet[curTask].TaskDurt ) << 16;
    tempvariable |= ( (uint16_t) TimeSheet[curTask].TaskPrid );

    return (tempvariable);  // Return 32bit variable
}

/**
  * @brief:  Timer 15 Interrupt Service Routine handler (does include TIM1 break)
  * @param:  None (void input)
  * @retval: None (void output)
  */
void TIM1_BRK_TIM15_IRQHandler(void) {
    __HAL_TIM_CLEAR_IT(&htim15,             // Interrupt is only expected for TIM16 event
                       TIM_FLAG_UPDATE);    // interrupt, therefore clear flag

    TimeKeeper++;       // Increment time keeper parameter
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
