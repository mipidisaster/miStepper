/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEPPER_VOLTAGE_Pin GPIO_PIN_0
#define STEPPER_VOLTAGE_GPIO_Port GPIOA
#define STEPPER_CURRENT_Pin GPIO_PIN_1
#define STEPPER_CURRENT_GPIO_Port GPIOA
#define FAN_VOLTAGE_Pin GPIO_PIN_2
#define FAN_VOLTAGE_GPIO_Port GPIOA
#define FAN_CURRENT_Pin GPIO_PIN_3
#define FAN_CURRENT_GPIO_Port GPIOA
#define AS5048_Pin GPIO_PIN_4
#define AS5048_GPIO_Port GPIOA
#define A4988_DIR_Pin GPIO_PIN_0
#define A4988_DIR_GPIO_Port GPIOB
#define A4988_STEP_Pin GPIO_PIN_1
#define A4988_STEP_GPIO_Port GPIOB
#define A4988_NSLEEP_Pin GPIO_PIN_2
#define A4988_NSLEEP_GPIO_Port GPIOB
#define A4988_NRESET_Pin GPIO_PIN_10
#define A4988_NRESET_GPIO_Port GPIOB
#define A4988_MS3_Pin GPIO_PIN_11
#define A4988_MS3_GPIO_Port GPIOB
#define A4988_MS2_Pin GPIO_PIN_12
#define A4988_MS2_GPIO_Port GPIOB
#define A4988_MS1_Pin GPIO_PIN_13
#define A4988_MS1_GPIO_Port GPIOB
#define A4988_NENABLE_Pin GPIO_PIN_14
#define A4988_NENABLE_GPIO_Port GPIOB
#define PWM_FAN_Pin GPIO_PIN_15
#define PWM_FAN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

void TIM1_BRK_TIM15_IRQHandler(void);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
