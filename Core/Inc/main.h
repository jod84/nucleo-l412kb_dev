/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC32_IN_Pin LL_GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin LL_GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define RS485_TX_Pin LL_GPIO_PIN_9
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin LL_GPIO_PIN_10
#define RS485_RX_GPIO_Port GPIOA
#define RS485_TXDE_Pin LL_GPIO_PIN_12
#define RS485_TXDE_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DBG_LED_Pin LL_GPIO_PIN_3
#define DBG_LED_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

#define RS485_GPIO_Port 		GPIOA

#define DBG_LED_On()			LL_GPIO_SetOutputPin(DBG_LED_GPIO_Port, DBG_LED_Pin);
#define DBG_LED_Off()			LL_GPIO_ResetOutputPin(DBG_LED_GPIO_Port, DBG_LED_Pin);
#define DBG_LED_Toggle()		LL_GPIO_TogglePin(DBG_LED_GPIO_Port, DBG_LED_Pin);

#define DBG_LED_Init()			DBG_LED_Off(); \
								GPIO_InitStruct.Pin = DBG_LED_Pin; \
								GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; \
								GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW; \
								GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; \
								GPIO_InitStruct.Pull = LL_GPIO_PULL_NO; \
								LL_GPIO_Init(DBG_LED_GPIO_Port, &GPIO_InitStruct);

#define RS485_TxEnable() 		LL_GPIO_SetOutputPin(RS485_TXDE_GPIO_Port, RS485_TXDE_Pin);
#define RS485_IsTxEnable()		(LL_GPIO_IsOutputPinSet(RS485_TXDE_GPIO_Port, RS485_TXDE_Pin) == 1u)
#define RS485_TxDisable()		LL_GPIO_ResetOutputPin(RS485_TXDE_GPIO_Port, RS485_TXDE_Pin);



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
