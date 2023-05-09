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
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "queue.h"


typedef struct
{
    uint32_t Bid;               /* Box Identification number */
    int8_t BoxActiveStatus;     /* Box status Active/DeActive */
    uint8_t UnitPrice;          /* Deduction Amount per minute */
    uint8_t SCharge;            /* Service Charge */
    uint8_t Mode;               /* Restricted MOde or Normal Mode */
    // Active Hours
    // InCall Accept Status
}SysCfg_t;  /* Total 8 Bytes */

extern SysCfg_t SysCfg;
extern I2C_HandleTypeDef hi2c1;
extern QueueHandle_t GsmQueue;

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#pragma push(pack,1)
typedef struct
{
    uint32_t CardUniqId;
    uint8_t CustomerName[20];
    uint16_t CustomerId;
    uint16_t Balance:16;
    uint16_t RegionCode;
    uint8_t Number[3][10]; /*120 Bits */
}CustDetails_t;
#pragma pop(pack)

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
#define Hook_Switch_Pin GPIO_PIN_1
#define Hook_Switch_GPIO_Port GPIOA
#define MFRC522_CS_Pin GPIO_PIN_4
#define MFRC522_CS_GPIO_Port GPIOA
#define MFRC522_RST_Pin GPIO_PIN_4
#define MFRC522_RST_GPIO_Port GPIOC
#define ILI9341_CS_Pin GPIO_PIN_12
#define ILI9341_CS_GPIO_Port GPIOB
#define ILI9341_BKLT_Pin GPIO_PIN_6
#define ILI9341_BKLT_GPIO_Port GPIOC
#define ILI9341_DC_Pin GPIO_PIN_7
#define ILI9341_DC_GPIO_Port GPIOC
#define ILI9341_RES_Pin GPIO_PIN_8
#define ILI9341_RES_GPIO_Port GPIOC
#define GSM_KEY_Pin GPIO_PIN_8
#define GSM_KEY_GPIO_Port GPIOA
#define KEY_RET4_Pin GPIO_PIN_15
#define KEY_RET4_GPIO_Port GPIOA
#define KEY_RET3_Pin GPIO_PIN_10
#define KEY_RET3_GPIO_Port GPIOC
#define KEY_SCAN3_Pin GPIO_PIN_11
#define KEY_SCAN3_GPIO_Port GPIOC
#define KEY_RET2_Pin GPIO_PIN_12
#define KEY_RET2_GPIO_Port GPIOC
#define KEY_SCAN2_Pin GPIO_PIN_2
#define KEY_SCAN2_GPIO_Port GPIOD
#define KEY_SCAN1_Pin GPIO_PIN_3
#define KEY_SCAN1_GPIO_Port GPIOB
#define KEY_RET1_Pin GPIO_PIN_4
#define KEY_RET1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
