/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef DEBUG
    #include <stdio.h>

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif

#include "ili9341.h"
#include "mfrc522.h"
#include "Keypad4x3.h"
#include "RFIDHandler.h"
#include "gsm.h"
#include "fonts.h"
#include "ee24.h"
#include "userinterface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
QueueHandle_t GsmQueue;
uint8_t QueueBuffer[20];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void UserTask1(void const * argument);
void TaskGsm(void const * argument);
static void IntBuffToString(uint8_t *String,uint8_t *NumBuff,uint8_t Nof);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct
{
    uint8_t HookState;
}SysVariables;

SysCfg_t SysCfg;




PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
    return ch;
}


static void IntBuffToString(uint8_t *String,uint8_t *NumBuff,uint8_t Nof)
{
    uint8_t Cnt = 0;
    for(Cnt=0;Cnt<Nof;Cnt++)
    {
        String[Cnt] = NumBuff[Cnt]+'0';

    }
    String[Cnt] = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    BaseType_t Result;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  MFRC522_Init();
  printf("Hello World..\n");

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(Key_RFID, UserTask1, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(Key_RFID), NULL);
  osThreadDef(GmsHandler, TaskGsm, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(GmsHandler), NULL);
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
  (void) Result;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MFRC522_CS_Pin|GSM_KEY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MFRC522_RST_Pin|ILI9341_DC_Pin|ILI9341_RES_Pin|KEY_SCAN3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ILI9341_CS_Pin|KEY_SCAN1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI9341_BKLT_GPIO_Port, ILI9341_BKLT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KEY_SCAN2_GPIO_Port, KEY_SCAN2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Hook_Switch_Pin */
  GPIO_InitStruct.Pin = Hook_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Hook_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MFRC522_CS_Pin */
  GPIO_InitStruct.Pin = MFRC522_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MFRC522_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MFRC522_RST_Pin ILI9341_DC_Pin */
  GPIO_InitStruct.Pin = MFRC522_RST_Pin|ILI9341_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ILI9341_CS_Pin */
  GPIO_InitStruct.Pin = ILI9341_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ILI9341_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ILI9341_BKLT_Pin ILI9341_RES_Pin KEY_SCAN3_Pin */
  GPIO_InitStruct.Pin = ILI9341_BKLT_Pin|ILI9341_RES_Pin|KEY_SCAN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_KEY_Pin */
  GPIO_InitStruct.Pin = GSM_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GSM_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_RET4_Pin */
  GPIO_InitStruct.Pin = KEY_RET4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_RET4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_RET3_Pin KEY_RET2_Pin */
  GPIO_InitStruct.Pin = KEY_RET3_Pin|KEY_RET2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_SCAN2_Pin */
  GPIO_InitStruct.Pin = KEY_SCAN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(KEY_SCAN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_SCAN1_Pin */
  GPIO_InitStruct.Pin = KEY_SCAN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(KEY_SCAN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_RET1_Pin */
  GPIO_InitStruct.Pin = KEY_RET1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_RET1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define ON_HOOK     0x0Au
#define OFF_HOOK    0x50u

void UpdateHookState(void)
{
    static uint8_t HookFilter = 0;

    HookFilter = HookFilter << 1;
    if(GPIO_PIN_SET == HAL_GPIO_ReadPin(Hook_Switch_GPIO_Port, Hook_Switch_Pin))
    {
        HookFilter |= 0x01u;
    }
    else
    {
        HookFilter &= 0xFEu;
    }

    if(HookFilter == 0x0F)SysVariables.HookState = OFF_HOOK;
    if(HookFilter == 0xF0)SysVariables.HookState = ON_HOOK;
}



enum {ST_INIT=0,ST_WAITforNETWORK,ST_OFFHOOK_WAIT,ST_NUM_DIAL,ST_ONHOOK_WAIT,ST_COLLECT_NUM,ST_CARD_WAIT,ST_METERING}SysState;
uint32_t CallDuration = 0;
uint32_t SecCnt = 0;
uint8_t CallSCharge = 0;
uint8_t CallUnitPrice = 0;
uint16_t CallPrice = 0;


uint8_t DispNumber[20];     /* To Display */
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
    bool fDialToneOn = NO;
    uint8_t DialedNumber[16];   /* To hold dialed number */

    uint8_t DialedCnt = 0u;

    osMessageQDef(GsmQueue,1, uint32_t);

    GsmQueue = xQueueCreate(1,4);

/* Infinite loop */
    for( ;; )
    {
        vTaskDelay(10);
        UpdateHookState();
        switch( SysState )
        {
            case ST_INIT:
            {
                int32_t  CfgStatus;
                CfgStatus = EE24_ReadConfig(&SysCfg);
                if( CfgStatus == -1) /* On First PowerOn, Corrupted cases,write defaults */
                {
                    EE24_SetCfgDefaults();  /* Write Defaults */
                }
                USER_DrawStatusBar();
                ILI9341_BkltOn();

                SysVariables.HookState = ON_HOOK;
                ILI9341_WriteString(55, 92, "WAITING FOR NETWORK", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
                SysState = ST_WAITforNETWORK;
                /* SysState = ST_METERING; */

#if 0
                int32_t  CfgStatus;
                CfgStatus = EE24_ReadConfig(&SysCfg);
                if( CfgStatus == -2)
                {
                    /* Change in Lock byte */
                }
                else
                {
                    if( CfgStatus == -1) /* On First PowerOn, Corrupted cases,write defaults */
                    {
                        EE24_SetCfgDefaults();  /* Write Defaults */
                    }
                    else
                    {
                        SysState = WAITforNETWORK;
                    }
                }
#endif
                break;
            }
            case ST_WAITforNETWORK:
                if(1 == gsm_registered())
                {
                    if(NO == SmartCard.Found)
                    {
                        ILI9341_WriteString(55, 92, "INSERT CARD           ", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
                    }
                SysState = ST_OFFHOOK_WAIT;
                }
                break;
            case ST_OFFHOOK_WAIT:
                if( (SysVariables.HookState == OFF_HOOK) && (YES == SmartCard.Found) )
                {
                    QueueBuffer[0] = 0x33;
                    osMessagePut(GsmQueue, QueueBuffer, 100);
                    fDialToneOn = true;
                    Key_Init();
                    SysState = ST_CARD_WAIT;
                }
                break;

            case ST_CARD_WAIT:
                if( YES == SmartCard.Found )
                {
                    if( 1 == SysCfg.Mode )
                    {
                        IntBuffToString(DispNumber, SmartCard.Number1, 10);
                        ILI9341_WriteString(80, 80, DispNumber, Font_16x26, ILI9341_WHITE, ILI9341_BLACK);
                        IntBuffToString(DispNumber, SmartCard.Number2, 10);
                        ILI9341_WriteString(80, 120, DispNumber, Font_16x26, ILI9341_COLOR565(192, 192, 192),
                                ILI9341_BLACK);
                        IntBuffToString(DispNumber, SmartCard.Number3, 10);
                        ILI9341_WriteString(80, 160, DispNumber, Font_16x26, ILI9341_COLOR565(192, 192, 192),
                                ILI9341_BLACK);
                        IntBuffToString(DispNumber, SmartCard.Number1, 10);
                        ILI9341_WriteString(80, 200, DispNumber, Font_16x26, ILI9341_COLOR565(192, 192, 192),
                                ILI9341_BLACK);
                    }
                    else
                    {
                        ILI9341_WriteString(55, 92, "DAIL THE NUMBER           ", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
                        SysState = ST_COLLECT_NUM;
                    }
                    DialedCnt = 0;
                }

                if(( ON_HOOK == SysVariables.HookState  ) || (NO == SmartCard.Found))
                {
                    SysState = ST_ONHOOK_WAIT;
                }

                break;
            case ST_COLLECT_NUM:
                {
                    int8_t Key;
                    Key = Key_GetData();
                    if( (-1 != Key) && (Key < 10) )
                    {
                        if(true == fDialToneOn)
                        {
                            QueueBuffer[0] = 0x44;
                            osMessagePut(GsmQueue,QueueBuffer,100);
                            fDialToneOn = false;
                        }

                        if( 1 == SysCfg.Mode )
                        {
                            if( Key < 5 )
                            {
                                DialedNumber[0] = Key;
                                DispNumber[0] = Key + '0';
                                DispNumber[1] = 0;
                                DialedCnt = 1;
                            }
                        }
                        else
                        {
                            DialedNumber[DialedCnt] = Key;
                            DispNumber[DialedCnt]   = Key + '0';
                            DispNumber[DialedCnt+1] = 0;
                            DialedCnt++;
                        }
                        ILI9341_WriteString(170-((DialedCnt*26)/2), 92, DispNumber, Font_16x26, ILI9341_WHITE,ILI9341_BLACK);
                    }

                    if( (DialedCnt > 9) || (Key == KEY_HASH && DialedCnt != 0) )
                    {

                        QueueBuffer[0] = 0x11;
                        QueueBuffer[1] = 0x00; //Size of the Msg.For now 0
                        uint8_t Cnt;
                        for(Cnt=0;Cnt<10;Cnt++)
                        {
                            QueueBuffer[Cnt+2] = DispNumber[Cnt];
                        }
                        QueueBuffer[Cnt+2] = 0; /* Terminate String Buffer */
                        osMessagePut(GsmQueue,QueueBuffer,100);
                        SysState = ST_NUM_DIAL;
                    }

                    if( ( ON_HOOK == SysVariables.HookState) || (NO == SmartCard.Found) )
                    {
                        SysState = ST_ONHOOK_WAIT;
                    }
                }
                break;
            case ST_NUM_DIAL:

                if(( ON_HOOK == SysVariables.HookState  ) || (NO == SmartCard.Found) )
                {
                    CallPrice = SysCfg.UnitPrice + SysCfg.SCharge;
                    QueueBuffer[0] = 0x22;
                    osMessagePut(GsmQueue,QueueBuffer,100);
                    SysState = ST_ONHOOK_WAIT;
                }

                if(gsm_call_state() == CL_ACTIVE)
                {
                    SysState = ST_METERING;
                    SecCnt = HAL_GetTick();
                    CallDuration = 1;
                }

                if(gsm_call_state() == CL_DISCONNECT)
                {
                    SysState = ST_CARD_WAIT;
                }

                break;

            case ST_METERING:
            {
                if(( ON_HOOK == SysVariables.HookState  ) || (NO == SmartCard.Found) )
                {
                    CallPrice = SysCfg.UnitPrice + SysCfg.SCharge;
                    QueueBuffer[0] = 0x22;
                    osMessagePut(GsmQueue,QueueBuffer,100);
                    SysState = ST_ONHOOK_WAIT;
                }

                if((HAL_GetTick() - SecCnt) > 1000)
                {
                    SecCnt = HAL_GetTick();
                    CallDuration++;
                }
//              sprintf(DispNumber,"%02s:%02s",(uint8_t)(CallDuration/600),(uint8_t)(CallDuration%600));
                sprintf(DispNumber,"%02d:%02d",(uint8_t)(CallDuration/60),(uint8_t)(CallDuration%60));
                ILI9341_WriteString(55, 92, DispNumber, Font_16x26, ILI9341_WHITE,ILI9341_BLACK);
                break;
            }
            case ST_ONHOOK_WAIT:

                if(true == fDialToneOn)
                {
                    QueueBuffer[0] = 0x44;
                    osMessagePut(GsmQueue,QueueBuffer,100);
                    fDialToneOn = false;
                }
                else
                {
                    USER_ClearDisplay();
                    QueueBuffer[0] = 0x22;
                    osMessagePut(GsmQueue,QueueBuffer,100);
                }
                SysState = ST_WAITforNETWORK;
                break;

            default:
                break;

        }
    }

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
