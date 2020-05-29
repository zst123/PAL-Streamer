/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "usbd_cdc_if.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	HAL_UART_Transmit(&huart2, (uint8_t *) "Y", 1, HAL_MAX_DELAY);
	__NOP();
}

// Display Buffer
#define ZST_DISPLAY_BLANK_COUNT (20) // no. of blanking lines
#define ZST_DISPLAY_LINE_BYTES  (30) // no. of bytes per line
#define ZST_DISPLAY_LINE_COUNT  (304 - ZST_DISPLAY_BLANK_COUNT) // no. of lines
// USB CDC Receive Callback
#define ZST_CDC_RX_SIZE (ZST_DISPLAY_LINE_COUNT * ZST_DISPLAY_LINE_BYTES) //(1024*12)
#define ZST_CDC_TX_SIZE (200)

uint8_t ZST_CDC_RxBuffer[ZST_CDC_RX_SIZE];
uint8_t ZST_CDC_TxBuffer[ZST_CDC_TX_SIZE];
//uint8_t ZST_DisplayBuffer[ZST_CDC_RX_SIZE];
#define ZST_DisplayBuffer ZST_CDC_RxBuffer

extern USBD_HandleTypeDef hUsbDeviceFS;
int8_t ZST_CDC_Init_FS(void) {
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, ZST_CDC_TxBuffer, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, ZST_CDC_RxBuffer);
  return (USBD_OK);
}
int8_t ZST_CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
  HAL_PCD_EP_Receive(hUsbDeviceFS.pData, CDC_OUT_EP, (uint8_t *) &ZST_CDC_RxBuffer, ZST_CDC_RX_SIZE);
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  return (USBD_OK);
}
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void ZST_CDC_Init() {
	USBD_Interface_fops_FS.Init = ZST_CDC_Init_FS;
	USBD_Interface_fops_FS.Receive = ZST_CDC_Receive_FS;
	snprintf((char *) ZST_CDC_TxBuffer, 200, "Y");
	memset(ZST_CDC_RxBuffer, '\x0A', ZST_CDC_RX_SIZE);
	memset(ZST_CDC_TxBuffer, '\0', ZST_CDC_TX_SIZE);
}
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

  //-----------------------------------------------------------------------------
  // Setup USB CDC
  ZST_CDC_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //-----------------------------------------------------------------------------
#define ENABLE_TIM1_PB1()  { SET_BIT(GPIOB->MODER, GPIO_MODER_MODER1_1); }
#define DISABLE_TIM1_PB1() { CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER1); }
#define ENABLE_TIM1_PA10()  { SET_BIT(GPIOA->MODER, GPIO_MODER_MODER10_1); }
#define DISABLE_TIM1_PA10() { CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER10); }

  //-----------------------------------------------------------------------------
  /*
   * Frequency calculations:
   * Choose PWM period of 14,
   * Period frequency = 96MHz/14 = 6.857MHz
   * Channel frequency = 6.857MHz*9 = 61.7MHz
   */
#define PAL_SETUP() { TIM1->ARR = 14-1; } // Period of  14 (Auto-reload register)
#define PAL_VH()	{ TIM1->CCR3 = 0; }
#define PAL_HIGH()  { TIM1->CCR3 = 4; }
#define PAL_LOW()   { TIM1->CCR3 = 7-1; } // Duty cycle (Compare register)

  // Setup TIM1_CH3 (APB2) to transmit VHF signal.
  PAL_SETUP();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  //-----------------------------------------------------------------------------
#define WATCH_PRESCALER  (1)
#define WATCH_RESET()          { TIM11->CNT = 0; }
#define WATCH_WAIT_UNTIL(x)    { while (TIM11->CNT < (x)); }
#define WATCH_GET_VALUE()      (TIM11->CNT)

#define USEC_CALCULATE(usec) ((uint16_t) ceil((usec) * HAL_RCC_GetHCLKFreq() * 1.0 / WATCH_PRESCALER))

  // Setup TIM11 as continuous timer
  __HAL_TIM_SET_PRESCALER(&htim11, WATCH_PRESCALER-1);
  __HAL_TIM_SET_AUTORELOAD(&htim11, 0xFFFF);
  HAL_TIM_Base_Start(&htim11);

  const uint16_t USEC_FRONT_PORCH = USEC_CALCULATE(1.65e-6);
  const uint16_t USEC_HORI_SYNC = USEC_CALCULATE(4.7e-6);
  const uint16_t USEC_BACK_PORCH = USEC_CALCULATE(1.0e-6); // arbitary
  const uint16_t USEC_64 = USEC_CALCULATE(64.5e-6); // H = 64us + 0.5us tolerance
  // const uint16_t USEC_63 = USEC_CALCULATE(63e-6);

  const uint16_t USEC_32 = USEC_CALCULATE(32.25e-6); // 0.5H = 32us
  const uint16_t USEC_27_3 = USEC_CALCULATE(27.3e-6);
  const uint16_t USEC_2_35 = USEC_CALCULATE(2.5e-6);

#define LONGSYNC() { \
	WATCH_RESET(); \
	PAL_LOW(); WATCH_WAIT_UNTIL(USEC_27_3);  \
	PAL_HIGH(); WATCH_WAIT_UNTIL(USEC_32);  \
}
#define SHORTSYNC() { \
	WATCH_RESET(); \
	PAL_LOW(); WATCH_WAIT_UNTIL(USEC_2_35);  \
	PAL_HIGH(); WATCH_WAIT_UNTIL(USEC_32);  \
}

  //-----------------------------------------------------------------------------
  // Setup TIM4 to overflow on every 8 ticks, and clock TIM5 to count the bytes
  const float RAW_FRAME_TIME = 55e-6;
  const uint16_t tim4_prescaler = (uint16_t) (HAL_RCC_GetHCLKFreq() * RAW_FRAME_TIME / ZST_DISPLAY_LINE_BYTES / 8);

  __HAL_TIM_SET_PRESCALER(&htim4, tim4_prescaler);
  __HAL_TIM_SET_AUTORELOAD(&htim4, 8-1);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
  __HAL_TIM_SET_PRESCALER(&htim5, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim5, 0xFFFF);
  HAL_TIM_Base_Start(&htim5);

#define DISPLAY_RESET()              { TIM5->CNT = 0; TIM4->CNT = 0;  }
#define DISPLAY_GET_BIT_VALUE()      (TIM4->CNT)
#define DISPLAY_GET_BYTE_VALUE()     (TIM5->CNT)

  //-----------------------------------------------------------------------------
  // Setup UART to receive to buffer using DMA
  const uint16_t dma_size = ZST_DISPLAY_LINE_COUNT * ZST_DISPLAY_LINE_BYTES;
  uint8_t dma_buffer[dma_size];
  HAL_UART_Receive_DMA(&huart2, (uint8_t *) &dma_buffer, dma_size);

  // Select interrupt to sleep up
  hdma_usart2_rx.Instance->CR  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
  hdma_usart2_rx.Instance->CR  &= ~(DMA_IT_HT);
  hdma_usart2_rx.Instance->FCR &= ~(DMA_IT_FE);
  hdma_usart2_rx.Instance->CR  |= DMA_IT_TC;

  //-----------------------------------------------------------------------------
  uint16_t line = 0;
  while (1) {
	  for (line = 0; line < 5; line++) { LONGSYNC(); }
	  for (line = 0; line < 5; line++) { SHORTSYNC(); }
    __NOP();

	  for (line = 0; line < 304; line++) {
		  WATCH_RESET();
		  PAL_HIGH();
		  WATCH_WAIT_UNTIL(USEC_FRONT_PORCH);
		  PAL_LOW();
		  WATCH_WAIT_UNTIL(USEC_FRONT_PORCH + USEC_HORI_SYNC);
		  PAL_HIGH();
		  WATCH_WAIT_UNTIL(USEC_FRONT_PORCH + USEC_HORI_SYNC + USEC_BACK_PORCH);
		  DISPLAY_RESET();
      __NOP();

		  if (line < ZST_DISPLAY_BLANK_COUNT) {
			  // blanking
		  } else {
			  const uint32_t displayed_line = line - ZST_DISPLAY_BLANK_COUNT;
			  const uint32_t array_offset = displayed_line*ZST_DISPLAY_LINE_BYTES;
			  uint32_t lbit = 0;
			  uint32_t lbyte = 0;
			  uint8_t isHigh;

			  while (1) {
				  lbit = DISPLAY_GET_BIT_VALUE();
				  if (lbit == 0) {
					  lbyte = DISPLAY_GET_BYTE_VALUE();
					  if (lbyte >= ZST_DISPLAY_LINE_BYTES) {
						  break;
					  }
				  }
				  isHigh = ZST_DisplayBuffer[array_offset + lbyte] & (1<<lbit);
				  if (isHigh) {
					  PAL_VH(); //1
				  } else {
					  PAL_HIGH(); //0
				  }
			  }
		  }
		  WATCH_WAIT_UNTIL(USEC_64);
	  }

#define NON_INTERLACE
#ifndef NON_INTERLACE
	  for (line = 0; line < 5; line++) { SHORTSYNC(); }
	  for (line = 0; line < 5; line++) { LONGSYNC(); }
	  for (line = 0; line < 4; line++) { SHORTSYNC(); }
    __NOP();

	  for (line = 0; line < 305; line++) {
	  }
#endif

	  for (line = 0; line < 6; line++) { SHORTSYNC(); }
    __NOP();

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
