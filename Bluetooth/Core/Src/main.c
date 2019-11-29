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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void tachChuoi(void);
void SG_WriteRegister(uint16_t dat);
void SG_Reset();
void SG_freqSet(long frequency, int wave);
void InitSigGen(void);
void SerialCommand(void);
void SG_freqReset(long frequency, int wave);
void readADC(void);
void convertADC(void);
void display(void);
void calculation(void);
void resetValue(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rxBufferUart1[4];       // thanh ghi cho UART1
uint8_t rxBufferUart2[7];		// thanh ghi cho UART2
uint8_t ChieuCao;
uint8_t CanNang;
uint8_t GioiTinh;
uint8_t Tuoi;

uint8_t flag =0;		// co flag

uint32_t value[2];
uint32_t sumofMAG = 0;
uint32_t sumofPHS = 0;

uint32_t sumofMAG1 = 0;
uint32_t sumofPHS1 = 0;

uint16_t voltMAG = 0;
uint16_t voltPHS = 0;

uint16_t MAG = 0;
int16_t PHS = 0;

uint32_t R50 = 0;
uint32_t Xc = 0;

uint32_t xFFM = 0;
uint32_t xFM = 0;
uint32_t xTBW = 0;
uint32_t xECW = 0;
uint32_t xICW = 0;
uint32_t xBCM = 0;

const uint8_t numberOfDigits = 6;		//Thanh ghi gia tri AD9833 tu 1 - 999,999 Hz.
uint8_t freqSG[6] = {0,0,0,0,5,0};		// gia tri 50 Khz


// Khai bao dang song
const uint16_t wSine     = 0b0000000000000000;
const uint16_t wTriangle = 0b0000000000000010;
const uint16_t wSquare   = 0b0000000000101000;

uint16_t waveType;

// ham 10^y
uint32_t Power(uint16_t y) {
	uint32_t t = 1;
  for (uint8_t i = 0; i < y; i++)
    t = t * 10;
  return t;
}
// tinh toan tan so cho mang
uint32_t calcFreq(uint8_t* freqSG) {
	uint32_t i = 0;
  for (uint8_t x = 0; x < numberOfDigits; x++)
    i = i + freqSG[x] * Power(x);
  return i;
}

//-----------------------------------------------------------------------------
// SG_WriteRegister
//-----------------------------------------------------------------------------
void SG_WriteRegister(uint16_t dat) {
	HAL_GPIO_WritePin(GPIOB, Clk_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Clk_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, FSync_Pin, GPIO_PIN_RESET);

  for (uint8_t i = 0; i < 16; i++) {
    if (dat & 0x8000)
    	HAL_GPIO_WritePin(Data_GPIO_Port, Data_Pin, GPIO_PIN_SET);
    else
    	HAL_GPIO_WritePin(Data_GPIO_Port, Data_Pin, GPIO_PIN_RESET);
    dat = dat << 1;
    HAL_GPIO_WritePin(GPIOB, Clk_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, Clk_Pin, GPIO_PIN_RESET);
  }
  HAL_GPIO_WritePin(GPIOB, Clk_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, FSync_Pin, GPIO_PIN_SET);
}
//-----------------------------------------------------------------------------
// SG_Reset
//-----------------------------------------------------------------------------
void SG_Reset() {
  HAL_Delay(100);
  SG_WriteRegister(0x100);
  HAL_Delay(100);
}
//-----------------------------------------------------------------------------
// SG_freqSet
//    set the SG frequency regs
//-----------------------------------------------------------------------------
void SG_freqSet(long frequency, int wave) {
  long fl = frequency * (0x10000000 / 25000000.0);
  SG_WriteRegister(0x2000 | wave);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
}
//-----------------------------------------------------------------------------
// InitSigGen
//-----------------------------------------------------------------------------
void InitSigGen(void) {
  HAL_GPIO_WritePin(GPIOB, FSync_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, Clk_Pin, GPIO_PIN_SET);
  SG_Reset();
  waveType = wSine;
  SG_freqReset(calcFreq(freqSG), waveType);
}
//-----------------------------------------------------------------------------
// SerialCommand
//   if a byte is available in teh seril input buffer
//   execute it as a command
//-----------------------------------------------------------------------------


void SerialCommand(void) {
	for(int i=0; i < 7; i++)
	{
		if ((rxBufferUart2[i] >= '0') && (rxBufferUart2[i] <= '9')){
			for (int j=5; j>0; j--) freqSG[j] = freqSG[j-1];
			      freqSG[0] = rxBufferUart2[i] - '0';
		}else {
		      switch (rxBufferUart2[i]) {
		        case 'S': waveType = wSine; SG_freqReset(calcFreq(freqSG), waveType); break;   // SigGen wave is sine
		        case 'T': waveType = wTriangle; SG_freqReset(calcFreq(freqSG), waveType); break;   // SigGen wave is triangle
		        case 'Q': waveType = wSquare; SG_freqReset(calcFreq(freqSG), waveType); break;   // SigGen wave is square
		        case 'R': SG_Reset();  break;   // SigGen reset
		        default: return;
		      }
		}
	}
}
//-----------------------------------------------------------------------------
// SG_freqReset
//    reset the SG regs then set the frequency and wave type
//-----------------------------------------------------------------------------
void SG_freqReset(long frequency, int wave) {
  long fl = frequency * (0x10000000 / 25000000.0);
  SG_WriteRegister(0x2100);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
  SG_WriteRegister(0xC000);
  SG_WriteRegister(wave);
  waveType = wave;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  InitSigGen();

  HAL_UART_Receive_DMA(&huart1, rxBufferUart1, 4);
  HAL_UART_Receive_DMA(&huart2, rxBufferUart2, 7);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if ( flag == 1)
	  {
		  readADC();
		  convertADC();
		  calculation();
		  display();
		  resetValue();
		  flag = 0;
	  }
	  SerialCommand();
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

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|Data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Clk_GPIO_Port, Clk_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSync_GPIO_Port, FSync_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : ButtonUser_Pin */
  GPIO_InitStruct.Pin = ButtonUser_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ButtonUser_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin Data_Pin */
  GPIO_InitStruct.Pin = LED_Pin|Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Clk_Pin FSync_Pin */
  GPIO_InitStruct.Pin = Clk_Pin|FSync_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		HAL_UART_Transmit_DMA(&huart2, rxBufferUart1, 4);
		tachChuoi();
		flag = 1;
	}
	if (huart->Instance == USART2)
	{
		//HAL_UART_Transmit_DMA(&huart1, rxBufferUart2, 7);
		uint8_t notice[5] = "OK!\r\n";
		HAL_UART_Transmit(&huart2, notice, 5, 1);
	}
}
 void tachChuoi(void)
 {
	 ChieuCao = rxBufferUart1[0];
	 CanNang = rxBufferUart1[1];
	 GioiTinh = rxBufferUart1[2];
	 Tuoi = rxBufferUart1[3];
 }

// ngat button
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(ButtonUser_GPIO_Port, ButtonUser_Pin) == GPIO_PIN_RESET)
	{
		uint8_t number1 = 49;
		uint8_t number2 = 50;
		uint8_t number3 = 51;
		uint8_t number4 = 52;
		uint8_t number5 = 53;
		uint8_t number6 = 54;

		uint32_t a1 = 12;
		uint32_t a2 = 13;
		uint32_t a3 = 14;
		uint32_t a4 = 15;
		uint32_t a5 = 15;
		uint32_t a6 = 16;

		//uint8_t number[6] = 7;

		HAL_UART_Transmit(&huart1, &number1, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *) &a1, 4, 1000);

		HAL_UART_Transmit(&huart1, &number2, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *) &a2, 4, 1000);

		HAL_UART_Transmit(&huart1, &number3, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *) &a3, 4, 1000);

		HAL_UART_Transmit(&huart1, &number4, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *) &a4, 4, 1000);

		HAL_UART_Transmit(&huart1, &number5, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *) &a5, 4, 1000);

		HAL_UART_Transmit(&huart1, &number6, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *) &a6, 4, 1000);
	}
}
//
void resetValue(void)
{
	sumofMAG = 0;
	sumofPHS = 0;
}
// Ham Fat Free Mass
float FFMfFunction(uint16_t height,uint16_t weight,uint16_t R50,uint16_t Xc,uint8_t sex)  // 1 la nam, 0 la nu
{
	float FFM = 0;
	FFM = -4.104 + 0.518*height*height/R50 + 0.231*weight + 0.130*Xc + 4.229*sex;
	return FFM;
}
// Ham Fat Mass
float FM(uint16_t weight,uint32_t FFM)
{
	float FM = 0;
	FM = weight - FFM;
	return FM;
}
// Ham Total Body Water
float TBW(uint16_t height, uint16_t weight, uint16_t R50, uint8_t sex) // 1 la nam, 0 la nu
{
	float TBW = 0;
	if (sex == 1)
	{
		TBW = 8.399 + 0.396*height*height/R50 + 0.143*weight;
		return TBW;
	}
	else
	{
		TBW = 8.315 + 0.382*height*height/R50 + 0.105*weight;
		return TBW;
	}
}
//Ham Extra Cellular Water
float ECW(uint16_t height,uint16_t weight,uint16_t R50,uint16_t Xc,uint8_t healthy,uint8_t sex)    // 0 la nam, 1 la nu; 1 la khoe, 2 la om
{
	float ECW = 0;
	ECW = -5.22 + 0.20*height*height/R50 + 0.005*height*height/Xc + 0.08*weight + 1.9*healthy + 1.86*sex;
	return ECW;
}
float ICW(uint32_t TBW,uint32_t ECW)
{
	float ICW = 0;
	ICW = TBW - ECW;
	return ICW;
}
// Ham Body Cell Mass
float BCM(uint16_t height,uint16_t weight,uint16_t Xc,uint8_t sex)
{
	float BCM = 0;
	if (sex == 1)
	{
		BCM = 1/120*((0.76*(59.06*powf(height, 1.6)/powf(Xc,0.5)) + 18.52*weight - 386.66));
		return BCM;
	}
	else
	{
		BCM = 1/120*((0.96*(1.306*powf(height, 2.07)/powf(Xc,0.36)) + 5.79*weight - 230.51));
		return BCM;
	}
}
//
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	__NOP();
}
//
void readADC(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, value, 2);
	for (uint8_t i = 0; i < 30; i++)
	{
		HAL_Delay(100);
		sumofMAG = sumofMAG + value[1];
		sumofPHS = sumofPHS + value[0];
		HAL_ADC_Start(&hadc1);
	}
	HAL_ADC_Stop_DMA(&hadc1);
	sumofMAG1 = sumofMAG/30;
	sumofPHS1 = sumofPHS/30;
}
//

//
void convertADC(void)
{
	voltMAG = sumofMAG1*3300/4095;
	voltPHS = sumofPHS1*3300/4095;
	voltPHS = voltPHS - 50;

	MAG = 510*powf(10.0, ((float)(voltMAG-900))/600);
	PHS = -((900 - voltPHS)/10+90);

	R50 =  MAG* cosf(PHS*M_PI/180);
	Xc =   MAG* sinf(abs(PHS)*M_PI/180);
}
//
void calculation(void)
{
	xFFM = FFMfFunction(ChieuCao, CanNang, R50, Xc, GioiTinh);
	xFM = FM(CanNang, xFFM);
	xTBW = TBW(ChieuCao, CanNang, R50, GioiTinh);
	if (GioiTinh == 1)
	{
		xECW = ECW(ChieuCao, CanNang, R50, Xc, 1, 0);
	}
	else
	{
		xECW = ECW(ChieuCao, CanNang, R50, Xc, 1, 1);
	}
	xICW = ICW(xTBW, xECW);
	xBCM = BCM(ChieuCao, CanNang, Xc, GioiTinh);
}
//
void display(void)
{
	uint8_t number1 = 49;
	uint8_t number2 = 50;
	uint8_t number3 = 51;
	uint8_t number4 = 52;
	uint8_t number5 = 53;
	uint8_t number6 = 54;
	uint8_t number7 = 55;
	uint8_t number8 = 56;

	HAL_UART_Transmit(&huart1, &number1, 1, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *) &xFFM, 4, 10);

	HAL_UART_Transmit(&huart1, &number2, 1, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *) &xFM, 4, 10);

	HAL_UART_Transmit(&huart1, &number3, 1, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *) &xTBW, 4, 10);

	HAL_UART_Transmit(&huart1, &number4, 1, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *) &xECW, 4, 10);

	HAL_UART_Transmit(&huart1, &number5, 1, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *) &xICW, 4, 10);

	HAL_UART_Transmit(&huart1, &number6, 1, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *) &xBCM, 4, 10);

	HAL_UART_Transmit(&huart1, &number7, 1, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *) &R50, 4, 10);

	HAL_UART_Transmit(&huart1, &number8, 1, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *) &Xc, 4, 10);
}
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
