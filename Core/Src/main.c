/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fonts.h"
#include "ssd1306.h"
#include "task.h"
#include <stdio.h>
#include "fft.h"
#include "tomasitos_library.h"
#include "math.h"
//#include "twid256.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_BUTTON 	GPIOB,GPIO_PIN_13
#define BUTTON			GPIOB,GPIO_PIN_12
#define LED				GPIOC,GPIO_PIN_13
#define NSEN 130
#define VMAX 720

#define PIN0			GPIOA,GPIO_PIN_11
#define PIN1			GPIOA,GPIO_PIN_12
#define PIN2			GPIOA,GPIO_PIN_15
#define PIN3			GPIOB,GPIO_PIN_3
#define PIN4			GPIOB,GPIO_PIN_4
#define PIN5			GPIOB,GPIO_PIN_5

//#define FFT_BUFFER_SIZE 256
#define ADC_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for SignalGenerator */
osThreadId_t SignalGeneratorHandle;
const osThreadAttr_t SignalGenerator_attributes = {
  .name = "SignalGenerator",
  .stack_size = 140 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ScreenControler */
osThreadId_t ScreenControlerHandle;
const osThreadAttr_t ScreenControler_attributes = {
  .name = "ScreenControler",
  .stack_size = 152 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InputControler */
osThreadId_t InputControlerHandle;
const osThreadAttr_t InputControler_attributes = {
  .name = "InputControler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DataControler */
osThreadId_t DataControlerHandle;
const osThreadAttr_t DataControler_attributes = {
  .name = "DataControler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for eventa2 */
osMessageQueueId_t eventa2Handle;
const osMessageQueueAttr_t eventa2_attributes = {
  .name = "eventa2"
};
/* USER CODE BEGIN PV */
typedef enum{display_init,display1,display2A,display2B,display2C,display3,displayUART}state_t;
typedef enum{null,button,button2,dec,inc}event_t;
volatile int period=26;
//volatile uint16_t muestra_vieja=0;
volatile int x=0;
uint8_t xmax=0;
volatile uint8_t amplitud=100;
volatile uint32_t j_adc=0;
volatile uint8_t Flag_muestras=0;
uint16_t adc_buffer[ADC_BUFFER_SIZE];
uint32_t p_buffer[ADC_BUFFER_SIZE];
struct cmpx FFT_buffer[ADC_BUFFER_SIZE];
volatile int button_flag_counter=0;
volatile int button_flag_counter_ant=4;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void SignalGeneratorTask(void *argument);
void ScreenControlerTask(void *argument);
void InputControlerTask(void *argument);
void DataControlerTask(void *argument);

/* USER CODE BEGIN PFP */
void display(state_t *actual, event_t evt);
void action(state_t dsp);
void parche();
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef hadc1);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED, 1);
  HAL_TIM_Base_Start_IT(&htim2);

  SSD1306_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of eventa2 */
  eventa2Handle = osMessageQueueNew (7, sizeof(event_t), &eventa2_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SignalGenerator */
  SignalGeneratorHandle = osThreadNew(SignalGeneratorTask, NULL, &SignalGenerator_attributes);

  /* creation of ScreenControler */
  ScreenControlerHandle = osThreadNew(ScreenControlerTask, NULL, &ScreenControler_attributes);

  /* creation of InputControler */
  InputControlerHandle = osThreadNew(InputControlerTask, NULL, &InputControler_attributes);

  /* creation of DataControler */
  DataControlerHandle = osThreadNew(DataControlerTask, NULL, &DataControler_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 400000;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void callback_in(int tag) {
	switch (tag) {
	case 0:
		HAL_GPIO_WritePin(PIN0, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(PIN1, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(PIN2, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(PIN3, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(PIN4, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(PIN0, GPIO_PIN_SET);
		break;
	}
}

void callback_out(int tag) {
	switch (tag) {
	case 0:
		HAL_GPIO_WritePin(PIN0, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(PIN1, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(PIN2, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(PIN3, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(PIN4, GPIO_PIN_RESET);
		break;
	default:
		HAL_GPIO_WritePin(PIN0, GPIO_PIN_RESET);
		break;
	}
}

/*
void parche(){
	//if(adc_buffer[250]==muestra_vieja){
		for(int i=0;i<40;i++){
			adc_buffer[i+216]=0;
		}
	//}
	//muestra_vieja=adc_buffer[250];
}*/

void action(state_t dsp){
	int AMPL_ONDA=0;
	int AMPL_CURSOR=0;
	int FREC_ONDA=0;
	int FREC_CURSOR=0;
	char buffer_char[10];
	switch(dsp){
	case display_init:
		SSD1306_GotoXY(0,0);
		SSD1306_DrawFilledRectangle(2, 2, 124, 60, SSD1306_COLOR_WHITE);
		SSD1306_DrawFilledRectangle(4, 4, 120, 56, SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(6, 9);
		SSD1306_Puts("Analizador", &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(20, 25);
		SSD1306_Puts("espectral", &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(6, 50);
		SSD1306_Puts("->Ir a display 1", &Font_7x10, SSD1306_COLOR_WHITE);

		SSD1306_UpdateScreen();
		break;
	case display1:
		for(uint16_t i=0;i<ADC_BUFFER_SIZE;i++){
			p_buffer[i]=(uint32_t)(sqrt((FFT_buffer[i].real*FFT_buffer[i].real)+(FFT_buffer[i].imag*FFT_buffer[i].imag)));
		}
		FREC_CURSOR=500*x/128;
		AMPL_CURSOR=1650*p_buffer[x]/350;
		PrintBufferAsGraph(p_buffer,ADC_BUFFER_SIZE,AMPL_CURSOR,FREC_CURSOR,&xmax);
		SSD1306_DrawLine(x,0,x,63,SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();

	break;
	case display2A:
			  SSD1306_GotoXY(0, 0);
			  SSD1306_Puts("pantalla2", &Font_7x10, SSD1306_COLOR_WHITE);

			  SSD1306_DrawFilledRectangle(0, 20, 100, 10, SSD1306_COLOR_WHITE);
			  SSD1306_GotoXY(0, 20);
			  SSD1306_Puts("Amplitud", &Font_7x10, SSD1306_COLOR_BLACK);
			  SSD1306_GotoXY(0, 30);
			  SSD1306_Puts("Frecuencia", &Font_7x10, SSD1306_COLOR_WHITE);
			  SSD1306_GotoXY(0, 40);
			  SSD1306_Puts("Cursor", &Font_7x10, SSD1306_COLOR_WHITE);
			  break;
	case display2B:
			  SSD1306_GotoXY(0, 0);
			  SSD1306_Puts("pantalla2", &Font_7x10, SSD1306_COLOR_WHITE);

			  SSD1306_DrawFilledRectangle(0, 30, 100, 10, SSD1306_COLOR_WHITE);
			  SSD1306_GotoXY(0, 20);
			  SSD1306_Puts("Amplitud", &Font_7x10, SSD1306_COLOR_WHITE);
			  SSD1306_GotoXY(0, 30);
			  SSD1306_Puts("Frecuencia", &Font_7x10, SSD1306_COLOR_BLACK);
			  SSD1306_GotoXY(0, 40);
			  SSD1306_Puts("Cursor", &Font_7x10, SSD1306_COLOR_WHITE);
			  break;
	case display2C:
			  SSD1306_GotoXY(0, 0);
			  SSD1306_Puts("pantalla2", &Font_7x10, SSD1306_COLOR_WHITE);
			  SSD1306_DrawFilledRectangle(0, 40, 100, 10, SSD1306_COLOR_WHITE);
			  SSD1306_GotoXY(0, 20);
			  SSD1306_Puts("Amplitud", &Font_7x10, SSD1306_COLOR_WHITE);
			  SSD1306_GotoXY(0, 30);
			  SSD1306_Puts("Frecuencia", &Font_7x10, SSD1306_COLOR_WHITE);
			  SSD1306_GotoXY(0, 40);
			  SSD1306_Puts("Cursor", &Font_7x10, SSD1306_COLOR_BLACK);
			  break;
	case display3:
		FREC_CURSOR=500*x/128;
		AMPL_CURSOR=1650*p_buffer[x]/350;
		AMPL_ONDA=1650*p_buffer[xmax]/350;
		FREC_ONDA=500*xmax/128;

		ImprimirLinea("display 3:",1);
		SSD1306_GotoXY(0, 20);
		SSD1306_Puts("AMPL Onda       ", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(80, 20);
		sprintf(buffer_char, "%dmV ", AMPL_ONDA);
		SSD1306_Puts(buffer_char, &Font_7x10, SSD1306_COLOR_BLACK);

		SSD1306_GotoXY(0, 30);
		SSD1306_Puts("FREC Onda     ", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(80, 30);
		sprintf(buffer_char, "%d Hz",FREC_ONDA);
		SSD1306_Puts(buffer_char, &Font_7x10, SSD1306_COLOR_BLACK);

		SSD1306_GotoXY(0, 40);
		SSD1306_Puts("AMPL Cursor  ", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(80, 40);
		sprintf(buffer_char, "%dmV ", AMPL_CURSOR);
		SSD1306_Puts(buffer_char, &Font_7x10, SSD1306_COLOR_BLACK);

		SSD1306_GotoXY(0, 50);
		SSD1306_Puts("FREC Cursor  ", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(80, 50);
		sprintf(buffer_char, "%d Hz", FREC_CURSOR);
		SSD1306_Puts(buffer_char, &Font_7x10, SSD1306_COLOR_BLACK);
		break;

	case displayUART:
		taskENTER_CRITICAL();
		HAL_TIM_Base_Stop_IT(&htim2);
		for(int i=0;i<ADC_BUFFER_SIZE;i++){
			char datac1[5];
			char datac2[4];
			char datac3[3];
			char datac4[2];
			int datai=(int)1650*p_buffer[i]/350;
			if(datai>999){
				sprintf(datac1,"%d,",datai);
				HAL_UART_Transmit(&huart1, (uint8_t*)&datac1, sizeof(datac1),HAL_MAX_DELAY);
			}
			if(datai<1000 && datai>99){
				sprintf(datac2,"%d,",datai);
				HAL_UART_Transmit(&huart1, (uint8_t*)&datac2, sizeof(datac2),HAL_MAX_DELAY);
			}
			if(datai<100 && datai>9){
				sprintf(datac3,"%d,",datai);
				HAL_UART_Transmit(&huart1, (uint8_t*)&datac3, sizeof(datac3),HAL_MAX_DELAY);
			}
			if(datai<10){
				sprintf(datac4,"%d,",datai);
				HAL_UART_Transmit(&huart1, (uint8_t*)&datac4, sizeof(datac4),HAL_MAX_DELAY);
			}
		}
		HAL_TIM_Base_Start_IT(&htim2);
		taskEXIT_CRITICAL();
		break;
	default:
		break;
	}
}

void display(state_t *actual, event_t evt){
	switch(*actual){
	case display_init:
		if(evt==button){
			action(display1);
			*actual=display1;
		}
		break;
	case display1:
		switch(evt){
		case button:
			button_flag_counter_ant=4;
			action(display2A);
			*actual=display2A;
			break;
		case button2:
			action(displayUART);
			*actual=displayUART;
			break;
		case inc:
			switch(button_flag_counter_ant){
			case 0:
				amplitud++;
				if(amplitud>100){
					amplitud=100;
				}
				break;
			case 1:
				period++;
				break;
			case 2:
				x++;
				break;
			default:
				break;
			}
			action(display1);
			break;
		case dec:
			switch(button_flag_counter_ant){
			case 0:
				amplitud--;
				break;
			case 1:
				period--;
				break;
			case 2:
				x--;
				break;
			default:
				break;
			}
			action(display1);
			break;
		default:
			action(display1);
			break;
		}

	break;
	case display2A:
		button_flag_counter=button_flag_counter_ant;
		switch(evt){
		case button:
			SSD1306_Clear();
			action(display3);
			*actual=display3;
			break;
		case button2:
			action(displayUART);
			*actual=displayUART;
			break;
		case inc:
			button_flag_counter++;
			break;
		case dec:
			button_flag_counter--;
			break;
		default:
			break;
		}

		if (button_flag_counter < 0) {
			button_flag_counter = 2;
		}

		if (button_flag_counter > 2) {
				button_flag_counter = 0;
			}
		if(button_flag_counter_ant!=button_flag_counter){
			SSD1306_Clear();
		}

		switch(button_flag_counter){
		case 0:
			action(display2A);
			break;
		case 1:
			action(display2B);
			break;
		case 2:
			action(display2C);
			break;
		default:
			break;
		}
		if(button_flag_counter_ant!=button_flag_counter){
					SSD1306_UpdateScreen();
					button_flag_counter_ant=button_flag_counter;
				}
	break;
	case display3:
		switch(evt){
		case button:
			action(display1);
			*actual=display1;
			break;
		case button2:
			action(displayUART);
			*actual=displayUART;
			break;
		default:
			action(display3);
			break;
		}
	break;
	case displayUART:
		action(display1);
		*actual=display1;
		break;
	default:
		break;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_SignalGeneratorTask */
/**
  * @brief  Function implementing the SignalGenerator thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SignalGeneratorTask */
void SignalGeneratorTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	vTaskSetApplicationTaskTag(NULL, (void*)1);
	uint16_t onda[NSEN] = {0};
	int idx = 0;
	uint32_t period_anterior=0;
	uint8_t amplitud_anterior=0;
	uint32_t period_PWM=0;
  /* Infinite loop */
  for(;;)
  {
	  uint32_t stack_SGT=4*osThreadGetStackSpace(SignalGeneratorHandle);
	  if(period_anterior!=period || amplitud_anterior!=amplitud){
		  period_anterior=period;
		  amplitud_anterior=amplitud;
		  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
		  period_PWM=set_PWM_period(period);

		  __HAL_TIM_SET_AUTORELOAD(&htim3, period_PWM);
		  __HAL_TIM_SET_COUNTER(&htim3, 0);

		    for (idx = 0; idx < NSEN; idx++) {
		        onda[idx] = (uint16_t)((float)((float)amplitud/100)*(period_PWM+1)/ 2.0 * (sin(2.0 * M_PI * idx / (float)NSEN) + 1.0));
		    }

		    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)onda, NSEN);
	  }
	  osDelay(250);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ScreenControlerTask */
/**
* @brief Function implementing the ScreenControler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ScreenControlerTask */
void ScreenControlerTask(void *argument)
{
  /* USER CODE BEGIN ScreenControlerTask */
	vTaskSetApplicationTaskTag(NULL, (void*)2);
	uint32_t flag_SCT=0;
	event_t eventSCT=null;
	state_t estado=display_init;
	action(display_init);
	/* Infinite loop */
	for(;;){
		uint32_t stack_SCT=4*osThreadGetStackSpace(ScreenControlerHandle);
		flag_SCT=ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(flag_SCT==1){
			flag_SCT=0;
			if(osMessageQueueGetSpace(eventa2Handle)<7){
				osMessageQueueGet(eventa2Handle, &eventSCT, 0, 100);
			}
			else{
				eventSCT=null;
			}
			display(&estado,eventSCT);
			HAL_TIM_Base_Start_IT(&htim2);
		}
	osDelay(100);
	}
  /* USER CODE END ScreenControlerTask */
}

/* USER CODE BEGIN Header_InputControlerTask */

/**
* @brief Function implementing the InputControler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InputControlerTask */
void InputControlerTask(void *argument)
{
  /* USER CODE BEGIN InputControlerTask */
  /* Infinite loop */
	vTaskSetApplicationTaskTag(NULL, (void*)3);
	Encoder_Status St=Neutral;
	event_t event;
	uint32_t antirebote=0;
	uint32_t antirebote2=0;
  for(;;)
  {
	uint32_t stack_ICT=4*osThreadGetStackSpace(InputControlerHandle);
	event=null;
	St=Encoder_Get_Status();
	if (St==Incremented){
		event=inc;
	}
	if(St==Decremented){
		event=dec;
	}
	uint16_t SW_encoder = HAL_GPIO_ReadPin(ENCODER_BUTTON);
	if(antirebote==0){
	if(SW_encoder == 0){
		event=button;
		antirebote=1;
	}
	}
	antirebote=antirebote_ftn(antirebote);

	uint16_t SW_button = HAL_GPIO_ReadPin(BUTTON);
	if(antirebote2==0){
	if(SW_button == 1){
		antirebote2=1;
		event=button2;
	}
	}
	antirebote2=antirebote_ftn(antirebote2);

	if(event!=null){
		osMessageQueuePut(eventa2Handle, &event, 0,100);
	}
	osDelay(1);
  }
  /* USER CODE END InputControlerTask */
}

/* USER CODE BEGIN Header_DataControlerTask */
/**
* @brief Function implementing the DataControler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataControlerTask */
void DataControlerTask(void *argument)
{
  /* USER CODE BEGIN DataControlerTask */
	vTaskSetApplicationTaskTag(NULL, (void*)4);
  /* Infinite loop */
  for(;;){
	 if(Flag_muestras==1){
		 Flag_muestras=0;
		 uint32_t freeHeap = xPortGetFreeHeapSize();
		 uint32_t stack_DCT=4*osThreadGetStackSpace(DataControlerHandle);
			for(int j_fft=0;j_fft<ADC_BUFFER_SIZE;j_fft++){
				FFT_buffer[j_fft].real=(float)(3.3*adc_buffer[j_fft]/4095.0);
				FFT_buffer[j_fft].imag=0;
			}
			FFT(FFT_buffer,ADC_BUFFER_SIZE);
			xTaskNotify(ScreenControlerHandle, 1,eSetValueWithOverwrite);
	 }
	  osDelay(1);
  }
  /* USER CODE END DataControlerTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2){
	  HAL_GPIO_WritePin(PIN5,1);
	  if(j_adc>=ADC_BUFFER_SIZE-1){
		  j_adc=0;
		  Flag_muestras=1;
		  HAL_TIM_Base_Stop_IT(&htim2);
	  }
	  else{
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  uint16_t ADC_value = HAL_ADC_GetValue(&hadc1);
		  adc_buffer[j_adc]=ADC_value;
		  j_adc++;
	  }
	  HAL_GPIO_WritePin(PIN5,0);
  }
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
