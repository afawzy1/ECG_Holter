/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

/* USER CODE BEGIN Includes */
#include "stm32_topway_16x2.h"
#include "RTC_user_init.h"
#include "string.h"

#define AQUIRED			((uint8_t)1)
#define SAVED			((uint8_t)2)
#define PAUSED			((uint8_t)3)

#define ADC_DATA_LGTH   (300)

#define KEY_UP_MSK		((uint8_t) 0x01)
#define KEY_DOWN_MSK	((uint8_t) 0x02)
#define KEY_OK_MSK		((uint8_t) 0x04)
#define KEY_C_MSK		((uint8_t) 0x08)
#define NO_KEY_MSK		((uint8_t) 0x00)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart6;

osThreadId SYSS_handlerHandle;
osThreadId DisplayLCDHandle;
osThreadId MonitorKeysHandle;
osThreadId WriteSDHandle;
osThreadId BLK_EN_TaskHandle;
osThreadId VBat_MonitorHandle;
osMessageQId BLK_EN_QHandle;
osMessageQId KeysCommandHandle;
osMessageQId LCDCommandHandle;

/* USER CODE BEGIN PV */
/* Type definitions ---------------------------------------------------------*/

typedef struct{
	uint16_t ADC_Data[ADC_DATA_LGTH];
	uint8_t Status;
}EKGData_S;

typedef enum{
	lcdstate_printtime,
	lcdstate_printmenu,

}lcdstate_E;

typedef enum{
	Recording,
	N_Recording
}RecStateDisplay_E;

typedef enum{
	SetRecordTime,
	StartOperation,
	SubRecTime,
	SubRecord
}MainMenuState_E;

typedef enum{
	StartRecording,
	StopRecording
}RecordingState_E;

/* Private variables ---------------------------------------------------------*/
static volatile uint16_t dma_buffer[ADC_DATA_LGTH];
EKGData_S EKGData;
static volatile uint16_t Vbat_value;
static uint8_t lcd_firstline[16] = "                ";
static uint8_t lcd_secondline[16] = "                ";
static RecStateDisplay_E RecStateDisplay = N_Recording;
static MainMenuState_E mainmenustate = SetRecordTime;
static MainMenuState_E premainmenustate = SetRecordTime;
static RecordingState_E recordingstate = StartRecording;
static uint8_t hrs,min;
static uint8_t keycmd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_RTC_Init(void);
void SYSS_MainTask(void const * argument);
void StartDispalyLCD(void const * argument);
void StartMonitorKeys(void const * argument);
void StartWriteSD(void const * argument);
void Start_BLKEN(void const * argument);
void Read_Vbat(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void MX_LCD_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
FATFS fileSystem;
FIL EKGFile;
uint8_t path[] = "EKG.txt\0";
UINT testBytes;
FRESULT res;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t indx = 0;
	if(SAVED == EKGData.Status)
	{
		for(indx = 0; indx < ADC_DATA_LGTH; indx++)
		{
			EKGData.ADC_Data[indx] = dma_buffer[indx];
		}
		EKGData.Status = AQUIRED;
/*		HAL_ADC_Stop(&hadc1);
		HAL_ADC_Stop_DMA(&hadc1);*/
	}
	else if(AQUIRED  == EKGData.Status)
	{
		LCD1602_print(" SYSTEM FAILURE ");
		_Error_Handler(__FILE__, __LINE__);
	}
	else
	{
		/*do nothing*/
	}
}

static void MX_LCD_Init(void)
{
    LCD1602_Begin4BIT(LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_EN_Pin,
                        LCD_D0_GPIO_Port, LCD_D0_Pin, LCD_D1_Pin, LCD_D2_Pin, LCD_D3_Pin);
    LCD1602_noCursor();
    LCD1602_noBlink();
    LCD1602_clear();
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
/*	RecStateDisplay = N_Recording;
	lcdstate_E lcdState = lcdstate_printtime;
	xQueueSend(LCDCommandHandle, (void *)&lcdState, ( TickType_t ) 0);*/
	RecStateDisplay = N_Recording;
	HAL_ADC_Stop(&hadc1);
    HAL_ADC_Stop_DMA(&hadc1);
    f_close(&EKGFile);

}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
  MX_FATFS_Init();
  memset(&(EKGData.ADC_Data), ((uint8_t)0), sizeof(EKGData.ADC_Data));
  MX_LCD_Init();
  SetSystemTime();
  LCD1602_clear();
  if(f_mount(&fileSystem,SDPath, 1) == FR_OK)
  {
    res = f_open(&EKGFile, (char*)path, FA_CREATE_ALWAYS | FA_WRITE );

  }
  else
  {
  	LCD1602_print(" SYSTEM FAILURE ");
  	 _Error_Handler(__FILE__, __LINE__);
  }
  EKGData.Status = SAVED;

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

  /* Create the thread(s) */
  /* definition and creation of SYSS_handler */
  osThreadDef(SYSS_handler, SYSS_MainTask, osPriorityNormal, 0, 128);
  SYSS_handlerHandle = osThreadCreate(osThread(SYSS_handler), NULL);

  /* definition and creation of DisplayLCD */
  osThreadDef(DisplayLCD, StartDispalyLCD, osPriorityIdle, 0, 128);
  DisplayLCDHandle = osThreadCreate(osThread(DisplayLCD), NULL);

  /* definition and creation of MonitorKeys */
  osThreadDef(MonitorKeys, StartMonitorKeys, osPriorityNormal, 0, 128);
  MonitorKeysHandle = osThreadCreate(osThread(MonitorKeys), NULL);

  /* definition and creation of WriteSD */
  osThreadDef(WriteSD, StartWriteSD, osPriorityHigh, 0, 128);
  WriteSDHandle = osThreadCreate(osThread(WriteSD), NULL);

  /* definition and creation of BLK_EN_Task */
  osThreadDef(BLK_EN_Task, Start_BLKEN, osPriorityIdle, 0, 128);
  BLK_EN_TaskHandle = osThreadCreate(osThread(BLK_EN_Task), NULL);

  /* definition and creation of VBat_Monitor */
  osThreadDef(VBat_Monitor, Read_Vbat, osPriorityIdle, 0, 128);
  VBat_MonitorHandle = osThreadCreate(osThread(VBat_Monitor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of BLK_EN_Q */
  osMessageQDef(BLK_EN_Q, 3, uint16_t);
  BLK_EN_QHandle = osMessageCreate(osMessageQ(BLK_EN_Q), NULL);

  /* definition and creation of KeysCommand */
  osMessageQDef(KeysCommand, 3, uint8_t);
  KeysCommandHandle = osMessageCreate(osMessageQ(KeysCommand), NULL);

  /* definition and creation of LCDCommand */
  osMessageQDef(LCDCommand, 3, lcdstate_E);
  LCDCommandHandle = osMessageCreate(osMessageQ(LCDCommand), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 251;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_SET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 18;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 1;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 50;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_SET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZER_Pin|BLC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_EN_Pin|LCD_D0_Pin|LCD_D1_Pin 
                          |LCD_D2_Pin|LCD_D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin BLC_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|BLC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYUP_Pin KEYDOWN_Pin */
  GPIO_InitStruct.Pin = KEYUP_Pin|KEYDOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYRIGHT_Pin KEYLEFT_Pin */
  GPIO_InitStruct.Pin = KEYRIGHT_Pin|KEYLEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_EN_Pin LCD_D0_Pin LCD_D1_Pin 
                           LCD_D2_Pin LCD_D3_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_EN_Pin|LCD_D0_Pin|LCD_D1_Pin 
                          |LCD_D2_Pin|LCD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* SYSS_MainTask function */
void SYSS_MainTask(void const * argument)
{
  /* init code for FATFS */
//  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
  lcdstate_E lcdState = lcdstate_printmenu;
  memcpy(lcd_firstline, "    Main Menu   ", 16);
  memcpy(lcd_secondline, "<-Set duration->",16);
  /* Infinite loop */
  for(;;)
  {
	  xQueueReceive( KeysCommandHandle, &keycmd, ( TickType_t ) 0 );
	  if(keycmd == KEY_C_MSK)
	  {
	    lcdState = (lcdState == lcdstate_printtime) ? lcdstate_printmenu : lcdstate_printtime;
	    xQueueSend(LCDCommandHandle, (void *)&lcdState, ( TickType_t ) 0);

	  }
	  else
	  {
	    switch(mainmenustate)
	    {
	    case SetRecordTime:
	    	if(mainmenustate != premainmenustate)
	    	{
	    		memcpy(lcd_firstline, "    Main Menu   ", 16);
	    		memcpy(lcd_secondline, "<-Set duration->",16);
	  	  	  premainmenustate = mainmenustate;
	    	}
	  	  if(
	  			  (KEY_UP_MSK == keycmd)
				  ||
				  (KEY_DOWN_MSK == keycmd)
			)
	  	  {
	  		mainmenustate = StartOperation;
	  	  }
	  	  else if (KEY_OK_MSK == keycmd)
	  	  {
	  		mainmenustate = SubRecTime;
	  	  }
	  	  else
	  	  {

	  	  }
	  	  break;
	    case StartOperation:
	    	if(mainmenustate != premainmenustate)
	    	{
	    		memcpy(lcd_firstline, "    Main Menu   ", 16);
	    		memcpy(lcd_secondline, "<- Operation  ->",16);
	    		premainmenustate = mainmenustate;
	    	}
	  	  if(
	  			  (KEY_UP_MSK == keycmd)
				  ||
				  (KEY_DOWN_MSK == keycmd)
			)
	  	  {
	  		mainmenustate = SetRecordTime;
	  	  }
	  	  else if (KEY_OK_MSK == keycmd)
	  	  {
	  		mainmenustate = SubRecord;
	  	  }
	  	  else
	  	  {

	  	  }
	  	  break;
	    case SubRecord:
	    	if(mainmenustate != premainmenustate)
	    	{
		    	memcpy(lcd_firstline, "   Operation    ",16);
		    	memcpy(lcd_secondline, "<-   Start    ->",16);
	    		premainmenustate = mainmenustate;
	    	}
	    	if (
	    			(KEY_UP_MSK == keycmd)
					||
					(KEY_DOWN_MSK == keycmd)
			   )
	    	{
	    		recordingstate = (recordingstate == StartRecording) ? StopRecording : StartRecording;
		    	if(recordingstate == StartRecording)
		    	{
		    		memcpy(lcd_secondline, "<-   Start    ->",16);
		    	}
		    	else if(recordingstate == StopRecording)
		    	{
		    		memcpy(lcd_secondline, "<-    Stop    ->",16);
		    	}
		    	else
		    	{
		    		/*do nothing*/
		    	}
	    	}
	    	else if(KEY_OK_MSK == keycmd)
	    	{
	    		mainmenustate = StartOperation;
	    		lcdState = lcdstate_printtime;
	    		xQueueSend(LCDCommandHandle, (void *)&lcdState, ( TickType_t ) 0);
		    	if(recordingstate == StartRecording)
		    	{
		    		RecStateDisplay = Recording;
		    		HAL_ADC_Start(&hadc1);
		    		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buffer, ADC_DATA_LGTH);
		    	}
		    	else if(recordingstate == StopRecording)
		    	{
		    		RecStateDisplay = N_Recording;
		    		HAL_ADC_Stop(&hadc1);
		    	    HAL_ADC_Stop_DMA(&hadc1);
		    	    f_close(&EKGFile);
		    	}
		    	else
		    	{
		    		/*do nothing*/
		    	}
	    	}
	    	else
	    	{
	    		/*do nothing*/
	    	}
	    	break;
	    case SubRecTime:
	    	if(mainmenustate != premainmenustate)
	    	{
		    	memcpy(lcd_firstline, "  Set duration  ",16);
		    	memcpy(lcd_secondline, "                ",16);
	    		premainmenustate = mainmenustate;
	    	}
	    	if (KEY_UP_MSK == keycmd)
	    	{
	    		min ++;
	    		if(min == 60)
	    		{
	    			min = 0;
	    			hrs ++;
	    			if(hrs == 24)
	    			{
	    				hrs = 0;
	    			}
	    			else
	    			{
	    				/*do nothing*/
	    			}
	    		}
	    		else
	    		{
	    			/*do nothing*/
	    		}
	    	}
	    	else if(KEY_DOWN_MSK == keycmd)
	    	{
	    		if(min == 0)
	    		{
	    			min = 59;
	    			if(hrs == 0)
	    			{
	    				hrs = 23;
	    			}
	    			else
	    			{
	    				hrs --;
	    			}
	    		}
	    		else
	    		{
	    			min --;
	    		}
	    	}
	    	else if(KEY_OK_MSK == keycmd)
	    	{
	    		mainmenustate = SetRecordTime;
	    		/*Initialize the Alarm here*/
	    		InitializeAlarm(hrs, min);
	    	}
	    	else
	    	{
	    		/*do nothing*/
	    	}
	    	sprintf(lcd_secondline,"<- %.2d:%.2d:00  ->", hrs, min  );
	    	lcd_secondline[15] = ' ';
	    	break;
	    default:
	  	  break;
	    }
	  }
	  keycmd = NO_KEY_MSK;
	  osDelay(50);
  }
  /* USER CODE END 5 */ 
}

/* StartDispalyLCD function */
void StartDispalyLCD(void const * argument)
{
  /* USER CODE BEGIN StartDispalyLCD */
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	lcdstate_E lcdState = lcdstate_printmenu;
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive( LCDCommandHandle, &lcdState, ( TickType_t ) 0 ))
	  {
		  LCD1602_clear();
	  }
	  else
	  {
		 /* do nothing*/
	  }
	  switch(lcdState)
	  {
	  case lcdstate_printtime:
		  if(RecStateDisplay == Recording)
		  {
			  LCD1602_setCursor(1,4);
			  LCD1602_print("Recording  ");
		  }
		  else if(RecStateDisplay == N_Recording)
		  {
			  LCD1602_setCursor(1,4);
			  LCD1602_print("StandBy   ");
		  }
		  else
		  {
			 /* do nothing*/
		  }
		  HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
		  LCD1602_setCursor(2,5);
		  LCD1602_PrintInt((int)(sTime.Hours), (uint8_t)2);
		  LCD1602_print(":");
		  LCD1602_PrintInt((int)(sTime.Minutes), (uint8_t)2);
		  LCD1602_print(":");
		  LCD1602_PrintInt((int)(sTime.Seconds), (uint8_t)2);
		  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		  break;
	  case lcdstate_printmenu:
	  	  LCD1602_1stLine();
	  	  LCD1602_print((char *)lcd_firstline);
	  	  LCD1602_2ndLine();
	  	  LCD1602_print((char *)lcd_secondline);
	  default:
		  break;
	  }
		
    osDelay(200);
  }
  /* USER CODE END StartDispalyLCD */
}

/* StartMonitorKeys function */
void StartMonitorKeys(void const * argument)
{
  /* USER CODE BEGIN StartMonitorKeys */
	uint16_t tenSecDelay = 10000;
	uint8_t keysState = NO_KEY_MSK;
  /* Infinite loop */
  for(;;)
  {
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin)){
			keysState |=KEY_UP_MSK;
		}
		else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(KEYUP_GPIO_Port, KEYUP_Pin)){
			keysState &= (~KEY_UP_MSK);
		}
		else{
			/*do nothing*/
		}

		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(KEYDOWN_GPIO_Port, KEYDOWN_Pin)){
			keysState |=KEY_DOWN_MSK;
		}
		else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(KEYDOWN_GPIO_Port, KEYDOWN_Pin)){
			keysState &= (~KEY_DOWN_MSK);
		}
		else{
			/*do nothing*/
		}

		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(KEYRIGHT_GPIO_Port, KEYRIGHT_Pin)){
			keysState |=KEY_OK_MSK;
		}
		else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(KEYRIGHT_GPIO_Port, KEYRIGHT_Pin)){
			keysState &= (~KEY_OK_MSK);
		}
		else{
			/*do nothing*/
		}

		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(KEYLEFT_GPIO_Port, KEYLEFT_Pin)){
			keysState |=KEY_C_MSK;
		}
		else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(KEYLEFT_GPIO_Port, KEYLEFT_Pin)){
			keysState &= (~KEY_C_MSK);
		}
		else{
			/*do nothing*/
		}
		
		if(NO_KEY_MSK != keysState)
		{
			xQueueReset(BLK_EN_QHandle);
			xQueueSend(BLK_EN_QHandle, (void *)&tenSecDelay, ( TickType_t ) 0);
			xQueueSend(KeysCommandHandle, (void *)&keysState, ( TickType_t ) 0);
		}
		else
		{
			/*do nothing*/
		}
		
    osDelay(200);
  }
  /* USER CODE END StartMonitorKeys */
}

/* StartWriteSD function */
void StartWriteSD(void const * argument)
{
  /* USER CODE BEGIN StartWriteSD */
	RTC_TimeTypeDef sTime;
	uint16_t i;
	char msg1[30];
	int length;

	HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
	length = sprintf(msg1, "EKG Holter Data \n");
	f_write(&EKGFile, (void *)msg1, length, &testBytes);
	length = sprintf(msg1, "Recording Started at : ");
	f_write(&EKGFile, (void *)msg1, length, &testBytes);
	length = sprintf(msg1,"%.2d:", (sTime.Hours));
	f_write(&EKGFile, (void *)msg1, length, &testBytes);
	length = sprintf(msg1,"%.2d:", (sTime.Minutes));
	f_write(&EKGFile, (void *)msg1, length, &testBytes);
	length = sprintf(msg1,"%.2d\n", (sTime.Seconds));
	f_write(&EKGFile, (void *)msg1, length, &testBytes);

  /* Infinite loop */
  for(;;)
  {
	  if( AQUIRED == EKGData.Status)
	  {
	  	for(i= 0; i < ADC_DATA_LGTH; i++)
	  	{
	  		length = sprintf(msg1,"%.4d\n",EKGData.ADC_Data[i]);
	  		res = f_write(&EKGFile, (void *)msg1, length, &testBytes);
	  	}
  		f_sync(&EKGFile);
  		EKGData.Status = SAVED;
	  }
	else
	{
		/*do nothing*/
	}
	osDelay(1);
  }

  /* USER CODE END StartWriteSD */
}

/* Start_BLKEN function */
void Start_BLKEN(void const * argument)
{
  /* USER CODE BEGIN Start_BLKEN */
	uint16_t blkTimeOn;
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOA, BLC_Pin, GPIO_PIN_RESET);
	  while(!xQueueReceive( BLK_EN_QHandle, &blkTimeOn, ( TickType_t ) 0 ));
	  HAL_GPIO_WritePin(GPIOA, BLC_Pin, GPIO_PIN_SET);
	  osDelay((uint32_t)blkTimeOn);
  }
  /* USER CODE END Start_BLKEN */
}

/* Read_Vbat function */
void Read_Vbat(void const * argument)
{
  /* USER CODE BEGIN Read_Vbat */
  /* Infinite loop */
  for(;;)
  {
		HAL_ADCEx_InjectedStart(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		Vbat_value = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		HAL_ADCEx_InjectedStop(&hadc1);
    osDelay(500);
  }
  /* USER CODE END Read_Vbat */
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
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
