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
#include "stm32f4xx_hal.h"
#include "lcd.h"
#include "esp8266.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
osThreadId Read_RTCHandle;
osThreadId LM35Handle;
osThreadId INTERNET_STATUSHandle;
osThreadId WIFI_TXHandle;
osThreadId SWMS_CONFIGHandle;
/* USER CODE BEGIN PV */
uint8_t Time_Date[8];
uint8_t Mem_Write_data[8]={0x1,0x2,0x6,0x3,0x02,0x01,0x24};
extern int k;
int c=0;
char I2c_Number_Buf[10];
char Temp_Value[2];
char Temp_value[30];
int temp,raw_adc;
char Rx_Buff[100];
uint8_t Tx_Buff[10];
int i;
extern int t;
int K;
int kk;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);
void TASK1_READRTC_WRITELCD_1SEC(void const * argument);
void TASK2_READLM35_WRITELCD_5SEC(void const * argument);
void TASK3_CHECK_INTERNET_STATUS(void const * argument);
void TASK4_WIFI_TX(void const * argument);
void TASK5_SWMS_CONFIG(void const * argument);
void task(int a,uint8_t *);

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
	char I2c_Number_Buf[10];
	char Temp_Value[2];
	char Temp_value[30];
	int temp,raw_adc;
	extern int ev;
	extern int x;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	RM_LCD_Init();
	/*RM_LCD_Goto(3,0);
	RM_LCD_PutStr("RTC Testcase");
	//HAL_Delay(1000);
	osDelay(1000);
	osDelay(1000);
	//HAL_Delay(1000);
	RM_LCD_Clear();
			//Date & Time Write in to RTC 
			HAL_I2C_Mem_Write(&hi2c1,(0x68<<1),0x00,1, (uint8_t *)&Mem_Write_data[0],1,1000);
		 HAL_I2C_Mem_Write(&hi2c1,(0x68<<1),0x01,1, (uint8_t *)&Mem_Write_data[1],1,1000);
		 HAL_I2C_Mem_Write(&hi2c1,(0x68<<1),0x02,1, (uint8_t *)&Mem_Write_data[2],1,1000);
		 HAL_I2C_Mem_Write(&hi2c1,(0x68<<1),0x03,1, (uint8_t *)&Mem_Write_data[3],1,1000);
		 HAL_I2C_Mem_Write(&hi2c1,(0x68<<1),0x04,1, (uint8_t *)&Mem_Write_data[4],1,1000);
		 HAL_I2C_Mem_Write(&hi2c1,(0x68<<1),0x05,1, (uint8_t *)&Mem_Write_data[5],1,1000);
		 HAL_I2C_Mem_Write(&hi2c1,(0x68<<1),0x06,1, (uint8_t *)&Mem_Write_data[6],1,1000);
		 
	while(1)
	{
				if((HAL_I2C_IsDeviceReady(&hi2c1,(0x68<<1),1,10))==HAL_OK)
				 {
								RM_LCD_Write_Str(0 ,1,"                ");
								sprintf(I2c_Number_Buf,"%d",0x68);
								RM_LCD_Write_Str(0 ,1,"DS1307 Found:");
								RM_LCD_Write_Str(13 ,1,I2c_Number_Buf);
								//HAL_Delay(1000);
								//HAL_Delay(1000);
								osDelay(1000);
								osDelay(1000);
								break;
				 }
				 else
				 {
					     RM_LCD_Write_Str(0 ,1,"RTC Not Found:");
							 //HAL_Delay(1000);
							 //HAL_Delay(500);
							 //continue;
							osDelay(1000);
							osDelay(1000);
								break;
				 }
	}
	RM_LCD_Clear();
	WiFi_Init();*/
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Read_RTC */
  osThreadDef(Read_RTC, TASK1_READRTC_WRITELCD_1SEC, osPriorityIdle, 0, 128);
  Read_RTCHandle = osThreadCreate(osThread(Read_RTC), NULL);

  /* definition and creation of LM35 */
  osThreadDef(LM35, TASK2_READLM35_WRITELCD_5SEC, osPriorityIdle, 0, 128);
  LM35Handle = osThreadCreate(osThread(LM35), NULL);

  /* definition and creation of INTERNET_STATUS */
  osThreadDef(INTERNET_STATUS, TASK3_CHECK_INTERNET_STATUS, osPriorityIdle, 0, 128);
  INTERNET_STATUSHandle = osThreadCreate(osThread(INTERNET_STATUS), NULL);

  /* definition and creation of WIFI_TX */
  osThreadDef(WIFI_TX, TASK4_WIFI_TX, osPriorityIdle, 0, 128);
  WIFI_TXHandle = osThreadCreate(osThread(WIFI_TX), NULL);

  /* definition and creation of SWMS_CONFIG */
  osThreadDef(SWMS_CONFIG, TASK5_SWMS_CONFIG, osPriorityIdle, 0, 128);
  SWMS_CONFIGHandle = osThreadCreate(osThread(SWMS_CONFIG), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : ENTER_SWITCH_Pin */
  GPIO_InitStruct.Pin = ENTER_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENTER_SWITCH_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

/* USER CODE BEGIN Header_TASK1_READRTC_WRITELCD_1SEC */
/**
* @brief Function implementing the Read_RTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK1_READRTC_WRITELCD_1SEC */
void TASK1_READRTC_WRITELCD_1SEC(void const * argument)
{
  /* USER CODE BEGIN TASK1_READRTC_WRITELCD_1SEC */
  /* Infinite loop */
  for(;;)
  {
			HAL_I2C_Mem_Read(&hi2c1,(0x68<<1),0x0,1, (uint8_t *)&Time_Date[0],1,1000);
HAL_I2C_Mem_Read(&hi2c1,(0x68<<1),0x01,1, (uint8_t *)&Time_Date[1],1,1000);
HAL_I2C_Mem_Read(&hi2c1,(0x68<<1),0x02,1, (uint8_t *)&Time_Date[2],1,1000);
HAL_I2C_Mem_Read(&hi2c1,(0x68<<1),0x03,1, (uint8_t *)&Time_Date[3],1,1000);
HAL_I2C_Mem_Read(&hi2c1,(0x68<<1),0x04,1, (uint8_t *)&Time_Date[4],1,1000);
HAL_I2C_Mem_Read(&hi2c1,(0x68<<1),0x05,1, (uint8_t *)&Time_Date[5],1,1000);
HAL_I2C_Mem_Read(&hi2c1,(0x68<<1),0x06,1, (uint8_t *)&Time_Date[6],1,1000);

 RM_LCD_Write_CMD(0X80);
sprintf(Temp_Value, "%02d",Time_Date[2]-6*(Time_Date[2]>>4));
 RM_LCD_PutStr(Temp_Value);
 RM_LCD_Write_DATA(':');
 sprintf(Temp_Value, "%02d",Time_Date[1]-6*(Time_Date[1]>>4));
 RM_LCD_PutStr(Temp_Value);
 RM_LCD_Write_DATA(':');
 sprintf(Temp_Value, "%02d",Time_Date[0]-6*(Time_Date[0]>>4));
		
 RM_LCD_PutStr(Temp_Value);
RM_LCD_Goto(0,1);
//RM_LCD_PutStr("DATE: ");
sprintf(Temp_Value, "%02d",Time_Date[4]-6*(Time_Date[4]>>4));
RM_LCD_PutStr(Temp_Value);
RM_LCD_Write_DATA('/');
sprintf(Temp_Value, "%02d",Time_Date[5]-6*(Time_Date[5]>>4));
RM_LCD_PutStr(Temp_Value);
RM_LCD_Write_DATA('/');
sprintf(Temp_Value, "%02d",Time_Date[6]-6*(Time_Date[6]>>4));
RM_LCD_PutStr(Temp_Value);

//osDelay(300);
  }
  /* USER CODE END TASK1_READRTC_WRITELCD_1SEC */
}

/* USER CODE BEGIN Header_TASK2_READLM35_WRITELCD_5SEC */
/**
* @brief Function implementing the LM35 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK2_READLM35_WRITELCD_5SEC */
void TASK2_READLM35_WRITELCD_5SEC(void const * argument)
{
  /* USER CODE BEGIN TASK2_READLM35_WRITELCD_5SEC */
  /* Infinite loop */
  for(;;)
  {
    // ADC
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,10);
		raw_adc=HAL_ADC_GetValue(&hadc1); // digtal value range is 0 to 4095;
		temp=(raw_adc/12); // ANALOG SUPPLY - Reference Volatage - 3.6V ;
		sprintf(Temp_value, "%d", temp);
		RM_LCD_Write_DATA(' ');
		RM_LCD_Goto(10,0);
		RM_LCD_PutStr("T:");
		RM_LCD_PutStr(Temp_value);
		RM_LCD_Put_Char(0xDF);
		RM_LCD_Put_Char('C');
		osDelay(300);
  }
  /* USER CODE END TASK2_READLM35_WRITELCD_5SEC */
}

/* USER CODE BEGIN Header_TASK3_CHECK_INTERNET_STATUS */
/**
* @brief Function implementing the INTERNET_STATUS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK3_CHECK_INTERNET_STATUS */
void TASK3_CHECK_INTERNET_STATUS(void const * argument)
{
	int k;
  /* USER CODE BEGIN TASK3_CHECK_INTERNET_STATUS */
  /* Infinite loop */
  for(;;)
  {
		Tx_Buff[0]=temp;
	for(i=0;i<8;i++)
	{
		Tx_Buff[i+1]=Time_Date[i];
	}
	HAL_I2C_Mem_Write(&hi2c1,(0x50<<1),0,1,Tx_Buff,5,1000);
	if(k==1)
	{
		HAL_I2C_Mem_Write(&hi2c1,(0x50<<1),0,1, (uint8_t *)&Rx_Buff,5,1000);
    osDelay(1800);
  }
}
  /* USER CODE END TASK3_CHECK_INTERNET_STATUS */
}

/* USER CODE BEGIN Header_TASK4_WIFI_TX */
/**
* @brief Function implementing the WIFI_TX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK4_WIFI_TX */
void TASK4_WIFI_TX(void const * argument)
{
  /* USER CODE BEGIN TASK4_WIFI_TX */
  /* Infinite loop */
  for(;;)
  {
		//ESP8266
			WiFi_Transmit();
			//osDelay(1);
  }
  /* USER CODE END TASK4_WIFI_TX */
}

/* USER CODE BEGIN Header_TASK5_SWMS_CONFIG */
/**
* @brief Function implementing the SWMS_CONFIG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TASK5_SWMS_CONFIG */
void TASK5_SWMS_CONFIG(void const * argument)
{
  /* USER CODE BEGIN TASK5_SWMS_CONFIG */
  /* Infinite loop */
  for(;;)
  {
		int n=30000000;
		RM_LCD_PutStr("   WELCOME  TO  ");
		RM_LCD_Write_CMD(0xc0);
		RM_LCD_PutStr(" KERNEL MASTER "); 
		while(n--)
		{
			while(!( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)))
		{
			RM_LCD_Write_CMD(0x01);
			RM_LCD_Goto(0,0);
			RM_LCD_PutStr("ENTER TIME AND ");
			RM_LCD_Goto(0,1);
			RM_LCD_PutStr("DATE  Y:DN  N:UP");

			while(1)
			{
				if(!( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)))
					{
							break;
					}
				if(!( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)))
				{
					RM_LCD_Write_CMD(0x01);
					RM_LCD_Write_CMD(0x80);
					RM_LCD_PutStr("HH:MM:SS");
					RM_LCD_Write_CMD(0xC0);
					RM_LCD_PutStr("00:00:00");
					task(23,&Mem_Write_data[0]);
					osDelay(1800);
					task(59,&Mem_Write_data[1]);
					osDelay(1800);
					task(59,&Mem_Write_data[2]);
					osDelay(1800);

					RM_LCD_Write_CMD(0X01);
					RM_LCD_Write_CMD(0X80);
					RM_LCD_PutStr("YY:MM:DD:WD");
					RM_LCD_Write_CMD(0XC0);
					RM_LCD_PutStr("00:00:00:00");
	
					task(100,&Mem_Write_data[6]);
					osDelay(1800);
					task(12,&Mem_Write_data[5]);
					osDelay(1800);
					task(31,&Mem_Write_data[4]);
					osDelay(1800);
					if(Mem_Write_data[3]>16)
					{
					Mem_Write_data[3]+=6;
					}
					task(7,&Mem_Write_data[3]);
					osDelay(1800);
					for(kk=0;kk<7;kk++)
					{
					Mem_Write_data[kk]=((Mem_Write_data[kk]/10)*16)+(Mem_Write_data[kk]%10); //CONVERTING HEXA DECIMAL INTO DECIMAL
					}
					//return 1;
    //osDelay(1);
				}
  /* USER CODE END TASK5_SWMS_CONFIG */
}
}
}
}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
