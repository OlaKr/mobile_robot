/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "tb6612fng.h"
#include "bh1750.h"
#include "light_control.h"
#include "wave_file.h"
#include "wave_player.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	GPIO_TypeDef* port;
	uint16_t pin;
}button_nr;

typedef enum
{
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	STOP,
	SLOW,
	FAST,
	LIGHT,
	PLAY,
	CLOSE
}control;

typedef enum
{
	//PLAY,
	END
}music;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINE_MAX_LENGTH 80

#define BUFFER_LEN  1

#define V_SLOW  40
#define V_FAST  99
#define V_AVERAGE  80
#define V_ZERO  0
#define V_LIGHT 49

#define MIN_LUX 30

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int len;
char buffer[100];

uint8_t RX_BUFFER[BUFFER_LEN] = {0};

float BH1750_lux;
float BH1750_lux2;
float BH1750_lux_sub;
float change;

extern uint8_t audio_file[];

control setCommand = CLOSE;
music setPlay = END;

uint8_t counter = 0;
//uint16_t timer_val;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int __io_putchar(int sign)
{
	if(sign=='\n')
	{
		uint8_t sign2='\r';
		HAL_UART_Transmit(&huart1, &sign2, 1, HAL_MAX_DELAY);
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)&sign, 1, HAL_MAX_DELAY);
	return 1;
}


static char line_buffer[LINE_MAX_LENGTH+1];
static uint32_t line_length;

void line_append(uint8_t value)
{
	if(value=='\r'||value=='\n')
	{
		if(line_length>0)
		{
			line_buffer[line_length]='\0';
			line_length=0;
		}
	}
	else
	{
		if(line_length>=LINE_MAX_LENGTH)
		{
			line_length=0;
		}
		line_buffer[line_length++]=value;
	}
}


uint8_t uart_rx_buffer;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		line_append(uart_rx_buffer);

	}
	HAL_UART_Receive_IT(&huart1, &uart_rx_buffer,1);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim==&htim2)
	{
		if (strcmp(line_buffer, "on")==0){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			printf("Command: %s\n", line_buffer);
		}
		else if(strcmp(line_buffer, "off")==0){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			printf("Command: %s\n", line_buffer);
		}
		else if(strcmp(line_buffer, "w")==0){
			setCommand = SLOW;
		}
		else if(strcmp(line_buffer, "s")==0){
			setCommand = STOP;
		}
		else if(strcmp(line_buffer, "t")==0){
			setCommand = FAST;
		}
		else if(strcmp(line_buffer, "f")==0){
			setCommand = FORWARD;
		}
		else if(strcmp(line_buffer, "b")==0){
			setCommand = BACKWARD;
		}
		else if(strcmp(line_buffer, "l")==0){
			setCommand = LEFT;
		}
		else if(strcmp(line_buffer, "r")==0){
			setCommand = RIGHT;
		}
		//else printf("Unrecognized command: %s\n", line_buffer);
	}

	if(htim==&htim5)
	{
		if(BH1750_OK == BH1750_ReadLight(&BH1750_lux))
		{
			//sprintf(buffer,"BH1750 Lux: %.2f\r\n", BH1750_lux);
			len=strlen(buffer);
			HAL_UART_Transmit(&huart1,buffer,len,100);
		}
		if(BH1750_OK == BH1750_ReadLight2(&BH1750_lux2))
		{
			//sprintf(buffer,"BH1750 Lux2: %.2f\r\n", BH1750_lux2);
			len=strlen(buffer);
			HAL_UART_Transmit(&huart1,buffer,len,100);
		}
	}

	if(htim==&htim9)
	{
		BH1750_lux_sub=BH1750_lux2-BH1750_lux;
		//sprintf(buffer,"SUB: %.2f\r\n", BH1750_lux_sub);
		if(strcmp(line_buffer, "i")==0)
		{
			setCommand = PLAY;
		}
//		if(strcmp(line_buffer, "i")==0)
//		{
//			setCommand = PLAY;
//		}


	}
}

void robotControl()
{
	if(FORWARD==setCommand)
	{
		TB6612_init(V_FAST,V_FAST,V_FAST,V_FAST);
	}
	else if(BACKWARD==setCommand)
	{
		TB6612_init(-V_FAST,-V_FAST,-V_FAST,-V_FAST);
	}
	else if(LEFT==setCommand)
	{
		TB6612_init(V_ZERO,V_AVERAGE,V_AVERAGE,V_ZERO);
	}
	else if(RIGHT==setCommand)
	{
		TB6612_init(V_AVERAGE,V_ZERO,V_ZERO,V_AVERAGE);
	}
	else if(STOP==setCommand)
	{
		TB6612_init(V_ZERO,V_ZERO,V_ZERO,V_ZERO);
	}
	else if(SLOW==setCommand)
	{
		TB6612_init(V_SLOW,V_SLOW,V_SLOW,V_SLOW);
	}
	else if(FAST==setCommand)
	{
		TB6612_init(V_FAST,V_FAST,V_FAST,V_FAST);
	}
}

void followTheLight(){
	if(BH1750_lux>MIN_LUX && BH1750_lux2>MIN_LUX)
	{
		TB6612_init(V_LIGHT-change,V_LIGHT+change,V_LIGHT+change,V_LIGHT-change);
	}
	else
	{
		TB6612_init(V_ZERO,V_ZERO,V_ZERO,V_ZERO);
	}
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
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1,&uart_rx_buffer,1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim9);
  //HAL_TIM_Base_Start_IT(&htim8);

  BH1750_Init(&hi2c1, &hi2c2);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);

  setMax();

  wave_player_init(&htim6, &hdac);
  //wave_player_start(audio_file);

  //HAL_TIM_Base_Start(&htim8);
  //timer_val = __HAL_TIM_GET_COUNTER(&htim8);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //wave_player_start(audio_file);
	  //HAL_Delay(2000);
//	  if(__HAL_TIM_GET_COUNTER(&htim8) - timer_val >= 10000)
//	  {
//		  //wave_player_start(audio_file);
//		  //HAL_Delay(2000);
//		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		  timer_val = __HAL_TIM_GET_COUNTER(&htim8);
//	  }


	  if(PLAY==setCommand)
	  {
		  //wave_player_start(audio_file);
		  HAL_Delay(2000);
		  setCommand=CLOSE;
	  }

		  //printf("HAHAHAHHAHA\n");

//	  if(counter==1)
	  printf("Time: %02d:%02d:%02d\n", time.Hours, time.Minutes, time.Seconds);
	  HAL_Delay(200);



	  robotControl();

	  proportionalPID();
	  if(setCommand==LIGHT) followTheLight();


		  //wave_player_start(audio_file);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	wave_player_prepare_half_buffer(SECOND_HALF_OF_BUFFER);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	wave_player_prepare_half_buffer(FIRST_HALF_OF_BUFFER);
}
//void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
//{
//	UNUSED(hrtc);
//	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//
//	//wave_player_start(audio_file);
//	//HAL_Delay(2000);
//
//}

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
