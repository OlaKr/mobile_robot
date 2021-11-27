#include "tb6612fng.h"
#include "main.h"


void set_motorA(int8_t speed)
{
	if (speed>=0)
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, RESET);
	}
	else
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, SET);
	}

	if(speed>=htim1.Instance->ARR)
		speed=htim1.Instance->ARR;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);
}

void set_motorB(int8_t speed)
{
	if (speed>=0)
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, RESET);
	}
	else
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, SET);
	}

	if(speed>=htim1.Instance->ARR)
		speed=htim1.Instance->ARR;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, speed);
}

void set_motorC(int8_t speed)
{
	if (speed>=0)
	{
		HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, SET);
		HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, RESET);
	}
	else
	{
		HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, RESET);
		HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, SET);
	}

	if(speed>=htim3.Instance->ARR)
		speed=htim3.Instance->ARR;
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, speed);
}

void set_motorD(int8_t speed)
{
	if (speed>=0)
	{
		HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
	}

	if(speed>=htim3.Instance->ARR)
		speed=htim3.Instance->ARR;
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, speed);
}

void TB6612_init(int8_t speedA, int8_t speedB, int8_t speedC, int8_t speedD)
{
	set_motorA(speedA);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	set_motorB(speedB);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	set_motorC(speedC);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	set_motorD(speedD);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}
