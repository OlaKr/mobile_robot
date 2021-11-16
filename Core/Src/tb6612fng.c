#include "tb6612fng.h"
#include "main.h"


void set_motorA_direction(TB6612_Direction dir)
{
	if (dir==CW)
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, RESET);
	}
	else if (dir==CCW)
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, SET);
	}
}

void set_motorB_direction(TB6612_Direction dir)
{
	if (dir==CW)
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, RESET);
	}
	else if (dir==CCW)
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, SET);
	}
}

void set_motorA_speed(uint8_t speed)
{
	if(speed>=htim1.Instance->ARR)
		speed=htim1.Instance->ARR;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);
}

void set_motorB_speed(uint8_t speed)
{
	if(speed>=htim1.Instance->ARR)
		speed=htim1.Instance->ARR;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, speed);
}

void tb6612_init(TB6612_Direction dirA, TB6612_Direction dirB, uint8_t speedA, uint8_t speedB)
{
	set_motorA_direction(dirA);
	set_motorA_speed(speedA);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	set_motorB_direction(dirB);
	set_motorB_speed(speedB);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
















