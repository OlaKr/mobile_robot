#include "tb6612fng.h"
#include "main.h"


void mode_control(TB6612_Mode mode)
{
	if(mode==Phase_Enable_Mode)
		HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, SET);
	else if(mode==In_In_Mode)
		HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, RESET);
}


void set_motorA_direction(TB6612_Direction dir)
{
	if (dir==CW)
		HAL_GPIO_WritePin(APHASE_GPIO_Port, APHASE_Pin, SET);
	else if (dir==CCW)
		HAL_GPIO_WritePin(APHASE_GPIO_Port, APHASE_Pin, RESET);
}

void set_motorA_speed(uint8_t speed)
{
	if(speed>=htim1.Instance->ARR)
		speed=htim1.Instance->ARR;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);
}


void tb6612_init()
{
	mode_control(Phase_Enable_Mode);
	set_motorA_direction(CCW);
	set_motorA_speed(0);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

