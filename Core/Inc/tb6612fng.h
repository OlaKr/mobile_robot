#ifndef INC_DRV8835_H_
#define INC_DRV8835_H_
#include "main.h"

typedef enum
{
	In_In_Mode=0,
	Phase_Enable_Mode=1
}TB6612_Mode;

typedef enum
{
	CW=0,
	CCW=1
}TB6612_Direction;

void tb6612_init();
void mode_control(TB6612_Mode);
void set_motorA_direction(TB6612_Direction);
void set_motorA_speed(uint8_t);

#endif /* INC_DRV8835_H_ */
