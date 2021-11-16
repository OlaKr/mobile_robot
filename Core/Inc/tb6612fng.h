#ifndef INC_TB6612_H_
#define INC_TB6612_H_
#include "main.h"

typedef enum
{
	CW=0,
	CCW=1
}TB6612_Direction;

void tb6612_init(TB6612_Direction, TB6612_Direction, uint8_t, uint8_t);
void set_motorA_direction(TB6612_Direction);
void set_motorA_speed(uint8_t);


#endif /* INC_TB6612_H_ */
