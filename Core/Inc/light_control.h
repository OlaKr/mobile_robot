#ifndef INC_LIGHT_CONTROL_H_
#define INC_LIGHT_CONTROL_H_
#include "main.h"

extern float BH1750_lux_sub;
extern float change;

void setMax(void);
void proportionalPID(void);

#endif /* INC_LIGHT_CONTROL_H_ */
