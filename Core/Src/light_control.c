#include "light_control.h"
#include "main.h"

#define MIN_SUB -500
#define MAX_SUB 500
#define MAX_V 50

void setMax(){
	if(BH1750_lux_sub>MAX_SUB) BH1750_lux_sub=MAX_SUB;
	if(BH1750_lux_sub<MIN_SUB) BH1750_lux_sub=MIN_SUB;
}

void proportionalPID(){
	float sub_for_one_lux = (float)MAX_V/(float)MAX_SUB;					//wartosc w prędkosci dla 1lux
	change = BH1750_lux_sub*sub_for_one_lux;								//wartosc w predkosci dla SUB
}
