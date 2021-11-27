#include "light_control.h"
#include "main.h"

#define MIN_SUB -500
#define MAX_SUB 500
#define MAX_V 50

void ustawmax(){
	if(BH1750_lux_sub>MAX_SUB) BH1750_lux_sub=MAX_SUB;
	if(BH1750_lux_sub<MIN_SUB) BH1750_lux_sub=MIN_SUB;
}

void przeliczenie(){
	float roznicadla1 = (float)MAX_V/(float)MAX_SUB;					//wartosc w prędkosci dla 1lux
	zmiana = BH1750_lux_sub*roznicadla1;		//wartosc w predkosci dla SUB
}
