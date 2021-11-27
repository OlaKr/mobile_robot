#ifndef INC_PID_H_
#define INC_PID_H_
#include "main.h"

typedef struct
{
	int previous_error;			//poprzedni błąd dla członu różniczkującego
	int total_error;			//suma uchybów dla członu całkującego
	float Kp;					//wzmocnienie członu proporcjonalnego
	float Ki;					//wzmocnienie członu całkującego*/
	float Kd;					//wzmocnienie członu różniczkującego*/
	int anti_windup_limit;		//Anti-Windup - ograniczenie członu całkującego
}pid_str;

