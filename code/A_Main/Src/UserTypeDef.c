/*
 * UserTypeDef.c
 *
 *  Created on: 20151217
 *      Author: admin
 */


#include "SysMain.h"

//

unsigned int ISR_FREQUENCY = 8; 				// Define the ISR frequency (kHz)
unsigned int  SPEED_LOOP_PSR = 1;				// 
float  T_PWM = (0.000125f);							//(0.001/ISR_FREQUENCY)    // Samping period (sec), see parameter.h
unsigned int  SVPWM_PWM_PERIOD = 15000;	//SYSTEM_FREQUENCY*1000000*T_PWM/2;	//PWM
unsigned int 	SVPWM_PWM_DEDTIME = 252;


void gSysTypeDefVarInit(void)
{

	ISR_FREQUENCY = INT_FREQ;
	SPEED_LOOP_PSR = ISR_FREQUENCY/POS_LOOP_FRE;
/*	
	if(SysParP9.GBIT2 % 10 == 1)
	{
		T_PWM = (0.001F/(ISR_FREQUENCY-0.1F));
	}else if(SysParP9.GBIT2 % 10 == 2)
	{
		T_PWM = (0.001F/(ISR_FREQUENCY+0.1F));
	}else{
		T_PWM =  0.001F/ISR_FREQUENCY;
	}
*/
	T_PWM =  0.001F/ISR_FREQUENCY;
	//T_PWM = 0.000125f;
	SVPWM_PWM_PERIOD = ((u32)SYSTEM_FREQUENCY*1000000uL)*T_PWM/2;	//PWM
	//SVPWM_PWM_DEDTIME = DEADTIME_DTG;//202;//(u32)SYSTEM_FREQUENCY*DEADTIME_NS*0.001f;
}

