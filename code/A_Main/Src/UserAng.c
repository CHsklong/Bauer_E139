/*
 * UserAng.c
 *
 *  Created on: 2015313
 *      Author: admin
 */


#include "SysMain.h"

float AngOffSet_U = 0;
float AngOffSet_V = 0;
s16 AngBordInfo = 0;

AdcChannelTypeDef AngVar_Vbus;

void ADC_CHANNEL_INIT(void)
{
	memset_16(&AngVar_Vbus.iReg,0,sizeof_16(AdcChannelTypeDef));
	AngVar_Vbus.Gain = (VOLT_BUS_HD_GAIN*3.3)/4096;
	AngVar_Vbus.Offset = -(0.022f*4096)/3.3f;//
	AngVar_Vbus.Kn_1 = 1.0f/(1+2*PI*T_PWM*2*16000);
	AngVar_Vbus.Kn = 1.0f - AngVar_Vbus.Kn_1;
	AngVar_Vbus.LpKn_1 = 1.0f/(1+2*PI*T_PWM*240);
	AngVar_Vbus.LpKn = 1.0f - AngVar_Vbus.LpKn_1;
}

void ADC_LP_MACRO(AdcChannelTypeDef *dp)
{
		dp->tTemp =  dp->iReg  - dp->Offset;
		dp->Out =  dp->Out*dp->Kn_1  +  dp->tTemp * dp->Gain * dp->Kn;
		dp->OutLp =  dp->OutLp*dp->LpKn_1  +  dp->tTemp * dp->Gain * dp->LpKn;
}
