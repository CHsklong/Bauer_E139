/*
 * MathPidPos.c
 *
 *  Created on: 2015年3月18日
 *      Author: admin
 */


#include "SysMain.h"

PI_POS gPidPos;



void PI_POS_INIT(void)
{
	memset_16(&gPidPos.Ref,0,sizeof_16(PI_POS));
	gPidPos.Umax = 2.1f;
	gPidPos.Umin = -2.1f;
	gPidPos.VfGain = 0.0f;
	gPidPos.KVf = 1.0f/((BASE_SPD_RPM/60.0f) * SPEED_LOOP_PSR * T_PWM);
	gPidPos.Inc = 0.0f;
	gPidPos.PFlt = 1.0f/200;
	gPidPos.QepDline = 1.0f/(MotoPar.ENCL*4);
	gPidPos.CirLine = (MotoPar.ENCL*4)*ENCODER_CIRCLE;
}


#if 0
void PI_POS_MACRO(PI_POS *dp)
{
	//速度前馈计算
	dp->SpdGn = dp->Inc * dp->KVf;
	dp->VfTemp =  dp->SpdGn * dp->VfGain;

	dp->Tmp = dp->VfTemp - dp->VfOut;

	if(dp->Tmp > dp->PFlt) dp->VfOut = dp->VfOut + dp->PFlt;
	else if(dp->Tmp < -dp->PFlt) dp->VfOut = dp->VfOut - dp->PFlt;
	else dp->VfOut = dp->VfTemp;

	dp->Err = dp->Ref - dp->Fbk;
	if(dp->Err > _IQ(ENCODER_CIRCLE/2)) dp->Err = dp->Err - _IQ(ENCODER_CIRCLE);
	else if(dp->Err < _IQ(-ENCODER_CIRCLE/2)) dp->Err = dp->Err + _IQ(ENCODER_CIRCLE);

 	dp->Up = dp->Kp * dp->Err;

	dp->OutPreSat = dp->Up + dp->VfOut;

	if(dp->OutPreSat > dp->Umax){
		dp->Out = dp->Umax;
	}else if(dp->OutPreSat < dp->Umin){
		dp->OutPreSat = dp->Umin;
	}else{
		dp->Out = dp->OutPreSat;
	}

}
#endif
