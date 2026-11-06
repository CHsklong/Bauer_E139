/*
 * UserPulse.c
 *
 *  Created on: 2015321
 *      Author: admin
 *       
 */

#include "SysMain.h"


PulseTypedef gPulse;

void gPulseInit(void)
{
	memset_16(&gPulse.PwmEn,0,sizeof_16(PulseTypedef));
	gPulse.Grn = SysParP3.GRN1;
	gPulse.Grm = SysParP3.GRM1;
	gPulse.FiltLen = SysParP3.PFLT;
	gPulse.FltClrIndex = PULSE_FILT_LEN;
	gPulse.QepLine =(MotoPar.ENCL*4);
	gPulse.QepDline = 1.0f/(MotoPar.ENCL*4);
	gPulse.MaxFre = (SysParP0.MSPD * 0.00033333333f * 50.0f * gPulse.QepLine * 0.001f * 1* 2.1f)/(POS_LOOP_FRE*1);
	if(MotoPar.ENCT%100 == ENC_ABS_17BIT){
		gPulse.GrnScale = 8192;
		gPulse.GrmScale = 625;
	}else if(MotoPar.ENCT%100 == ENC_ABS_20BIT){
		gPulse.GrnScale = 65536;
		gPulse.GrmScale = 625;

	}else if(MotoPar.ENCT%100 == ENC_ABS_23BIT){
		gPulse.GrnScale = 524288;
		gPulse.GrmScale = 625;
	}else{
		gPulse.GrnScale = 1;
		gPulse.GrmScale = 1;
	}
}




void gPulseHandle(PulseTypedef *dp)
{
	s32 temp = 0;
	s16 temp16 = 0;
	/********************************************************/
	//
	if(dp->PwmEn == FALSE){
		dp->Grst = 0;
		dp->PFlag = TRUE;
		dp->NoPulseCnt = 0;
		dp->PincOut = 0;
		dp->PincOutI = 0;
		dp->FltPulSum = 0;
		dp->FltRest = 0;
		dp->FltIndex = 0;
		dp->FltAvg = 0;
		dp->Pool = 0;
		dp->MaxFre = (dp->MaxSpd * 50.0f * dp->QepLine * 0.001f * 2.1f)/(POS_LOOP_FRE);

		if(dp->FltClrIndex < PULSE_FILT_LEN)
		{
			memset(&dp->FltBuf[dp->FltClrIndex],0,PULSE_FILT_LEN);//PULSE_FILT_LEN  1001.44us  200 2.72us 4
			dp->FltClrIndex += PULSE_FILT_LEN;
		}

		dp->pIndex = 0;
		dp->pOfSum = 0;
		memset(&dp->pOfBuf[0],0,POS_LOOP_FRE*10);

		return;
	}
	/********************************************************/

	/********************************************************/
	//
	dp->PregGn = dp->Preg;
	/********************************************************/

	/********************************************************/
	//
	if(dp->Emgs){dp->PregGn = 0;}//
	if(dp->LmtL){
		if(dp->Dir == 0)
		{
			if(dp->PregGn > 0)  dp->PregGn = 0;
		}else{
			if(dp->PregGn < 0)  dp->PregGn = 0;
		}
	}
	if(dp->LmtR){
		if(dp->Dir == 0)
		{
			if(dp->PregGn < 0)  dp->PregGn = 0;
		}else{
			if(dp->PregGn > 0)  dp->PregGn = 0;
		}
	}
	/********************************************************/

	/*********************************************************/
	//
	if(dp->PregGn == 0)
	{
		if(dp->NoPulseCnt > 3){
			dp->PFlag = TRUE;
		}else{
			dp->NoPulseCnt++;
		}
	}else{
		dp->NoPulseCnt = 0;
		dp->PFlag = FALSE;
	}
	/*********************************************************/



	/********************************************************/
	//
	if(dp->FltZeroFlag == TRUE){
		dp->Pool = 0;
		dp->Grst = 0;
		dp->FltAvg = 0;
		dp->FltPulSum = 0;
		dp->FltRest = 0;
		dp->FltIndex = 0;
		if(dp->FltClrIndex < PULSE_FILT_LEN)
		{
			memset_16(&dp->FltBuf[dp->FltClrIndex],0,PULSE_FILT_LEN);//PULSE_FILT_LEN  1001.44us  200 2.72us 4
			dp->FltClrIndex += PULSE_FILT_LEN;
		}
	}else{
		dp->FltClrIndex = 0;

		if(dp->FiltLen > (PULSE_FILT_LEN-1))
		{
			dp->FiltLen = (PULSE_FILT_LEN-1);
		}

		/********************************************************/
		//
		dp->FltPulSum = dp->FltPulSum + dp->PregGn;
		dp->FltPulSum = dp->FltPulSum - dp->FltBuf[dp->FltIndex];
		dp->FltBuf[dp->FltIndex] = dp->PregGn;

		dp->FltIndex++;
		if(dp->FltIndex > dp->FiltLen) dp->FltIndex = 0;

		dp->FltAvg = (dp->FltPulSum + (s32)dp->FltRest) / (dp->FiltLen+1);
		if((dp->FltPulSum + dp->FltRest) < 0) {
			dp->FltRest = -((-(dp->FltPulSum + dp->FltRest))%(dp->FiltLen+1));
		}else{
			dp->FltRest = (dp->FltPulSum + dp->FltRest) % (dp->FiltLen+1);
		}
		/********************************************************/
	}
	/********************************************************/


	/********************************************************/
	//
	if(dp->Dir == 1) dp->FltAvg = -dp->FltAvg;

	if(dp->Emgs){dp->FltAvg = 0;}//
	if(dp->LmtL){if(dp->FltAvg > 0) dp->FltAvg = 0;}//
	if(dp->LmtR){if(dp->FltAvg < 0) dp->FltAvg = 0;}//

	dp->Pool +=  dp->FltAvg;

	temp = dp->FltAvg * (s32)dp->Grn + dp->Grst;
	dp->PincOutI = (s16)(temp/(s32)dp->Grm);
	dp->PincOut = dp->QepDline * (temp/dp->Grm);
        dp->PincOut = (dp->QepDline * temp)/dp->Grm;
        //dp->PincOut = (dp->QepDline * temp);


	if(temp >= 0)
	{
		dp->Grst = temp%dp->Grm;
	}else{
		dp->Grst = -((-temp)%dp->Grm);
	}

	/********************************************************/
	if(dp->pIndex < (POS_LOOP_FRE*10)){
		temp16 = abs(dp->PincOutI);
		dp->pOfSum += temp16;
		dp->pOfSum -= dp->pOfBuf[dp->pIndex];
		dp->pOfBuf[dp->pIndex] = temp16;

		dp->pIndex++;
		if(dp->pIndex >= (POS_LOOP_FRE*10))
		{
			dp->pIndex = 0;
		}

	}
	temp = dp->pOfSum/(POS_LOOP_FRE*10);
	if(temp > dp->MaxFre)
	{
		dp->OvrMaxFreFlag = TRUE;
	}
	/********************************************************/

}









