/*
 * MathBrake.c
 *
 *  Created on: 2015年3月28日
 *      Author: admin
 */

#include "SysMain.h"

#define T_BRAKE_PWM 100
#define MAX_BRAKE_PRD (T_BRAKE_PWM-10)

#if  POW_VER_STS == POW_VER_220
//-----------------------------------------------------------//
#define BRK_ON_VOLT 380
#define BRK_OFF_VOLT 365
#define BRK_PWR_VOLT 370L
//-----------------------------------------------------------//
#else
//-----------------------------------------------------------//
#define BRK_ON_VOLT 615
#define BRK_OFF_VOLT 600
#define BRK_PWR_VOLT 610L
//-----------------------------------------------------------//

#endif

BrakeTypedef BrakeVar = BRAKE_DEFAULTS;


/*************************************************************************/
void BrakeInit(void)
{
	memset_16(&BrakeVar.PwmEn,0,sizeof_16(BrakeTypedef));
	BrakeVar.Tbase = POS_LOOP_FRE * 100;
	BrakeVar.TbaseScale = 1.0/BrakeVar.Tbase;
	BrakeVar.pVolOn = BRK_ON_VOLT;
	BrakeVar.pVolOff = BRK_OFF_VOLT;
	BrakeVar.pRes = SysParP7.BRES;
	BrakeVar.pPow = SysParP7.BPWR;
	BrakeVar.DutyRefL = (float)BrakeVar.pPow *(float)BrakeVar.pRes/(BRK_PWR_VOLT*BRK_PWR_VOLT);
	BrakeVar.DutyRef = BrakeVar.DutyRefL * 8;
}
/*************************************************************************/


/*************************************************************************/
void BrakeMacro(BrakeTypedef *dp)
{
	float temp = 0;
	u16 i = 0;
	/**************************************************/
	if(dp->ErrFlag == TRUE){
		dp->OutSts = FALSE;

		if(dp->ErrWaitCnt > 4000){//等待1s左右
			dp->ErrFlag = FALSE;
			dp->ErrWaitCnt = 0;
			dp->DutyIndex = 0;
			dp->DutySum = 0;
			dp->DutyAvg = 0;
			for(i=0;i<BRK_POW_CALC_TIME;i++)
			{
				dp->DutyRatio[i] = 0;
			}

			dp->DutyIndexL = 0;
			dp->DutySumL = 0;
			dp->DutyAvgL = 0;
			for(i=0;i<BRK_POW_LONG_TIME;i++)
			{
				dp->DutyRatioL[i] = 0;
			}
		}else{
			dp->ErrWaitCnt++;
		}
		return;
	}

	if(dp->PwmEn == FALSE)
	{
		dp->DutyRefL = dp->PUsh * (float)dp->pPow *(float)dp->pRes*0.0000069252f;///(370L*370L);//功率系数：P/(380*380/R)
		dp->DutyRef = dp->DutyRefL * 8;
		dp->OutSts = FALSE;
		dp->ErrFlag = FALSE;
	}else{
		/**************************************************/
		if(dp->iVolt > dp->pVolOn)//只要判断到过压，随时可以打开计数器使能标志
		{
			dp->OutSts = TRUE;
		}else if(dp->iVolt < dp->pVolOff)
		{
			dp->OutSts = FALSE;
		}
		/**************************************************/
	}

	/**************************************************/
	if(dp->OutSts == TRUE)
	{
		dp->OnTicker++;
	}else{
		dp->OffTicker++;
	}
	if((dp->OffTicker + dp->OnTicker) >= BrakeVar.Tbase)//100ms
	{
		//---------------------------------------------------------------//
		//1s平均功率计算
		temp = (float)dp->OnTicker*dp->TbaseScale;
		dp->DutySum += temp;
		dp->DutySum -= dp->DutyRatio[dp->DutyIndex];
		if(dp->DutySum < 0) dp->DutySum = 0;
		dp->DutyRatio[dp->DutyIndex] =  temp;
		dp->DutyIndex++;

		dp->DutyAvg = dp->DutySum /BRK_POW_CALC_TIME;

		dp->OffTicker = 0;
		dp->OnTicker = 0;
		//---------------------------------------------------------------//

		//---------------------------------------------------------------//
		//10S平均计算
		if(dp->DutyIndex >= BRK_POW_CALC_TIME)
		{
			dp->DutyIndex = 0;

			dp->DutySumL += dp->DutyAvg;
			dp->DutySumL -= dp->DutyRatioL[dp->DutyIndexL];
			dp->DutyRatioL[dp->DutyIndexL] = dp->DutyAvg;
			dp->DutyIndexL++;
			dp->DutyAvgL = dp->DutySumL/BRK_POW_CALC_TIME;

			if(dp->DutyIndexL >= BRK_POW_LONG_TIME)
			{
				dp->DutyIndexL = 0;
			}
		}
		//---------------------------------------------------------------//
	}

	if((dp->DutyAvg > dp->DutyRef) || (dp->DutyAvgL > dp->DutyRefL)){
		dp->ErrFlag = TRUE;
		dp->OutSts = FALSE;
	}
	/**************************************************/

}
/*************************************************************************/

