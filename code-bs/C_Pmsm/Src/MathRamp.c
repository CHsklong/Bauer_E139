/*
 * MathRamp.c
 *
 *  Created on: 2015年12月8日
 *      Author: admin
 */

#include "MathRamp.h"

S_CurveTypedef ScurveVar;


/*********************************************************************************************/
//斜坡
float RamperF(float tar, float cur, float stup,float stdn)
{
	float err;
	err = tar - cur;
	if (err > stup)        	return(cur + stup);
  	else if (err < -stdn)  	return(cur - stdn);
	else                        	return(tar);
}
/*********************************************************************************************/

/*********************************************************************************************/
//斜坡
void Scurve_Macro(S_CurveTypedef *dp)
{

	if(dp->RampTime == 0)
	{
		dp->Spd_cur = RamperF(dp->Spd_tar,dp->Spd_cur,dp->Acc_tar,dp->Dec_tar);
		return;
	}
	dp->Jcc = dp->Acc_tar/dp->RampTime;
	dp->Jec = dp->Dec_tar/dp->RampTime;

	dp->Err = dp->Spd_tar - dp->Spd_cur;

	if(dp->Err == 0){
		dp->Spd_cur = dp->Spd_tar;
		dp->Acc_cur = 0;
		dp->Dec_cur = 0;
		dp->AccSts = 0;
		dp->DecSts = 0;
		dp->AccCnt = 0;
		dp->DecCnt = 0;
	}else if(dp->Err > 0){//加速
		if(dp->AccSts == 0)//加加速
		{
			if(dp->Err < dp->Jcc*dp->AccCnt*dp->AccCnt)
			{//切换至匀加速
				dp->AccSts = 2;
			}

			if(dp->AccCnt < dp->RampTime)
			{
				dp->Acc_cur = dp->Acc_cur + dp->Jcc;
				if(dp->Acc_cur > dp->Acc_tar) dp->Acc_cur = dp->Acc_tar;
				dp->AccCnt++;
			}else{//切换至匀加速
				dp->Acc_cur = dp->Acc_tar;
				dp->AccSts = 1;
			}
		}else if(dp->AccSts == 1)//匀加速
		{
			dp->Acc_cur = dp->Acc_tar;
			if(dp->Err < dp->Acc_tar*dp->RampTime)
			{
				dp->AccSts = 2;
			}
		}else if(dp->AccSts == 2)//减加速
		{
			if(dp->Err >= dp->Jcc*dp->AccCnt*dp->AccCnt)
			{//切换至匀加速
				dp->AccSts = 0;
			}

			if(dp->AccCnt > 0)
			{
				dp->Acc_cur = dp->Acc_cur - dp->Jcc;
				if(dp->Acc_cur < dp->Jcc)	dp->Acc_cur = dp->Jcc;
				dp->AccCnt--;
			}else{//切换至匀加速
				dp->Acc_cur = dp->Jcc;
			}
		}

		dp->Dec_cur = 0;
		dp->DecSts = 0;
		dp->DecCnt = 0;
	}else{//减速
		if(dp->DecSts == 0)//加加速
		{
			if(dp->Err > -dp->Jec*dp->DecCnt*dp->DecCnt)
			{//切换至匀加速
				dp->DecSts = 2;
			}

			if(dp->DecCnt < dp->RampTime)
			{
				dp->Dec_cur = dp->Dec_cur + dp->Jec;
				if(dp->Dec_cur > dp->Dec_tar) dp->Dec_cur = dp->Dec_tar;
				dp->DecCnt++;
			}else{//切换至匀加速
				dp->Dec_cur = dp->Dec_tar;
				dp->DecSts = 1;
			}
		}else if(dp->DecSts == 1)//匀加速
		{
			dp->Dec_cur = dp->Dec_tar;
			if(dp->Err > -dp->Dec_tar*dp->RampTime)
			{
				dp->DecSts = 2;
			}
		}else if(dp->DecSts == 2)//减加速
		{
			if(dp->Err <= -dp->Jec*dp->DecCnt*dp->DecCnt)
			{//切换至匀加速
				dp->DecSts = 0;
			}

			if(dp->DecCnt > 0)
			{
				dp->Dec_cur = dp->Dec_cur - dp->Jec;
				if(dp->Dec_cur < dp->Jec)	dp->Dec_cur = dp->Jec;
				dp->DecCnt--;
			}else{//切换至匀加速
				dp->Dec_cur = dp->Jec;
			}
		}
		dp->Acc_cur = 0;
		dp->AccSts = 0;
		dp->AccCnt = 0;
	}
	dp->Spd_cur = RamperF(dp->Spd_tar,dp->Spd_cur,dp->Acc_cur,dp->Dec_cur);
}
/*********************************************************************************************/


