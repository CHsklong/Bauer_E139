/*
 * MathOverLoad.c
 *
 *  Created on: 2016年5月16日
 *      Author: admin
 */

#include "SysMain.h"

MP_OverLoadTypeDef MP_OverLoadVar;

void MP_OverLoadInit(void)
{
	memset(&MP_OverLoadVar.flag,0,sizeof(MP_OverLoadTypeDef));
	MP_OverLoadVar.gRefQs =  0.33333333;
	MP_OverLoadVar.gRefCountQs = OVER_LOAD_THD_SCALE;//2倍过载1s
}



/****************************************************************************************************************************************************/
/************************************************************
 * 过载计算
 ************************************************************/
void MP_OverLoadMacro(MP_OverLoadTypeDef *dp)
{
	float tempq = 0;
	if(dp->flag == TRUE){
		dp->wait_cnt++;
		if(dp->wait_cnt > 2000)
		{
			dp->wait_cnt = 0;
			dp->flag = FALSE;
			dp->gCountQs = 0;
		}
		return;
	}
	tempq = fabs(dp->gQs);
	if(tempq >= dp->gRefQs )
	{
		dp->gQsAdd = tempq * tempq * tempq * tempq /( dp->gRefQs * dp->gRefQs * dp->gRefQs * dp->gRefQs);
		dp->gCountQs += dp->gQsAdd;
	}
	else if(dp->gCountQs >= 16)
	{
		dp->gCountQs -= 16;
	}
	if(dp->gCountQs > dp->gRefCountQs)	dp->flag = TRUE;
	else dp->flag = FALSE;
}
/****************************************************************************************************************************************************/


