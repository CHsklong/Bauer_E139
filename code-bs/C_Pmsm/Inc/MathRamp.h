/*
 * MathRamp.h
 *
 *  Created on: 2015Äê12ÔÂ8ÈÕ
 *      Author: admin
 */

#ifndef MATHRAMP_H_
#define MATHRAMP_H_

#include "Bsp.h"


typedef struct{
	float Err;
	float Spd_tar;
	float Spd_cur;
	float Acc_tar;
	float Acc_cur;
	float Dec_tar;
	float Dec_cur;
	float Jcc;
	float Jec;
	u16	  RampTime;
	u16   AccSts;
	u16   DecSts;
	u16   AccCnt;
	u16   DecCnt;
}S_CurveTypedef;

extern S_CurveTypedef ScurveVar;
float RamperF(float tar, float cur, float stup,float stdn);
void Scurve_Macro(S_CurveTypedef *dp);
#endif /* MATHRAMP_H_ */
