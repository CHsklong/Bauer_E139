/*
 * MathBrake.h
 *
 *  Created on: 2015年3月28日
 *      Author: admin
 */

#ifndef MATHBRAKE_H_
#define MATHBRAKE_H_

#include "Bsp.h"

#define BRK_POW_CALC_TIME 20 //20*100MS
#define BRK_POW_LONG_TIME 10
typedef struct{
	u16 PwmEn;					//输入信号


	u16 ErrFlag;
	u16 ErrWaitCnt;


	u16 pPow;	//参数 电阻功率
	u16 pRes;	//参数-电阻阻值
	float PUsh;	//参数-制动电阻使用率


	u16 pVolOn;	//开启电压
	u16 pVolOff;//关闭电压
	u16 iVolt;	//输入电压

	u16 OutSts;

	u16 OnTicker;
	u16 OffTicker;
	u16 Tbase;	//时间基准
	float TbaseScale;
	float DutyRatio[BRK_POW_CALC_TIME];
	float DutyRatioL[BRK_POW_LONG_TIME];
	float DutySum;
	float DutySumL;
	float DutyAvg;
	float DutyAvgL;
	u16   DutyIndex;
	u16   DutyIndexL;


	float DutyRef;	//比较参考值
	float DutyRefL;	//比较参考值
}BrakeTypedef;

extern BrakeTypedef BrakeVar;

#define BRAKE_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

void BrakeInit(void);
void BrakeMacro(BrakeTypedef *dp);
#endif /* MATHBRAKE_H_ */
