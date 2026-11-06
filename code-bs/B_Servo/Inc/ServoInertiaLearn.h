/*
 * ServoInertiaLearn.h
 *
 *  Created on: 2017年11月22日
 *      Author: admin
 */

#ifndef SERVOINERTIALEARN_H_
#define SERVOINERTIALEARN_H_

#define INERTIA_MAX_TIMES 16

typedef struct{
	u16 	RunSts;

	u16 	pAdcTime;	//加减速时间
	u16		pRunSpd;	//运行速度
	u32		pRunPos;	//运行距离
	u16 	pRunMode;	//运行模式
	u16 	pRunTimes;	//学习次数
	u16 	pRunEn;		//使能

	u16 	PosAt;		//到位
	u16 	tRunCnt;		//运行次数

	u16 	oTrg;		//触发
	u32 	oPos;		//输出的圈数
	s16 	oSpd;		//运行速度
	u16		oADT;		//加减速时间


}INERTIA_TYPEDEF;

extern INERTIA_TYPEDEF gInertiaVar;


typedef struct{
	u16 Index;
	u16 Times;
	u16 TaskTrg;
	u16 ExTrigUp;
	u16 ExTrigDn;
	u16 ExTrigUpBk;
	u16 ExTrigDnBk;
	u16 InTrigUpT1;
	u16 InTrigUpT2;
	u16 InTrigDnT1;
	u16 InTrigDnT2;
	float SpdFbk;
	float TorFbk;
	u16 UpTicker;
	u16 DnTicker;
	u16 UpCnt;
	u16 DnCnt;
	float SpdCalcUpT1;
	float SpdCalcUpT2;

	float TorSumUp;
	float TorAvgUp;
	float AccActUp;



	float SpdCalcDnT1;
	float SpdCalcDnT2;

	float TorSumDn;
	float TorAvgDn;
	float AccActDn;


	float InertVarUD[INERTIA_MAX_TIMES];
	float InertVarUDSum;
	float InertUDDat;

}InertiaCalcTypedef;
extern InertiaCalcTypedef gILCalc;

void ServoInertiaLearnMacro(void);
void InertiaCalc(void);
void InertiaFC(void);

#endif /* SERVOINERTIALEARN_H_ */
