/*
 * MathCurLoop.h
 *
 *  Created on: 201683
 *      Author: admin
 */

#ifndef MATHCURLOOP_H_
#define MATHCURLOOP_H_


#include "adc_cla.h"
#include "pi_f.h"
#include "clarke_f.h"
#include "park_f.h"
#include "ipark_f.h"
#include "svgen_f.h"
#include "pwmgen.h"


/****************************************************************************************************************************/
extern ADC_CLA 			Adc_U;
extern ADC_CLA 			Adc_V;
extern CLARKE_CLA	 	Clark;
extern PARK_CLA	 		Park;
extern iPARK_CLA 	 	Ipark;
extern SVGEN_CLA	 	SvGen;
extern PI_CLA 			PidIq;
extern PI_CLA 			PidId;
extern PWMGEN_CLA  		PwmGen;
/****************************************************************************************************************************/

/****************************************************************************************************************************/
typedef struct{
	float 	CurAS;		//A 1
	float 	CurBS;		//B 2
	float	Sine;		// 3
	float   Theta;
	float	Cosine;		// 3
	float 	IqRef;		// 4
	float 	IdRef;		//6
	float 	Kp;			//PI8
	float 	Ki;			//PI9
	float	Kwe;		//
	float 	Speed;		//
	float 	PwmEn;      //13
	float 	CtlMode;	//14
	float   PwmHprd;
	float	PwmDtime;	//
	float   AdcOffSet_U;
	float   AdcOffSet_V;
	float   AdcGain;
	float 	Vbus;
	float	QrefLowKn;
	float 	MotoVolt;	//
}CUR_LOOP_GVN_TYPEDEF;
extern CUR_LOOP_GVN_TYPEDEF	 gCurLoopGvn;
/****************************************************************************************************************************/

/****************************************************************************************************************************/
typedef struct{
	float   CurA;
	float	CurB;
	float 	CurC;
	float	CurPow;
	float 	IqFbk;
	float 	IqRef;
	float 	IdFbk;
	float 	IdRef;
	float 	UqOut;
	float	UdOut;
	float   QUi;
	float   DUi;
	float 	PwmCmp1;	//PWM1  10
	float 	PwmCmp2;	//PWM1  11
	float 	PwmCmp3;	//PWM1  12
	float  	Sector;
}CUR_LOOP_FBK_TYPEDEF;


typedef  struct{
    int16_t  spdRpm;
    int16_t  iqRef;
    int16_t  iqFbk;
    int16_t  curA;
    int16_t  curB;
    
}MONITOR_TYPEDEF;
 
extern MONITOR_TYPEDEF          gMoinitor;

extern CUR_LOOP_FBK_TYPEDEF	 gCurLoopFbk;
extern float QrefLowpassFlter;
/****************************************************************************************************************************/


void CurLoopGvnDataInit(void);
void CurLoopVarInit(void);
void CurLoopMacro(void);

#endif /* MATHCURLOOP_H_ */
