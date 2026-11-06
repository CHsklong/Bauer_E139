/*
 * MathCurLoop.c
 *
 *  Created on: 201683
 *      Author: admin
 */
#include "SysMain.h"
#include "UserHdPort.h"
#include "MathCurLoop.h"
#include "UserTypeDef.h"
#include "mcu.h"

#include "UserSysPar.h"
#include "UserAng.h"
#include "UserMotoPar.h"

#include "Filter.h"
/****************************************************************************************************************************/
ADC_CLA 		Adc_U;
ADC_CLA 		Adc_V;
CLARKE_CLA	 	Clark;
PARK_CLA	 	Park;
iPARK_CLA 	 	Ipark;
SVGEN_CLA	 	SvGen;
PI_CLA 			PidIq;
PI_CLA 			PidId;
PWMGEN_CLA  	PwmGen;
float 			QrefLowpassFlter = 0.0f;
CUR_LOOP_GVN_TYPEDEF	 gCurLoopGvn;
CUR_LOOP_FBK_TYPEDEF	 gCurLoopFbk;
/****************************************************************************************************************************/
MONITOR_TYPEDEF          gMoinitor;
int16_t		loopcnt;
/****************************************************************************************************************************/
void CurLoopGvnDataInit(void)
{
	gCurLoopGvn.CurAS = 0.0f;
	gCurLoopGvn.CurBS = 0.0f;
	gCurLoopGvn.Theta = 0.0f;
	gCurLoopGvn.Sine = 0.0f;
	gCurLoopGvn.Cosine = 0.99f;
	gCurLoopGvn.IqRef = 0.0f;
	gCurLoopGvn.IdRef = 0.0f;
	gCurLoopGvn.Kp = gMcCtlPar.Kpt;
	gCurLoopGvn.Ki = gMcCtlPar.Kit;
	gCurLoopGvn.Speed = 0.0f;
	gCurLoopGvn.PwmEn = 0.0f;
	gCurLoopGvn.CtlMode = 0.0f;
	gCurLoopGvn.PwmHprd = (float)(SVPWM_PWM_PERIOD / 2);
	gCurLoopGvn.PwmDtime = (float)(SVPWM_PWM_DEDTIME);
	gCurLoopGvn.AdcOffSet_U = AngOffSet_U;
	gCurLoopGvn.AdcOffSet_V = AngOffSet_V;
	gCurLoopGvn.AdcGain = gMcCtlPar.CurScale / (-2048.0f);

	gCurLoopGvn.Vbus = 300;
	gCurLoopGvn.QrefLowKn = 1.0f;
	gCurLoopGvn.MotoVolt = (float)MotoPar.VOLT;
}
/****************************************************************************************************************************/

/****************************************************************************************************************************/
void CurLoopVarInit(void)
{
	QrefLowpassFlter = 0.0;

	/*************************************************************/
	Adc_U.Gain = gCurLoopGvn.AdcGain;
	Adc_U.Offset = gCurLoopGvn.AdcOffSet_U;
	Adc_U.Out = 0.0f;
	Adc_U.iReg = gCurLoopGvn.AdcOffSet_U;
	Adc_U.tTemp = 0.0f;

	Adc_V.Gain = gCurLoopGvn.AdcGain;
	Adc_V.Offset = gCurLoopGvn.AdcOffSet_V;
	Adc_V.Out = 0.0f;
	Adc_V.iReg = gCurLoopGvn.AdcOffSet_V;
	Adc_V.tTemp = 0.0f;
	/*************************************************************/


	/*************************************************************/
	Clark.As = 0.0f;
	Clark.Bs = 0.0f;
	/*************************************************************/


	/*************************************************************/
	PidIq.Ref = 0.0f;
	PidIq.Fbk = 0.0f;
	PidIq.Err = 0.0f;
	PidIq.Kp = 0.0f;
	PidIq.Ki = 0.0f;
	PidIq.uKi = 0.0f;
	PidIq.tmp = 0.0f;
	PidIq.Kc = 0.0f;
	PidIq.Umax = 0.95f;
	PidIq.Umin = -0.95f;
	PidIq.OutPreSat = 0.0f;
	PidIq.SatErr = 0.0f;
	PidIq.ui = 0.0f;
	PidIq.Up = 0.0f;
	PidIq.Out = 0.0f;
	PidIq.VbusRatio = 0.0f;
	PidIq.Emf = 0.0f;
	/*************************************************************/

	/*************************************************************/
	PidId.Ref = 0.0f;
	PidId.Fbk = 0.0f;
	PidId.Err = 0.0f;
	PidId.Kp = 0.0f;
	PidId.Ki = 0.0f;
	PidId.uKi = 0.0f;
	PidId.tmp = 0.0f;
	PidId.Kc = 0.0f;
	PidId.Umax = 0.25f;
	PidId.Umin = -0.25f;
	PidId.OutPreSat = 0.0f;
	PidId.SatErr = 0.0f;
	PidId.ui = 0.0f;
	PidId.Up = 0.0f;
	PidId.Out = 0.0f;
	PidId.VbusRatio = 0.0f;
	PidId.Emf = 0.0f;
	/*************************************************************/



	/*************************************************************/
	PwmGen.HalfPerMax = gCurLoopGvn.PwmHprd;
	PwmGen.DTIME = gCurLoopGvn.PwmDtime;
	/*************************************************************/

	/*************************************************************/
	gCurLoopFbk.CurA = 0.0f;
	gCurLoopFbk.CurB = 0.0f;
	gCurLoopFbk.CurC = 0.0f;
	gCurLoopFbk.CurPow = 0.0f;
	gCurLoopFbk.IdFbk = 0.0f;
	gCurLoopFbk.IdRef = 0.0f;
	gCurLoopFbk.IqFbk = 0.0f;
	gCurLoopFbk.IqRef = 0.0f;
	gCurLoopFbk.UdOut = 0.0f;
	gCurLoopFbk.UqOut = 0.0f;
	gCurLoopFbk.QUi = 0.0f;
	gCurLoopFbk.DUi = 0.0f;
	gCurLoopFbk.PwmCmp1 = 0;
	gCurLoopFbk.PwmCmp2 = 0;
	gCurLoopFbk.PwmCmp3 = 0;
	gCurLoopFbk.Sector = 0.0f;
	/*************************************************************/

}
/****************************************************************************************************************************/
#define UV_CHANGE  0
/****************************************************************************************************************************/
ATTR_RAMFUNC void CurLoopMacro(void)
{
	float VbusRatio = 0.0f;

	//VbusRatio = (gCurLoopGvn.MotoVolt*1.414213f) / gCurLoopGvn.Vbus;
	VbusRatio = 1;
	VbusRatio = gCurLoopGvn.MotoVolt / gCurLoopGvn.Vbus;  //dc motor
	/*************************************************************************************************************/
	// IU IV FITER
#if UV_CHANGE == 1
	gFilterIU.Xn = MC_HD_Port.CurU ;
	filter_step1(&gFilterIU) ;
	Adc_V.iReg = gFilterIU.Yn ;

	gFilterIV.Xn = MC_HD_Port.CurV ;
	filter_step1(&gFilterIV) ;
	Adc_U.iReg = gFilterIV.Yn ;

#else
	gFilterIU.Xn = MC_HD_Port.CurU ;
	filter_step1(&gFilterIU) ;
	Adc_U.iReg = gFilterIU.Yn ;

	gFilterIV.Xn = MC_HD_Port.CurV ;
	filter_step1(&gFilterIV) ;
	Adc_V.iReg = gFilterIV.Yn ;
#endif
	/*
		Adc_U.iReg = MC_HD_Port.CurU ;
		Adc_V.iReg = MC_HD_Port.CurV ;
		*/
//---------------------------------------------------------------------------------/
	//

	ADC_CLA_MACRO(Adc_U)


	ADC_CLA_MACRO(Adc_V)
	//---------------------------------------------------------------------------------/

	//---------------------------------------------------------------------------------/
	//PWM
	//CLARKE
	if (gCurLoopGvn.PwmEn == 0.0f) {
		Clark.As = 0.0f;
		Clark.Bs = 0.0f;
		PidIq.Out = 0.0f;
		PidIq.ui = 0.0f;
		PidId.Out = 0.0f;
		PidId.ui = 0.0f;
		QrefLowpassFlter = 0.0f;
	} else {
		Clark.As = Adc_U.Out;

#if	CUR_SEL ==	CUR_UV
		Clark.Bs = Adc_V.Out;
#endif

#if	CUR_SEL ==	CUR_UW
		Clark.Bs = -Adc_U.Out - Adc_V.Out;
#endif
	}
	CLARKE_CLA_MACRO(Clark)
	//---------------------------------------------------------------------------------/

	//---------------------------------------------------------------------------------/
	//PARK
	Park.alpha = Clark.alpha;
	Park.beta = Clark.beta;
	Park.theta = gCurLoopGvn.Theta;
	Park.sine = gCurLoopGvn.Sine;
	Park.cos = gCurLoopGvn.Cosine;
	PARK_CLA_MACRO(Park)

	//---------------------------------------------------------------------------------/
	//	//
	//QrefLowpassFlter = QrefLowpassFlter * (1.0 - gCurLoopGvn.QrefLowKn) + gCurLoopGvn.QrefLowKn * gCurLoopGvn.IqRef;
	//PidIq.Ref = QrefLowpassFlter;

	PidIq.Ref = gCurLoopGvn.IqRef;

	PidId.Ref = gCurLoopGvn.IdRef;

	//---------------------------------------------------------------------------------/
	//D Q LV BO

	gFilterIq.Xn = Park.q ;
	filter_step1(&gFilterIq) ;
	PidIq.Fbk = gFilterIq.Yn ;

	gFilterId.Xn = Park.d ;
	filter_step1(&gFilterId) ;
	PidId.Fbk = gFilterId.Yn ;
	/*	*/
	/*
	//	PidIq.Fbk = Park.q;


		PidId.Fbk = Park.d;

	*/
	//---------------------------------------------------------------------------------/
	PidIq.Kp = gCurLoopGvn.Kp;
	PidIq.Ki = gCurLoopGvn.Ki;

	//PidIq.Emf = gCurLoopGvn.Speed * gCurLoopGvn.Kwe*0.0001;
	PidIq.Emf = 0;
	PidIq.VbusRatio = VbusRatio;
	PI_CLA_MACRO(PidIq)
	//---------------------------------------------------------------------------------/

	//---------------------------------------------------------------------------------/
	PidId.Kp = gCurLoopGvn.Kp;
	PidId.Ki = gCurLoopGvn.Ki;

	PidId.Emf = 0;
	PidId.VbusRatio = VbusRatio;
	PI_CLA_MACRO(PidId)
	//---------------------------------------------------------------------------------/



	//---------------------------------------------------------------------------------/
	if (gCurLoopGvn.CtlMode == 1.0f)
	{
		PidIq.Out = 0.0f;
		PidIq.ui = 0.0f;
		PidId.Out = 0.0f;
		PidId.ui = 0.0f;
		Ipark.q = gCurLoopGvn.IqRef;
		Ipark.d = gCurLoopGvn.IdRef;
	} else {
		Ipark.q = PidIq.Out;
		Ipark.d = PidId.Out;
	}

	Ipark.sine = Park.sine;
	Ipark.cos = Park.cos;
	iPARK_CLA_MACRO(Ipark)
	//---------------------------------------------------------------------------------/

	//---------------------------------------------------------------------------------/
	SvGen.Ualpha = Ipark.alpha;
	SvGen.Ubeta = Ipark.beta;
	SVGEN_CLA_MACRO(SvGen)
	//---------------------------------------------------------------------------------/


	//---------------------------------------------------------------------------------/

#if	OC_PWM_SEL == OC_PWM1
	//TIM_OCMode_PWM1
	PwmGen.MfuncC1 = SvGen.Ta;
	PwmGen.MfuncC2 = SvGen.Tb;
	PwmGen.MfuncC3 = SvGen.Tc;
#else
	////TIM_OCMode_PWM2
	PwmGen.MfuncC1 = -SvGen.Ta;
	PwmGen.MfuncC2 = -SvGen.Tb;
	PwmGen.MfuncC3 = -SvGen.Tc;
#endif
	/**/
	PwmGen.HalfPerMax = gCurLoopGvn.PwmHprd;
	PWMGEN_CLA_MACRO(PwmGen)

	//---------------------------------------------------------------------------------/

	//---------------------------------------------------------------------------------/
	gCurLoopFbk.CurA = Clark.As;
	gCurLoopFbk.CurB = Clark.Bs;
	gCurLoopFbk.CurC = - (Clark.As + Clark.Bs);
	gCurLoopFbk.CurPow = Clark.alpha * Clark.alpha + Clark.beta * Clark.beta;
	gCurLoopFbk.IqFbk = PidIq.Fbk;
	gCurLoopFbk.IqRef = PidIq.Ref;
	gCurLoopFbk.IdFbk = PidId.Fbk;
	gCurLoopFbk.IdRef = PidId.Ref;
	gCurLoopFbk.UdOut = Ipark.d;
	gCurLoopFbk.UqOut = Ipark.q;
	gCurLoopFbk.QUi =  PidIq.ui;
	gCurLoopFbk.DUi = PidId.ui;
	gCurLoopFbk.PwmCmp1 = PwmGen.CMP1;
	gCurLoopFbk.PwmCmp2 = PwmGen.CMP2;
	gCurLoopFbk.PwmCmp3 = PwmGen.CMP3;
	gCurLoopFbk.Sector = SvGen.VecSector;
	//---------------------------------------------------------------------------------/
	/*

		PWM1_CMPA = PwmGen.CMP1;
		PWM2_CMPA = PwmGen.CMP2;
		PWM3_CMPA = PwmGen.CMP3;
	*/
	// PwmGen.CMP1 = 1000;
	// PwmGen.CMP2 = 3000;
	// PwmGen.CMP3 = 2000;
#if UV_CHANGE == 1
	pwm_update_raw_cmp_central_aligned(HPM_PWM0, CMP_INDEX_0, CMP_INDEX_1, PwmGen.CMP2, PWM_RELOAD - PwmGen.CMP2);
	pwm_update_raw_cmp_central_aligned(HPM_PWM0, CMP_INDEX_2, CMP_INDEX_3, PwmGen.CMP1, PWM_RELOAD - PwmGen.CMP1);
#else
	pwm_update_raw_cmp_central_aligned(HPM_PWM0, CMP_INDEX_0, CMP_INDEX_1, PwmGen.CMP1, PWM_RELOAD - PwmGen.CMP1);
	pwm_update_raw_cmp_central_aligned(HPM_PWM0, CMP_INDEX_2, CMP_INDEX_3, PwmGen.CMP2, PWM_RELOAD - PwmGen.CMP2);
#endif

//   pwm_update_raw_cmp_central_aligned(HPM_PWM0, CMP_INDEX_4, CMP_INDEX_5, PwmGen.CMP3,PWM_RELOAD-PwmGen.CMP3);
	pwm_update_raw_cmp_central_aligned(HPM_PWM0, WH_PWM_OUTPIN, WL_PWM_OUTPIN, PwmGen.CMP3, PWM_RELOAD - PwmGen.CMP3);
	//SYS_LED_HIGH();


	//-----------------------------
	gMoinitor.spdRpm = gSpeed.SpeedRpm ;
	gMoinitor.iqRef  = gCurLoopFbk.IqRef * 3000 ;
	gMoinitor.iqFbk  = gCurLoopFbk.IqFbk * 3000 ;

	gMoinitor.curA   = Adc_U.Out * 3000 ;
	gMoinitor.curB   = Adc_V.Out * 3000 ;


#ifndef NXP_MOTOR_CTRL
	spi_abs_tor_read_angle();

	//gan_pos_pid_loop_abs();
	spi_abs_tor_dma_tr_rx();
#endif
//SYS_LED_LOW();
	

	loopcnt++;
	

	//spd_test_run();

	can_test();
	can_send_time_judge();


	


}
/****************************************************************************************************************************/

