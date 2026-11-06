/*
 * ServoTask_Pos.c
 *
 *  Created on: 2016421
 *      Author: admin
 */


#include "SysMain.h"

/**************************************************************************************************/
void Moto1_Pos_Init(void)
{
	MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Pos_Macro;
}
/**************************************************************************************************/
Uint8 pos_trig = 0;

/**************************************************************************************************/
void Moto1_Pos_Macro(void)
{

	float temp_tor = 0;
	static u16 ClrBk = 0;
	static u16 cpflag = 0;
	static float KiGain = 1.0;
	float ActNlp = 0.0;
	int16_t plus_temp;
	float  ftemp;

	//motor_encoder_add_handle();

	if (gPwmIsrTacker != SPEED_LOOP_PSR)
	{
		
		return;
	}
	//---------------------------------------------------------------------------------------------//
	//¹ýÁãµã²¹³¥
	//Î»ÖÃ·´À¡
	if (gQepAngle.QepType >= ENC_ABS_17BIT) {
		gPidPos.Fbk = AbsPar.absMlt;
	} else {

		gPidPos.Fbk = gQepAngle.RawThetanew;

	}


	//---------------------------------------------------------------------------------------------//


	//---------------------------------------------------------------------------------------------//
	//
	if (SysParP4.LT_TOR)
	{
		if (McSwPort.In.bit.TCMCH == 0) {
			temp_tor = SysParP5.TQ0 * 0.00033333333f;
		} else if (McSwPort.In.bit.TCMCH == 1) {
			temp_tor = SysParP5.TQ1 * 0.00033333333f;
		} else if (McSwPort.In.bit.TCMCH == 2) {
			temp_tor = SysParP5.TQ2 * 0.00033333333f;
		} else {
			temp_tor = SysParP5.TQ3 * 0.00033333333f;
		}
	} else {
		temp_tor = SysParP0.MTOR * 0.00033333333f;
	}


	//---------------------------------------------------------------------------------------------//
	gPulse.PwmEn = gSysCtlVar.Ctl.bit.PwmEn;
	gPulse.Dir = SysParP0.PDIR;
	//---------------------------------------------------------------------------------------------//
	if (SysParP9.GEN == 0)
	{
		
			//---------------------------------------------------------------------------------------------//
			//
			switch (McSwPort.In.bit.GNUM)
			{
			case 0:
				gPulse.Grn = (u32)SysParP3.GRN1 * gPulse.GrnScale;
				gPulse.Grm = (u32)SysParP3.GRM1 * gPulse.GrmScale * 2;
				break;
			case 1:
				gPulse.Grn = (u32)SysParP3.GRN2 * gPulse.GrnScale;
				gPulse.Grm = (u32)SysParP3.GRM2 * gPulse.GrmScale * 2;
				break;

			default:
				gPulse.Grn = (u32)SysParP3.GRN1 * gPulse.GrnScale;
				gPulse.Grm = (u32)SysParP3.GRM1 * gPulse.GrmScale * 2;

				break;

			}
			//---------------------------------------------------------------------------------------------//

			//---------------------------------------------------------------------------------------------//
			//
			gPulse.Preg = MC_HD_Port.ExPulse;
			//---------------------------------------------------------------------------------------------//

			//---------------------------------------------------------------------------------------------//
			if (McSwPort.In.bit.INHP) //
			{
				gPulse.Preg = 0;
			}
			if (McSwPort.In.bit.CMDINV) {
				gPulse.Dir = !gPulse.Dir;
			}
			//---------------------------------------------------------------------------------------------//
		


	} else {
		//

	}
//---------------------------------------------------------------------------------------------//

//
	if (SysParP3.DCLR == 0) //
	{
		if (McSwPort.In.bit.CCLR && (ClrBk == FALSE))
		{
			gPulse.FltZeroFlag = TRUE;
			gPidPos.Ref = gPidPos.Fbk;
			ClrBk = TRUE;
		} else if (McSwPort.In.bit.CCLR == FALSE) {
			gPulse.FltZeroFlag = FALSE;
			ClrBk = FALSE;
		} else {
			gPulse.FltZeroFlag = FALSE;
		}
	} else {
		ClrBk = FALSE;
		gPulse.FltZeroFlag = McSwPort.In.bit.CCLR;
		if (McSwPort.In.bit.CCLR)
		{
			gPidPos.Ref = gPidPos.Fbk;
		}
	}
//---------------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------------//
//
	gPulse.Emgs = gSysErr.ErrEmg.bit.SEmgsErr;
	gPulse.LmtL = gSysErr.ErrEmg.bit.PosLmtL;
	gPulse.LmtR = gSysErr.ErrEmg.bit.PosLmtR;
//---------------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------------//
	gPulse.MaxSpd = SysParP0.MSPD * 0.00033333333f;
	gPulseHandle(&gPulse);
//---------------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------------//
	gPidPos.Ref += gPulse.PincOutI;
	gPidPos.Inc = gPulse.PincOut;

//---------------------------------------------------------------------------------------------//


//---------------------------------------------------------------------------------------------//
//
	if (gSysCtlVar.Ctl.bit.PwmEn == FALSE)
	{
		gPulse.FiltLen = SysParP3.PFLT * POS_LOOP_FRE;
		gPidPos.Ref = gPidPos.Fbk;
		gPidSpd.ui = 0;
		gPidSpd.Out = 0;
	}
//---------------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------------//




//---------------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------------//
	if (gPidPos.Ref > gPidPos.CirLine ) gPidPos.Ref -= gPidPos.CirLine ;
	else if (gPidPos.Ref < 0) gPidPos.Ref += gPidPos.CirLine ;

	gPidPos.Kp = McSwPort.uKpp * 0.005f;				//
	gPidPos.VfGain = SysParP0.PFG * 0.01f;				//
	gPidPos.PFlt = 1.0f / (SysParP0.PFF * POS_LOOP_FRE);	//
	PI_POS_MACRO(gPidPos)
//---------------------------------------------------------------------------------------------//
	can_delt_cmd.plus_err = gPidPos.Err * 10000;
//---------------------------------------------------------------------------------------------//
#if 0
	if (gPulse.PFlag)
	{
		if (cpflag == 0) {
			if (fabs(gPidPos.Err) < 0.2f) {
				KiGain = 2.0f;
				cpflag = 1;
			}

		} else if (cpflag == 1) {
			if (fabs(gPidPos.Err) < 0.01f) {
				KiGain = 10.0f;
				cpflag = 2;
			}

		} else if (cpflag == 1) {
			if (fabs(gPidPos.Err) < 0.005f) {
				KiGain = 50.0f;
				cpflag = 3;
			}

		} else if (cpflag == 3) {
			if (fabs(gPidPos.Err) < 0.0001f) {
				KiGain = 1.0f;
				cpflag = 4;
			}
		} else {
			KiGain = 1.0f;
		}
	} else {
		KiGain = 1.0f;
		cpflag = 0;
	}
#endif
	KiGain = 1.0f;
//---------------------------------------------------------------------------------------------//


//---------------------------------------------------------------------------------------------//
	gPidSpd.Ref = gPidPos.Out;
	gPidSpd.Fbk = gSpeed.Speed;
	gPidSpd.Umax = temp_tor;
	gPidSpd.Umin = -temp_tor;

	gPidSpd.Kp = McSwPort.uKvp * 0.005f;
	gPidSpd.Ki = T_PWM * SPEED_LOOP_PSR * McSwPort.uKvi;
//gPidSpd.Ki = KiGain*T_PWM*SPEED_LOOP_PSR*McSwPort.uKvi;
	gPidSpd.Kc = 4 * gPidSpd.Ki;

	if (gPidSpd.Kp <= 0.5f) {
		gPidSpd.InterThd = gPidSpd.Umax * 2.0f;
	} else {
		gPidSpd.InterThd = gPidSpd.Umax * 0.1f / gPidSpd.Kp;
	}
	PI_SPD_MACRO(gPidSpd)
//---------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------//
	/*
	gFilter1.Xn = gPidSpd.Out ;
		filter_step1(&gFilter1) ;
		gCurLoopGvn.IqRef = gFilter1.Yn ;
	*/
	gCurLoopGvn.IqRef = gPidSpd.Out;


//---------------------------------------------------------------------------------------------//
	if (gPulse.PFlag && (cpflag == 4))
	{
		ActNlp = LowPasFilterFuzzyMacr(gPidSpd.Err, (float)SysParP9.NLP);
		if (ActNlp == 0) {
			gCurLoopGvn.QrefLowKn = 1.0f;
		} else {
			gCurLoopGvn.QrefLowKn = 1.0f - (1.0f / (1.0f + ((20 * PI) / ActNlp)));
		}
	} else {
		gCurLoopGvn.QrefLowKn = 1.0f;
	}
//---------------------------------------------------------------------------------------------//


}
/**************************************************************************************************/





