/*
 * ServoTask_Spd.c
 *
 *  Created on: 2016421
 *      Author: admin
 */

#include "SysMain.h"


/****************************************************************************************/
void Moto1_Spd_Init(void)
{
	memset_16(&ScurveVar.Err, 0, sizeof_16(S_CurveTypedef));
	ScurveVar.Spd_cur = gSpeed.Speed;
	MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Spd_Macro;
}

/****************************************************************************************/

ATTR_RAMFUNC float SPD_CMD_SELECT(void)
{
	float temp_spd = 0;

	if (SysParP4.SAS == 0) {		//
		temp_spd = SysParP4.CSPD * 0.00033333333f;
	} else if (SysParP4.SAS == 1) {	//
		temp_spd = SysParP4.SPD0 * 0.00033333333f;
	} else if (SysParP4.SAS == 2) {	//
		//
		if (McSwPort.In.bit.SPDCH == 0) {
			temp_spd = SysParP4.SPD0 * 0.00033333333f;
		} else if (McSwPort.In.bit.SPDCH == 1) {
			temp_spd = SysParP4.SPD1 * 0.00033333333f;
		} else if (McSwPort.In.bit.SPDCH == 2) {
			temp_spd = SysParP4.SPD2 * 0.00033333333f;
		} else if (McSwPort.In.bit.SPDCH == 3) {
			temp_spd = SysParP4.SPD3 * 0.00033333333f;
		}

	} else if (SysParP4.SAS == 3) {	//
		temp_spd = McSwPort.SpdGvn;
	} else if (SysParP4.SAS == 4) {
		
	} else if (SysParP4.SAS == 5) {

	}
	return temp_spd;


}
/****************************************************************************************/
ATTR_RAMFUNC void Moto1_Spd_Macro(void)
{
	float temp_spd = 0;
	float temp_tor = 0;
	float StAcc = 0.0f;
	float StDec = 0.0f;
	float MaxSpd = 0.0f;
	static u16 JogMode = 0;

	if (gPwmIsrTacker == SPEED_LOOP_PSR)
	{

		
		//---------------------------------------------------------------------------------------------//

		temp_spd = SPD_CMD_SELECT();


		//---------------------------------------------------------------------------------------------//


		//---------------------------------------------------------------------------------------------//
		//0
		if (McSwPort.In.bit.ZCLAMP)
		{
			temp_spd = 0;
		}
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		//
		if (SysParP0.PDIR)
		{
			temp_spd = -temp_spd;
		}

		if (McSwPort.In.bit.CMDINV)
		{
			temp_spd = -temp_spd;
		}
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		//STOP
		if (McSwPort.In.bit.STOP)
		{
			temp_spd = 0;
		}
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		/*		//
				if(HmiApi.JogRun == 0){
					gSysCtlVar.Son.bit.JogSon = FALSE;
					JogMode = 0;
				}

				if(JogMode){
					if(HmiApi.JogRun == HMIApi_JOG_CMD_START)
					{
						temp_spd = 0;
					}else if(HmiApi.JogRun == HMIApi_JOG_CMD_CW)
					{
						temp_spd = SysParP5.JOG * 0.00033333333f;
					}else if(HmiApi.JogRun == HMIApi_JOG_CMD_CCW)
					{
						temp_spd = SysParP5.JOG * 0.00033333333f;
						temp_spd = - temp_spd;
					}else{
						temp_spd = 0;
					}
				}
		*/		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		MaxSpd = SysParP0.MSPD * 0.00033333333f;
		if (temp_spd > MaxSpd)
		{
			temp_spd = MaxSpd;
		} else if (temp_spd < -MaxSpd)
		{
			temp_spd = -MaxSpd;
		}
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		//
		if (SysParP4.LT_TOR) //
		{
			temp_tor = TOR_CMD_SELECT();

		} else { //
			temp_tor = SysParP0.MTOR * 0.00033333333f; //
		}

		temp_tor = fabs(temp_tor);		//

		if (temp_tor > SysParP0.MTOR * 0.00033333333f)
		{
			temp_tor = SysParP0.MTOR * 0.00033333333f;
		}

		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		//
		if (SysParP4.TACC == 0)
		{
			StAcc = 1.0f;
		} else {
			StAcc = 1.0f / (POS_LOOP_FRE * SysParP4.TACC);
		}
		if (SysParP4.TDEC == 0)
		{
			StDec = 1.0f;
		} else {
			StDec = 1.0f / (POS_LOOP_FRE * SysParP4.TDEC);
		}
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		//

		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		//PI
		//---------------------------------------------------------------------------------------------//
		if (SysParP0.LSTP == 1) //
		{
			if (gSysErr.ErrEmg.bit.SEmgsErr)
			{
				temp_spd = 0;
			}
			if (gSysErr.ErrEmg.bit.PosLmtL)
			{
				if (temp_spd > 0) temp_spd = 0;
			}
			if (gSysErr.ErrEmg.bit.PosLmtR)
			{
				if (temp_spd < 0) temp_spd = 0;
			}
		}
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		ScurveVar.Spd_tar = temp_spd;
		ScurveVar.Acc_tar = StAcc;
		ScurveVar.Dec_tar = StDec;
		ScurveVar.RampTime = SysParP4.TSL * POS_LOOP_FRE;
		Scurve_Macro(&ScurveVar);
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		if (SysParP0.LSTP == 0) //
		{
			if (gSysErr.ErrEmg.bit.SEmgsErr)
			{
				ScurveVar.Spd_cur = 0;
			}
			if (gSysErr.ErrEmg.bit.PosLmtL)
			{
				if (ScurveVar.Spd_cur > 0) ScurveVar.Spd_cur = 0;
			}
			if (gSysErr.ErrEmg.bit.PosLmtR)
			{
				if (ScurveVar.Spd_cur < 0) ScurveVar.Spd_cur = 0;
			}
		}
		//---------------------------------------------------------------------------------------------//





		//---------------------------------------------------------------------------------------------//
		gPidSpd.Ref = ScurveVar.Spd_cur;
		gPidSpd.Fbk = gSpeed.Speed;
		gPidSpd.Umax = temp_tor;
		gPidSpd.Umin = -temp_tor;

		gPidSpd.Kp = McSwPort.uKvp * 0.005f;
		gPidSpd.Ki = T_PWM * SPEED_LOOP_PSR * McSwPort.uKvi; //PI_SPD_KI_MACRO(gPidSpd.Ref,McSwPort.uKvi);
		gPidSpd.Kc = 4 * gPidSpd.Ki;
		if (gPidSpd.Kp <= 0.5f) {
			gPidSpd.InterThd = gPidSpd.Umax * 2.0f;
		} else {
			gPidSpd.InterThd = gPidSpd.Umax * 0.1f / gPidSpd.Kp;
		}
		PI_SPD_MACRO(gPidSpd)
		//----------------------------------------






		//---------------------------------------------------------------------------------------------//
		if (gSysCtlVar.Ctl.bit.PwmEn == FALSE)
		{
			gPidSpd.ui = 0;
			gPidSpd.Out = 0;
			gPidSpd.Ref = 0;
			temp_spd = 0;
			ScurveVar.Spd_cur = 0.0f;
		}
		//---------------------------------------------------------------------------------------------//
		/*
		gFilter1.Xn = gPidSpd.Out ;
			filter_step1(&gFilter1) ;
			gCurLoopGvn.IqRef = gFilter1.Yn ;
		                */
		//---------------------------------------------------------------------------------------------//
		gCurLoopGvn.IqRef = gPidSpd.Out;
		//---------------------------------------------------------------------------------------------//
	} else {
		// can_test();
		//   yc_pos_judge_edge();

	}

	if (gQepAngle.QepType >= ENC_ABS_17BIT) {
		gPidPos.Fbk = AbsPar.absMlt;
	} else {
		gPidPos.Fbk = gQepAngle.RawThetanew;

	}

	gPidPos.Ref = gPidPos.Fbk ;
	gPidPos.Err = 0;
	if (gQepAngle.FirstIndexFlag == TRUE)
	{
		gPidPos.IndexFlag = TRUE;
	}

	gCurLoopGvn.QrefLowKn = 1.0f;
}
/****************************************************************************************/

