/*
 * ServoTask_Tor.c
 *
 *  Created on: 2016421
 *      Author: admin
 */

#include "SysMain.h"


 
/****************************************************************************************/
void Moto1_Tor_Init(void)
{
	MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Tor_TASK;
}

/****************************************************************************************/
float TOR_CMD_SELECT(void)
{
	float temp_tor = 0;

	if(SysParP5.TAS == 0){	// 
		temp_tor = SysParP5.CTOR*0.00033333333f;//
	}else if(SysParP5.TAS == 1){	//
	
		temp_tor = SysParP5.TQ0*0.00033333333f;//
	}else if(SysParP5.TAS == 2){	//
	
		if(McSwPort.In.bit.TCMCH == 0){
			temp_tor = SysParP5.TQ0*0.00033333333f;//
		}else if(McSwPort.In.bit.TCMCH == 1){
			temp_tor = SysParP5.TQ1*0.00033333333f;//1
		}else if(McSwPort.In.bit.TCMCH == 2){
			temp_tor = SysParP5.TQ2*0.00033333333f;//2
		}else {
			temp_tor = SysParP5.TQ3*0.00033333333f;//3
		}
	
	}else if(SysParP5.TAS == 3){	//
		temp_tor = McSwPort.TorGvn;//
	
	}

	return  temp_tor ;
}
/****************************************************************************************/
void Moto1_Tor_TASK(void)
{
	float temp_spd = 0;
	float temp_tor = 0;
	float ramp_tor = 0.0;
	//---------------------------------------------------------------------------------------------//
	//
	temp_tor = TOR_CMD_SELECT();

	//---------------------------------------------------------------------------------------------//


	//---------------------------------------------------------------------------------------------//
	if(SysParP0.PDIR)
	{
		temp_tor = -temp_tor;
	}
	if(McSwPort.In.bit.CMDINV)
	{
		temp_tor = -temp_tor;
	}
	//---------------------------------------------------------------------------------------------//

	//---------------------------------------------------------------------------------------------//
	//
	temp_spd = SPD_CMD_SELECT();
	temp_spd = fabs(temp_spd);
	//---------------------------------------------------------------------------------------------//

	//---------------------------------------------------------------------------------------------//
	if(gSysErr.ErrEmg.bit.SEmgsErr) {temp_spd = 0;temp_tor = 0;}
	if(gSysErr.ErrEmg.bit.PosLmtL) {if(temp_tor > 0) temp_tor = 0;}
	if(gSysErr.ErrEmg.bit.PosLmtR) {if(temp_tor < 0) temp_tor = 0;}
	if(McSwPort.In.bit.STOP)  {temp_spd = 0;temp_tor = 0;}
	//---------------------------------------------------------------------------------------------//


	if(gPwmIsrTacker == SPEED_LOOP_PSR){
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		//

		//---------------------------------------------------------------------------------------------//


		//---------------------------------------------------------------------------------------------//
		if(temp_tor < 0){
			temp_spd = -temp_spd;
		}
		gPidSpd.Ref = temp_spd;
		gPidSpd.Fbk = gSpeed.Speed;
		gPidSpd.Umax = fabs(temp_tor);
		gPidSpd.Umin = -gPidSpd.Umax;

		gPidSpd.Kp = McSwPort.uKvp*0.005f;
		gPidSpd.Ki = T_PWM*SPEED_LOOP_PSR*McSwPort.uKvi;//PI_SPD_KI_MACRO(gPidSpd.Ref,McSwPort.uKvi);
		gPidSpd.Kc = 4 * gPidSpd.Ki;
		if(gPidSpd.Kp <= 0.5f){
			gPidSpd.InterThd = gPidSpd.Umax*2.0f;
		}else{
			gPidSpd.InterThd = gPidSpd.Umax/gPidSpd.Kp;
		}

		PI_SPD_MACRO(gPidSpd)

		if(SysParP1.TFLT == 0)
		{
			ramp_tor = 0.33333f;
		}else{
			ramp_tor =  0.3333f/(SysParP1.TFLT*POS_LOOP_FRE);
		}

		if(temp_tor >= 0)
		{
			if(gPidSpd.Err > 0.02f){
				gCurLoopGvn.IqRef = RamperF(temp_tor,gCurLoopGvn.IqRef,ramp_tor,ramp_tor);

				//gCurLoopGvn.IqRef = temp_tor;
			}else{
				gCurLoopGvn.IqRef = gPidSpd.Out;
			}
		}else{
			if(gPidSpd.Err < -0.02f){
				gCurLoopGvn.IqRef = RamperF(temp_tor,gCurLoopGvn.IqRef,ramp_tor,ramp_tor);
				//gCurLoopGvn.IqRef = temp_tor;
			}else{
				gCurLoopGvn.IqRef = gPidSpd.Out;
			}
		}
	}

	//---------------------------------------------------------------------------------------------//
	if(gSysCtlVar.Ctl.bit.PwmEn == FALSE)
	{
		gPidSpd.ui = 0;
		gPidSpd.Out = 0;
		gCurLoopGvn.IqRef = 0;
		temp_tor = 0;
		gPidSpd.Fbk = gSpeed.Speed;
		gPidSpd.Ref = gSpeed.Speed;
	}
	//---------------------------------------------------------------------------------------------//
	if(gQepAngle.QepType >= ENC_ABS_17BIT){
		gPidPos.Fbk = AbsPar.absMlt;
	}else{
		gPidPos.Fbk = gQepAngle.RawThetanew;

	}
	
	gPidPos.Ref = gPidPos.Fbk ;
	gPidPos.Err = 0;

	gCurLoopGvn.QrefLowKn = 1.0f;
			//---------------------------------------------------------------------------------------------//
/*		//
		gFilter1.Xn = gPidSpd.Out ;
			filter_step1(&gFilter1) ;
			gCurLoopGvn.IqRef = gFilter1.Yn ;
*/
}
