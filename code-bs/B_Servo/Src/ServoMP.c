/*
 * ServoMP.c
 *
 *  Created on: 2016512
 *      Author: admin
 */


#include "SysMain.h"

#define ANG_REF_ERR_TIME (50*POS_LOOP_FRE)
#define BRK_VOLT_ERR_TIME (POS_LOOP_FRE*22)

float ferrspd ;
float cmderrspd ;

ATTR_RAMFUNC void MC_Mp_Handle(void)
{
	s16 ang_err_max = 4095;
	s16 ang_err_min = 0;
	static u16 ang_err_spd_cnt = 0;
	static u16 ang_err_tor_cnt = 0;
	static u16 BrkOvltCnt = 0;
	static u16 LowPwrCnt = 0;
	static u16 UVWCnt = 0,AbzCnt = 0;
	static u16 CurCnt = 0;
	/**********************************************************************/
	//
	if(gSysCtlVar.Ctl.bit.PwmEn)
	{           // 100  --4000
		if((MC_HD_Port.CurU < 10) || (MC_HD_Port.CurU > 4086) || (MC_HD_Port.CurV < 10) || (MC_HD_Port.CurV > 4086))
		{
			CurCnt++;
			if(CurCnt > 10)
			{
				CurCnt = 0;
				gSysErr.ErrHd.bit.OverPhaseCurrent = TRUE;
			}
		}else{
			CurCnt = 0;
		}
	}
	/**********************************************************************/


	/**********************************************************************/
	//
	if( gPulse.OvrMaxFreFlag)
	{
		gSysErr.ErrCtl.bit.PulseErr = TRUE;
		gPulse.OvrMaxFreFlag = FALSE;
	}
	/**********************************************************************/

	/**********************************************************************/
	//
#ifdef   MP_OPEN
	if(fabs(gPidPos.Err) > SysParP7.PDEV)
	{
		gSysErr.ErrCtl.bit.OverPosition = TRUE;
	}
#endif
	/**********************************************************************/

	/**********************************************************************/
	//
	if(fabs(gSpeed.SpeedFiterRpm) > SysParP7.SDEV)
	{
		gSysErr.ErrCtl.bit.OverSpeed = TRUE;

               ferrspd =  gSpeed.SpeedFiterRpm ;
              
	}
	/**********************************************************************/

		/**********************************************************************/
		//UVW
		if((MC_HD_Port.EncoderQD == 0) || (MC_HD_Port.EncoderQD == 7) )
		{
			UVWCnt++;
			if(UVWCnt > 4){
				//gSysErr.ErrCtl.bit.EncErrUVW = TRUE;
			}
		}else{UVWCnt = 0;}
		/**********************************************************************/

		/**********************************************************************/
		//Z
		if(gQepAngle.ErrFlag)
		{
			gSysErr.ErrCtl.bit.EncErr_ABZ = TRUE;
		}
		/**********************************************************************/

	
		/**********************************************************************/


	



	/**********************************************************************/
	// 
	if(AngVar_Vbus.OutLp < (SysParP7.LVH*1.414f))
	{
		if(LowPwrCnt >= (POS_LOOP_FRE*210))
		{
			//gSysErr.ErrHd.bit.LowVoltage = TRUE;
		}else{
			LowPwrCnt++;
		}
	}else{
		LowPwrCnt = 0;
		if(gSysCtlVar.Ctl.bit.PwmEn)
		{
			if(AngVar_Vbus.OutLp > (SysParP7.OVH*1.414f + 30))
			{
				if(BrkOvltCnt > BRK_VOLT_ERR_TIME){
					gSysErr.ErrCtl.bit.BreakOverVolt = TRUE;
				}else{
					BrkOvltCnt++;
				}
			}else{
				BrkOvltCnt = 0;
			}
		}else{
			if((gSysErr.ErrCtl.bit.BreakOverVolt == 0) && (gSysErr.ErrCtl.bit.OverBreak == 0) )
			{
				if(AngVar_Vbus.OutLp > (SysParP7.OVH*1.414f))
				{
					gSysErr.ErrHd.bit.OverVoltage = TRUE;
				}
			}
		}
	}
	/**********************************************************************/

	/**********************************************************************/
	//
	ang_err_max = EX_REF_ANG_ZERO_DIG + ((u32)AngTorRef[ASPD_ID].ErrVolt * EX_REF_ANG_ZERO_DIG)/10000l;
	ang_err_min = EX_REF_ANG_ZERO_DIG - ((u32)AngTorRef[ASPD_ID].ErrVolt * EX_REF_ANG_ZERO_DIG)/10000l;
	if(AngTorRef[ASPD_ID].iReg > ang_err_max)
	{
		if(ang_err_spd_cnt > ANG_REF_ERR_TIME)
		{
			gSysErr.ErrHd.bit.VrefErrV = TRUE;
		}else{
			ang_err_spd_cnt++;
		}
	}else if(AngTorRef[ASPD_ID].iReg < ang_err_min){
		if(ang_err_spd_cnt > ANG_REF_ERR_TIME)
		{
			gSysErr.ErrHd.bit.VrefErrV = TRUE;
		}else{
			ang_err_spd_cnt++;
		}
	}else{
		ang_err_spd_cnt = 0;
	}
	/**********************************************************************/


	/**********************************************************************/
	//
	ang_err_max = EX_REF_ANG_ZERO_DIG + ((u32)AngTorRef[ATOR_ID].ErrVolt * EX_REF_ANG_ZERO_DIG)/10000l;
	ang_err_min = EX_REF_ANG_ZERO_DIG - ((u32)AngTorRef[ATOR_ID].ErrVolt * EX_REF_ANG_ZERO_DIG)/10000l;
	if(AngTorRef[ATOR_ID].iReg > ang_err_max)
	{
		if(ang_err_tor_cnt > ANG_REF_ERR_TIME)
		{
			gSysErr.ErrHd.bit.VrefErrT = TRUE;
		}else{
			ang_err_tor_cnt++;
		}
	}else if(AngTorRef[ATOR_ID].iReg < ang_err_min){
		if(ang_err_tor_cnt > ANG_REF_ERR_TIME)
		{
			gSysErr.ErrHd.bit.VrefErrT = TRUE;
		}else{
			ang_err_tor_cnt++;
		}
	}else{
		ang_err_tor_cnt = 0;
	}
	/**********************************************************************/



	/**************************************************************************************/
	BrakeVar.PwmEn = gSysCtlVar.Ctl.bit.PwmEn;
	BrakeVar.iVolt = AngVar_Vbus.Out;
	BrakeVar.pPow = SysParP7.BPWR;
	BrakeVar.pRes = SysParP7.BRES;
	BrakeVar.PUsh = SysParP7.BPWH * 0.01f;
	BrakeVar.pVolOn = SysParP7.OVH*1.414f;
	BrakeVar.pVolOff = SysParP7.OVH*1.414f -20;
	BrakeMacro(&BrakeVar);
	/**************************************************************************************/

	/**********************************************************************/
	if(BrakeVar.ErrFlag)
	{
	//	gSysErr.ErrCtl.bit.OverBreak = TRUE;
	}
	/**********************************************************************/

	/**********************************************************************/
	MP_OverLoadVar.gRefQs = SysParP7.OLH * 0.00033333333f;
	MP_OverLoadVar.gRefCountQs = OVER_LOAD_THD_SCALE * SysParP7.OLT*0.1f;
	MP_OverLoadVar.gQs = gCurLoopFbk.IqFbk;
	MP_OverLoadMacro(&MP_OverLoadVar);
	if(MP_OverLoadVar.flag )
	{
		gSysErr.ErrCtl.bit.OverPower = TRUE;
	}
	/**********************************************************************/

}




