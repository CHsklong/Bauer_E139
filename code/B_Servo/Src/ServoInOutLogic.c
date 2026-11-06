/*
 * ServoInOutLogic.c
 *
 *  Created on: 2016511
 *      Author: admin
 */

#include "SysMain.h"
//------------------------------------------------------------------------------------------------------------

ATTR_RAMFUNC void SwInPortLogic(void)
{
	static u16 ARST_BK = 0;
	/*****************************************************************************/
	if(McSwPort.In.bit.POS_LMT)//
	{
		gSysErr.ErrEmg.bit.PosLmtL = TRUE;
	}

	if(McSwPort.In.bit.NPOS_LMT)//
	{
		gSysErr.ErrEmg.bit.PosLmtR = TRUE;
	}

	if(McSwPort.In.bit.EMGS)//
	{
		gSysErr.ErrEmg.bit.SEmgsErr = TRUE;
	}

	if(McSwPort.In.bit.ARST && !ARST_BK) //
	{
		gSysErrClearAll();
		ARST_BK = TRUE;
	}else if(!McSwPort.In.bit.ARST)
	{
		ARST_BK = FALSE;
	}

	gSysCtlVar.Son.bit.ExSon = McSwPort.In.bit.SON;//
	gSysCtlVar.Son.bit.SoftSon = SysParP0.SON;
	/*****************************************************************************/
}
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
void SwOutPortLogic(void)
{
	static u16 SbrkOnCnt = 0;
	static u16 SbrkOffCnt = 0;
	//--------------------------------------------------------------------------------------------------------------------------------------//
	/**************************************************************************************************/
	// RDY
	if(			(gSysErr.ErrHd.all  & gSysErrStop.ErrHd.all)
			|| 	(gSysErr.ErrCtl.all & gSysErrStop.ErrCtl.all)
			|| 	(gSysErr.ErrCom.all & gSysErrStop.ErrCom.all)
			|| 	(gSysErr.ErrEmg.all & gSysErrStop.ErrEmg.all)	)
	{
		McSwPort.Out.bit.SRDY = FALSE;
		McSwPort.Out.bit.SERR = TRUE;
	}else{
		McSwPort.Out.bit.SRDY = TRUE;
		McSwPort.Out.bit.SERR = FALSE;
	}
	/**************************************************************************************************/

	/**************************************************************************************************/
	//
	if(			(gSysErr.ErrHd.all  & ~gSysErrStop.ErrHd.all)
			|| 	(gSysErr.ErrCtl.all & ~gSysErrStop.ErrCtl.all)
			|| 	(gSysErr.ErrCom.all & ~gSysErrStop.ErrCom.all)
			|| 	(gSysErr.ErrEmg.all & ~gSysErrStop.ErrEmg.all)	)
	{
		McSwPort.Out.bit.WARN = TRUE;
	}else{
		McSwPort.Out.bit.WARN = FALSE;
	}
	/**************************************************************************************************/


	/**************************************************************************************************/
	//
	 if(gPulse.PFlag)
	 {
		 if(abs(gPidPos.Ref - gPidPos.Fbk) <= SysParP2.PER)
		 {
			 McSwPort.Out.bit.SINP = TRUE;
		 }else{
			 McSwPort.Out.bit.SINP = FALSE;
		 }
	 }else{
		 McSwPort.Out.bit.SINP = FALSE;
	 }
	/**************************************************************************************************/

	/**************************************************************************************************/
	//
	McSwPort.Out.bit.SON = gSysCtlVar.Ctl.bit.PwmEn;
	/**************************************************************************************************/

	/**************************************************************************************************/
	//
	McSwPort.Out.bit.FAN = HD_Temp.gFan_sts;
	/**************************************************************************************************/

	/**************************************************************************************************/
	//
	McSwPort.Out.bit.TQL = gPidSpd.TorLmtFlag;
	/**************************************************************************************************/

	/**************************************************************************************************/

		if(gQepAngle.IndexSyncFlag){
			McSwPort.Out.bit.SZFLG = TRUE;
			McSwPort.Out.bit.HZFLG = TRUE;
			gQepAngle.IndexSyncFlag = FALSE;
		}else{
			McSwPort.Out.bit.SZFLG = FALSE;
			McSwPort.Out.bit.HZFLG = FALSE;
		}

	

	/**************************************************************************************************/


	/*
	if(abs(MontorVar.TorFbkAvg)>SysParP2.TNCM)
	{
		McSwPort.Out.bit.TAT = TRUE;
	}else{
		McSwPort.Out.bit.TAT = FALSE;
	}
	*/

	/**************************************************************************************************/
	
	if(abs(gSpeed.SpeedFiterRpm) <= SysParP2.ZSPD)
	{
		McSwPort.Out.bit.ZSPDF = TRUE;
	}else{
		McSwPort.Out.bit.ZSPDF = FALSE;
	}
	

	/**************************************************************************************************/
	/*
	if(abs(MontorVar.SpeedRunRpm) >= SysParP2.SSPD)
	{
		McSwPort.Out.bit.SPD_OK = TRUE;
	}else{
		McSwPort.Out.bit.SPD_OK = FALSE;
	}*/
	/**************************************************************************************************/

	/**************************************************************************************************/
	/*
	if(abs(MontorVar.SpeedGvnRpm - MontorVar.SpeedRunRpm) <= SysParP2.SPOK)
	{
		McSwPort.Out.bit.TSPD = TRUE;
	}else{
		McSwPort.Out.bit.TSPD = FALSE;
	}
        */
	/**************************************************************************************************/

	/**************************************************************************************************/
	//
	if(gSysCtlVar.Ctl.bit.PwmEn == TRUE)
	{
		SbrkOffCnt = 0;
		if(SbrkOnCnt >= (SysParP2.MBT1*POS_LOOP_FRE))
		{
			McSwPort.Out.bit.SBRK = TRUE;
		}else{
			SbrkOnCnt++;
		}
	}else{
		SbrkOnCnt = 0;
		if(SbrkOffCnt >= (SysParP2.MBT2*POS_LOOP_FRE))
		{
			McSwPort.Out.bit.SBRK = FALSE;
		}else{
			SbrkOffCnt++;
			if(McSwPort.Out.bit.ZSPDF)
			{
				SbrkOffCnt = SysParP2.MBT2*POS_LOOP_FRE;
				McSwPort.Out.bit.SBRK = FALSE;
			}
		}
	}
	/**************************************************************************************************/


	/**************************************************************************************************/
	//
	McSwPort.Out.bit.BRKR =  BrakeVar.OutSts;
	/**************************************************************************************************/
}

//------------------------------------------------------------------------------------------------------------

