/*
* UserMCIsr.c
* PWMADC
*
*  Created on: 2018-1-12
*  Author: admin
*/

#include "SysMain.h"

SYS_CTL_VAR_TYPEDEF gSysCtlVar = SYS_CTL_VAR_DEFAULTS;

/******************************************************************************/
//
void SysTickerHandle(void);
void MC_SafeCtl(void);
void MC_SONCtl(void);
/******************************************************************************/


/******************************************************************************/
//
void (*MC_Task_Ptr)(void);
void (*ABS_TASK_PTR)(void);

/******************************************************************************/

/******************************************************************************/
//TZ
ATTR_RAMFUNC void PwmTZIsrHandle(void)
{
	MC_HD_Port.MCInport.bit.IBOV = TRUE;
	SVPWMOFF();
}
/******************************************************************************/


/******************************************************************************/
//
u16 gPwmIsrTacker = 0;
/******************************************************************************/
ATTR_RAMFUNC void gMainIsrHanlde(void)
{

	static u16 LedTicker = 0;
	u16		cnt1 , cnt2;
//	SYS_LED_CTL(TRUE);

//	OUTPORT_OUT1(TRUE);
HPM_MCAN0->CCCR &= 0xfffe;
	/*******************************************************************************/
//	ADC_SOFT_START_CHANGE();			//ADC
//	IWDG_ReloadCounter();				//
	/*******************************************************************************/
	// SYS_LED_TOG();
	//SYS_LED_HIGH();
	/*******************************************************************************/
	//
	MainTaskTicker1ms++;
	//HmiApi.SysTikcer++;
//	E2PromWaitCnt++;
	LedTicker++;
	if (LedTicker >= 4000)
	{
		SYS_LED_TOG();
		LedTicker = 0;
	}
	write_csr(CSR_MCYCLE, 0);

	runTimea = read_csr(CSR_MCYCLE);
	/*******************************************************************************/

	/*******************************************************************************/
	gPwmIsrTacker++;
	if (gPwmIsrTacker >= (SPEED_LOOP_PSR + 1)) gPwmIsrTacker = 1;
	/*******************************************************************************/

	/*******************************************************************************/
	MC_HD_Port_Read();			//
	/*******************************************************************************/

	/*******************************************************************************/
	MC_SafeCtl();				//
	/*******************************************************************************/

	/*******************************************************************************/
	MC_SONCtl();
	/*******************************************************************************/

	/*******************************************************************************/
	(*MC_Task_Ptr)();			//
	/*******************************************************************************/

	/*******************************************************************************/
	MC_HD_Port_Load();			//
	/*******************************************************************************/
	//-----------------------------------------------------------------
	//----------- 调零模式 --工作模式切换-----------------
	//
	run_mode_change_handle();
	/*******************************************************************************/
//	if(!gDebugDlog.DlogEn) DebugModBus_Txd();

	//Dlog();
	/*******************************************************************************/


	/*if(MC_HD_Port.MCInport.bit.IBOV==FALSE){
		SVPWMON();
	}
	*/
	/*******************************************************************************/
	if (gPwmIsrTacker == 1) {
		cnt1 = ( HPM_PWM1->CNT ) >> 4; //TMR1->CNT;
	} else {
		cnt2 = ( HPM_PWM1->CNT ) >> 4; //TMR1->CNT;
	}



	// SYS_LED_LOW();

	//45US
	/*******************************************************************************/
}
/***************************************************************************************************/

/***************************************************************************************************/
ATTR_RAMFUNC void MC_SafeCtl(void)
{

	/*****************************************************************************/
	//      ipm
	if (MC_HD_Port.MCInport.bit.IBOV /*&& gSysCtlVar.Ctl.bit.PwmEn*/) {		//
		gSysErr.ErrHd.bit.ShortCircuit = TRUE;												//
		gSysCtlVar.Ctl.bit.PwmEn = FALSE;															//PWM
		MC_HD_Port.MCOutport.bit.BusEnPort = FALSE;										//
	}
	/*****************************************************************************/

	/*****************************************************************************/
	if (MC_HD_Port.MCInport.bit.IFOV && gSysCtlVar.Ctl.bit.PwmEn) {		//
		gSysErr.ErrHd.bit.ShortCircuit = TRUE;												//
		gSysCtlVar.Ctl.bit.PwmEn = FALSE;															//PWM
		MC_HD_Port.MCOutport.bit.BusEnPort = FALSE;										//
	}
	/*****************************************************************************/


	/**********************************************************************/
	//
	if (MC_HD_Port.MCInport.bit.BrkOV) {
		gSysErr.ErrHd.bit.BrkOvrCur = TRUE;
	}
	/**********************************************************************/

}
/***************************************************************************************************/

/***************************************************************************************************/
#define BOOST_TIME 10
ATTR_RAMFUNC void MC_SONCtl(void)
{
	static u16 WaitCnt = 0;
	static u16 PwmCnt = 0;

	//---------------------------------------------------------------------------------------------
	if (gSysCtlVar.Ctl.bit.SysSts < SYS_RUNN)
	{
		WaitCnt = 0;
		return;
	}

	if (WaitCnt < 5) {
		WaitCnt++;
		gSysCtlVar.Ctl.bit.HdPwmEn = FALSE;
		gSysCtlVar.Ctl.bit.PwmEn = FALSE;
		return;
	}
	//---------------------------------------------------------------------------------------------


	//---------------------------------------------------------------------------------------------
	if (gPwmIsrTacker == 1)
	{
		gSysCtlVar.Son.bit.SonAll = 	gSysCtlVar.Son.bit.ExSon  || gSysCtlVar.Son.bit.DebugSon \
		                                || 	gSysCtlVar.Son.bit.JogSon || gSysCtlVar.Son.bit.SoftSon;


		if ((gSysCtlVar.Son.bit.SonAll == TRUE ) && (gSysCtlVar.Son.bit.SonAllBk == FALSE ) )
		{
			gSysCtlVar.Ctl.bit.HdPwmEn = TRUE;
			gSysCtlVar.Son.bit.SonAllBk = TRUE;
		} else if (gSysCtlVar.Son.bit.SonAll == FALSE)
		{
			gSysCtlVar.Ctl.bit.HdPwmEn = FALSE;
			gSysCtlVar.Son.bit.SonAllBk = FALSE;
		}
	}
	//---------------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------------
	if (			(gSysErr.ErrHd.all & gSysErrStop.ErrHd.all)
	                || 	(gSysErr.ErrCtl.all & gSysErrStop.ErrCtl.all)
	                || 	(gSysErr.ErrCom.all & gSysErrStop.ErrCom.all)
	                || 	(gSysErr.ErrEmg.all & gSysErrStop.ErrEmg.all)	)
	{
		gSysCtlVar.Ctl.bit.HdPwmEn = FALSE;
	}
	//---------------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------------
	if (gSysCtlVar.Ctl.bit.HdPwmEn == TRUE)
	{
		if (gSysCtlVar.Ctl.bit.PwmEn == FALSE) {
			if (PwmCnt == 0)
			{
				//PWM_LOWBRIDGE_OPEN();
				disable_all_pwm_output();
				close_high_open_low_pwm_output();
				init_power = 1;
				PwmCnt++;
			} else if (PwmCnt < BOOST_TIME)
			{
				PwmCnt++;
			} else if (PwmCnt == BOOST_TIME) {
				//PWM_LOWBRIDGE_CLOSE();
				disable_all_pwm_output();
				init_power = 0;

				gSysCtlVar.Ctl.bit.PwmEn = TRUE;
			}
		}
	} else {
		PwmCnt = 0;
		gSysCtlVar.Ctl.bit.PwmEn = FALSE;
	}
	//---------------------------------------------------------------------------------------------
}
/***************************************************************************************************/



