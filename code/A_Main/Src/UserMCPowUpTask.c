/*
 * UserMCPowUp.c
 *
 *  Created on: 201546
 *      Author: admin
 */

#include "SysMain.h"

#define PWM_OPEN_DLY 40
#define CUR_ZERO_LEARN_TIMES 1000L
#define POW_UP_DLY 10000L
#define BORD_INFO_DLY 1000L
#define RELAY_ON_DLY 1000L
#define	ABS_DLY		400

int8_t init_power ;

void SysPowUpTask(void)
{
	//
	static u16 isrticer = 0; //
	static u32 DrvInfo = 0;
	static u32 at_u1 = 0, at_v1 = 0;
	static u32 at_zl_pos = 0, at_zl_cur = 0;

	gSysCtlVar.Ctl.bit.SysSts = SYS_PWUP;

	MC_HD_Ang_Port_Read();

	if (isrticer == 0)
	{
		MC_HD_Port.MCOutport.bit.BusEnPort = TRUE;		//
		//PWM_LOWBRIDGE_OPEN();
		disable_all_pwm_output();
		close_high_open_low_pwm_output();
		init_power = 1;

	} else if (isrticer < PWM_OPEN_DLY) {

	} else if (isrticer < (PWM_OPEN_DLY + CUR_ZERO_LEARN_TIMES) ) {				//
		at_u1 += MC_HD_Port.CurU;
		at_v1 += MC_HD_Port.CurV;
		at_zl_pos += MC_HD_Port.zl_pos;
		at_zl_cur += MC_HD_Port.zl_cur;
	} else if (isrticer == (PWM_OPEN_DLY + CUR_ZERO_LEARN_TIMES)) {
		//PWM_LOWBRIDGE_CLOSE();
		init_power = 0;
		disable_all_pwm_output();
		/******************************************************************************************************/
		//
		//AngOffSet_U = at_u1/1000.0L;
		//AngOffSet_V = at_v1/1000.0L;
		/******************************************************************************************************/
		AngOffSet_U = at_u1 / 1000.0L;
		AngOffSet_V = at_v1 / 1000.0L;

		AngOffSet_zl_pos = at_zl_pos / 1000.0L ;
		AngOffSet_zl_cur = at_zl_cur / 1000.0L ;

		AngOffSet_zl_pos = 600 ; //588 ;  靠下越小
		/******************************************************************************************************/
		//
		if ((AngOffSet_U > CUR_ZERO_OFFSET_MAX) || (AngOffSet_U < CUR_ZERO_OFFSET_MIN))
		{
			gSysErr.ErrHd.bit.CurSensorError1 = TRUE;
		}
		if ((AngOffSet_V > CUR_ZERO_OFFSET_MAX) || (AngOffSet_V < CUR_ZERO_OFFSET_MIN))
		{
			gSysErr.ErrHd.bit.CurSensorError2 = TRUE;
		}
		/******************************************************************************************************/
	} else if (isrticer < (POW_UP_DLY + PWM_OPEN_DLY + CUR_ZERO_LEARN_TIMES)) {

	} else if (isrticer < (POW_UP_DLY + PWM_OPEN_DLY + CUR_ZERO_LEARN_TIMES + BORD_INFO_DLY)) {
		/******************************************************************************************************/
		//
		AngVar_Vbus.iReg = MC_HD_Port.Vbus;
		ADC_LP_MACRO(&AngVar_Vbus);
		/******************************************************************************************************/

		/******************************************************************************************************/
		//
		//DrvInfo += MC_HD_Port.DrvInfo;
		/******************************************************************************************************/

	} else if (isrticer == (POW_UP_DLY + PWM_OPEN_DLY + CUR_ZERO_LEARN_TIMES + BORD_INFO_DLY)) {
		//
		/******************************************************************************************************/
		//

		/*
		AngBordInfo = DrvInfo/1000;

		if(AngBordInfo < BORD_INFO_SCALE1)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[0].Index,sizeof_16(MC_DRVBORD_INFO));
		}else if(AngBordInfo < BORD_INFO_SCALE2)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[1].Index,sizeof_16(MC_DRVBORD_INFO));
		}else if(AngBordInfo < BORD_INFO_SCALE3)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[2].Index,sizeof_16(MC_DRVBORD_INFO));
		}else if(AngBordInfo < BORD_INFO_SCALE4)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[3].Index,sizeof_16(MC_DRVBORD_INFO));
		}else if(AngBordInfo < BORD_INFO_SCALE5)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[4].Index,sizeof_16(MC_DRVBORD_INFO));
		}else if(AngBordInfo < BORD_INFO_SCALE6)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[5].Index,sizeof_16(MC_DRVBORD_INFO));
		}else if(AngBordInfo < BORD_INFO_SCALE7)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[6].Index,sizeof_16(MC_DRVBORD_INFO));
		}else if(AngBordInfo < BORD_INFO_SCALE8)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[7].Index,sizeof_16(MC_DRVBORD_INFO));
		}else if(AngBordInfo < BORD_INFO_SCALE9)
		{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[8].Index,sizeof_16(MC_DRVBORD_INFO));
		}else{
			memcpy_16(&DrvBoarInfo.Index,&DRVBORD_INFO_INIT[0].Index,sizeof_16(MC_DRVBORD_INFO));
		}

		if(DrvBoarInfo.Index != MotoPar.DrvId)
		{
			gSysErr.ErrCtl.bit.MotoErr = TRUE;//
		}

		*/
		DrvBoarInfo.CurMax = 16 ;//8.25 ;       //8.25
		DrvBoarInfo.Scale  = 1.0  ;
		DrvBoarInfo.Index  =  0   ;



		SysParP8.HVER = 1 ; //DrvBoarInfo.Index;
		/******************************************************************************************************/

		/*
		if( AngVar_Vbus.OutLp < (SysParP7.LVH*1.414) )
		{
			gSysErr.ErrHd.bit.PowerUpErr = TRUE;			//
			MC_HD_Port.MCOutport.bit.StPort = FALSE;	//
		}else{
			gSysErr.ErrHd.bit.PowerUpErr = FALSE;			//ok
			MC_HD_Port.MCOutport.bit.StPort = TRUE;		//
		}
		*/

		/******************************************************************************************************/
		MotoCtlParMacro();		//
		/******************************************************************************************************/
	} else if (isrticer == (POW_UP_DLY + PWM_OPEN_DLY + CUR_ZERO_LEARN_TIMES + BORD_INFO_DLY + RELAY_ON_DLY)) { //

	} else if (isrticer >= (POW_UP_DLY + PWM_OPEN_DLY + CUR_ZERO_LEARN_TIMES + BORD_INFO_DLY + RELAY_ON_DLY + ABS_DLY)) { //
		isrticer = 0;
		gPwmIsrTacker = 0;
		MC_Task_Ptr = &MC_SysInit;

		
		
	}


	isrticer++;
}

