/*
 * UserMCSysInit.c
 *
 *  Created on: 201546
 *      Author: admin
 */

#include "SysMain.h"

int8_t smod_old = 0;
int8_t smod_sts = 0;

/*****************************************************************************************************/
void MC_SysInit(void)
{
	gSysCtlVar.Ctl.bit.SysSts = SYS_INIT;

	MC_HD_Ang_Port_Read();

	/*******************************************************/
	//



	if (SysParP8.ENCT == ENC_ABS_17BIT) {
		MotoPar.ENCL = 32768;
		MotoPar.ENCT = ENC_ABS_17BIT;
		MotoPar.EZRO = AbsPar.CalibratedAngle;


		if (AbsPar.absID != SysParP8.ENCT) {
			//gSysErr.ErrCtl.bit.EncPar = TRUE;

		}
		AbsPar.absID = ENC_ABS_17BIT ;
		AbsPar.Half = 1 << (AbsPar.absID - 1) ;


	} else {
		gSysErr.ErrCtl.bit.EncPar = TRUE;
	}



	/*******************************************************/
	MotoPar.POLES = SysParP8.POLES;
	/*******************************************************/
	ANGLE_INIT_ALL();
	ANGLE_MACRO_ALL(AbsPar.abs, AbsPar.absIncOut); //	ANGLE_MACRO_ALL();
	SPEED_INIT_ALL();
	gSpeed.OldMechTheta = gSpeed.MechTheta = gQepAngle.MechTheta;
	MC_ANG_SWPORT_INIT();
	PI_POS_INIT();
	PI_SPD_INIT();
	gPulseInit();	//
	//InterPosInit();
	MP_OverLoadInit();
	PaseVoltInit();
	BrakeInit();
	CurLoopGvnDataInit();
	CurLoopVarInit();

	MC_Task_Ptr = &MC_JUMP_TASK;


	//SysParP8.SMOD = MC_ZERO_MODE;

}
/*****************************************************************************************************/



/*****************************************************************************************************/
void MC_JUMP_TASK(void)
{
	MC_HD_Ang_Port_Read();

	if (SysParP8.SMOD == 0) {
		gSysCtlVar.Ctl.bit.SysSts = SYS_RUNN;
		MC_Task_Ptr = &MC_SERVO_TASK;
		MOTO1_SERVO_TASK_Api_Ptr = &MOTO1_SERVO_TASK_SEL;
		return;
	}


	if (SysParP8.SMOD == MC_DEBG_MODE) {
		gSysCtlVar.Ctl.bit.SysSts = SYS_DEBUG;
		//MC_Task_Ptr = &MC_Debug;
	} else if (SysParP8.SMOD == MC_ZERO_MODE) {
		gSysCtlVar.Ctl.bit.SysSts = SYS_ZERO;
		gSysErr.ErrCtl.bit.MotoErr = 0;
		gSysErr.ErrCtl.all = 0;

		MC_Task_Ptr = &MC_Zero;
	} else if (SysParP8.SMOD == MC_TEST_MODE) {
		gSysCtlVar.Ctl.bit.SysSts = SYS_TEST;

		gSysErr.ErrHd.all = 0;
		gSysErr.ErrEmg.all = 0;
		gSysErr.ErrCom.all = 0;
		gSysErr.ErrCtl.all = 0;

		//MC_Task_Ptr = &MC_Test;
	} else {
		gSysCtlVar.Ctl.bit.SysSts = SYS_RUNN;
		MC_Task_Ptr = &MC_SERVO_TASK;
		MOTO1_SERVO_TASK_Api_Ptr = &MOTO1_SERVO_TASK_SEL;
	}
}
/*****************************************************************************************************/


ATTR_RAMFUNC void run_mode_change_handle(void)
{
	switch (smod_sts)
	{
	case 0:
		if (smod_old != SysParP8.SMOD)
		{
			smod_old = SysParP8.SMOD ;
			smod_sts++;
		}
		break;
	case 1:
		SysParP0.SON = 0;
		smod_sts++;
		break;
	case 2:
		smod_sts++;
		break;
	case 3:
		smod_sts++;
		break;
	case 4:
		smod_sts++;
		break;
	case 5:
		MC_Task_Ptr = &MC_JUMP_TASK;
		smod_sts++;
		break;
	case 6:
		smod_sts++;
		break;
	case 7:
		smod_sts++;
		break;
	case 8:
		MotoZeroVar.Sts = 1;
		smod_sts++;
		break;
	case 9:
		if (flash_save_enable == 1) {
			smod_sts++;
		}
		break;
	case 10:
		if (flash_save_enable == 0)
		{
			smod_sts++;
		}
		break;
	case 11:
		SysParP8.SMOD = 0 ;
		smod_old = 0 ;
		smod_sts++;
		break;
//-------------------------------
	case 12:
		MC_Task_Ptr = &MC_SysInit;
		smod_sts++;
		break;
	case 13:
		smod_sts++;
		break;
	case 14:
		SysParP4.SAS = 0;
		// SysParP0.SON = 1;
		smod_sts++;
		break;
	case 15:
		smod_sts++;
		break;
	case 16:
		smod_sts = 0 ;
		break;


	default:
		smod_sts = 0;
		break;

	}
}



