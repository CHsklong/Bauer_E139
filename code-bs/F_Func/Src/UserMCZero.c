/*
 * UserMCZero.c
 *
 *  Created on: 201546
 *      Author: admin
 */

#include "SysMain.h"

/*********************************************************************/
MOTO_ZERO_TYPEDEF MotoZeroVar;
/*********************************************************************/
//MontorVar.PosErrQep
s16  encode_zero;
/*********************************************************************/
void ZeroTaskMacro(MOTO_ZERO_TYPEDEF *dp);
void openloopTaskMacro(MOTO_ZERO_TYPEDEF *dp);
/*********************************************************************/
#define	ZERO_MODE_ABZ	1
#define	ZERO_MODE_ABS	0

static u16 InitFlag = 0;
/*********************************************************************/
ATTR_RAMFUNC void MC_Zero(void)
{


	if (InitFlag == 0)
	{
		memset_16(&MotoZeroVar.Sts, 0, sizeof_16(MOTO_ZERO_TYPEDEF));

		//MotoZeroVar.Dgvn = (SysParP8.CEN/100)*0.001f;
                    MotoZeroVar.Dgvn = 0.8; // 0.26;
		//if(MotoZeroVar.Dgvn > 0.3f) MotoZeroVar.Dgvn = 0.2f;
		MotoZeroVar.HoldTime = (SysParP8.CEN%100)*20000l;
		//--------------------------
		MotoZeroVar.DrefAddTim = 10000;
		MotoZeroVar.DrefHoldTim = 10000;
		MotoZeroVar.refSpd = 30;

		MotoZeroVar.stop = 0;

		InitFlag = TRUE;

		return;
	}
	//SYS_LED_LOW();
//         spi1_state_rx_chn = dma_check_transfer_status(HPM_HDMA, HW_SPI_RX_DMA);
	ABS_MOTOR_READ_ANGLE();
	//
	ANGLE_MACRO_ALL(AbsPar.abs, AbsPar.absIncOut); //

	if (gQepAngle.QepType >= ENC_ABS_17BIT)
	{
		MotoZeroVar.Mode = ZERO_MODE_ABS;
		/**********************************************************************/
		//
		if (AbsPar.absCrcErr > 2)
		{
			gSysErr.ErrCtl.bit.EncErr_ABZ = TRUE;
		}
		/**********************************************************************/

		MotoZeroVar.EzQepFbk = AbsPar.abs;

	} else {
		MotoZeroVar.Mode = ZERO_MODE_ABZ;
		/**********************************************************************/
		//ABZ


		/**********************************************************************/

		MotoZeroVar.EzQepFbk = (ENC_QEPCNT_REG % MotoZeroVar.QepLine);
	}

	/**********************************************************************/

	/**********************************************************************/

	/**********************************************************************/
	//MontorVar.ServoErrId = ErrId;
	/**********************************************************************/

	/**************************************************************************/
	MotoZeroVar.Ticker++;
	switch (MotoZeroVar.Sts)
	{
	case 0:
		MotoZeroVar.PwmEn = 0;
		gSysCtlVar.Son.bit.SoftSon = MotoZeroVar.PwmEn;
		break;
	case 1:
		if (SysParP8.SMOD) {
			MotoZeroVar.ZeroEn = TRUE;
			gQepAngle.CalibratedAngle = 0;
			//HmiApi.HmiId = 0;
			//HmiApi.Index = 2;
			MotoZeroVar.QepLine = MotoPar.ENCL * 4;
			MotoZeroVar.QepElc = MotoZeroVar.QepLine / (MotoPar.POLES / 2);
			MotoZeroVar.QepMax = MotoZeroVar.QepLine - 50; //MotoZeroVar.QepElc*3/5;
			MotoZeroVar.QepMin =  50;   //MotoZeroVar.QepElc*2/5;

			if (MotoZeroVar.Ticker > (SPEED_LOOP_PSR * 500l)) {
				MotoZeroVar.Sts++;      // = 1;
				MotoZeroVar.Ticker = 0;
				SysParP8.SMOD = 0;
				SysParP6.SAVE = TRUE;
			}
		} else {
			MotoZeroVar.Sts++;      // = 1;
			MotoZeroVar.Ticker = 0;
		}
		break;
	case 2:
		if (MotoZeroVar.Ticker >= (SPEED_LOOP_PSR + 1))  MotoZeroVar.Ticker = 1;

		/**********************************************************************/
		//
//		ANGLE_MACRO_ALL(AbsPar.abs,AbsPar.absIncOut);//1.5us
//		MotoZeroVar.EzQepFbk = (ENC_QEPCNT_REG%MotoZeroVar.QepLine);
		/**********************************************************************/


		if (MotoZeroVar.Ticker == 1)
		{
			/**********************************************************************/
			//
			if (gQepAngle.QepType < ENC_ABS_17BIT) { //ABZ
				gSpeed.FirstIndexFlag = gQepAngle.FirstIndexFlag;
				gSpeed.FirstIndexMechTheta = gQepAngle.FirstIndexMechTheta;

			}

			gSpeed.MechTheta = gQepAngle.MechTheta;
			SPEED_MACRO_ALL(gSpeed)	//1.4us
			/**********************************************************************/

		}
		ZeroTaskMacro(&MotoZeroVar);
		gSysCtlVar.Son.bit.SoftSon = MotoZeroVar.PwmEn;
		/*****************************************************************************/
		break;

	case  11:
		if (SysParP8.SMOD) {
			MotoZeroVar.ZeroEn = TRUE;
			gQepAngle.CalibratedAngle = 0;
			//HmiApi.HmiId = 0;
			//HmiApi.Index = 2;
			MotoZeroVar.QepLine = MotoPar.ENCL * 4;
			MotoZeroVar.QepElc = MotoZeroVar.QepLine / (MotoPar.POLES / 2);
			MotoZeroVar.QepMax = MotoZeroVar.QepLine - 50; //MotoZeroVar.QepElc*3/5;
			MotoZeroVar.QepMin =  50;   //MotoZeroVar.QepElc*2/5;

			if (MotoZeroVar.Ticker > (SPEED_LOOP_PSR * 500l)) {
				MotoZeroVar.Sts++;      // = 1;
				MotoZeroVar.Ticker = 0;
				SysParP8.SMOD = 0;
				SysParP6.SAVE = TRUE;
			}
		} else {
			MotoZeroVar.Sts++;      // = 1;
			MotoZeroVar.Ticker = 0;
		}
		break;
	case 12:
		if (MotoZeroVar.Ticker >= (SPEED_LOOP_PSR + 1))  MotoZeroVar.Ticker = 1;

		/**********************************************************************/
		//
//		ANGLE_MACRO_ALL(AbsPar.abs,AbsPar.absIncOut);//1.5us
//		MotoZeroVar.EzQepFbk = (ENC_QEPCNT_REG%MotoZeroVar.QepLine);
		/**********************************************************************/


		if (MotoZeroVar.Ticker == 1)
		{
			/**********************************************************************/
			//
			if (gQepAngle.QepType < ENC_ABS_17BIT) { //ABZ
				gSpeed.FirstIndexFlag = gQepAngle.FirstIndexFlag;
				gSpeed.FirstIndexMechTheta = gQepAngle.FirstIndexMechTheta;

			}

			gSpeed.MechTheta = gQepAngle.MechTheta;
			SPEED_MACRO_ALL(gSpeed)	//1.4us
			/**********************************************************************/

		}
		openloopTaskMacro(&MotoZeroVar);
		gSysCtlVar.Son.bit.SoftSon = MotoZeroVar.PwmEn;
		/*****************************************************************************/
		break;
	}
	/**************************************************************************/


	/**************************************************************************/
	// 27.8uS
	gCurLoopGvn.Theta = MotoZeroVar.Theta;
	//gCurLoopGvn.Sine =  arm_sin_f32(MotoZeroVar.Theta*2*PI);
	//gCurLoopGvn.Cosine =  arm_cos_f32(MotoZeroVar.Theta*2*PI);

	gCurLoopGvn.Sine =  hpm_dsp_sin_f32(MotoZeroVar.Theta * 2 * PI);
	gCurLoopGvn.Cosine =  hpm_dsp_cos_f32(MotoZeroVar.Theta * 2 * PI);

	gCurLoopGvn.Kp = gMcCtlPar.Kpt;
	gCurLoopGvn.Ki = gMcCtlPar.Kit;
	gCurLoopGvn.PwmEn = (float)gSysCtlVar.Ctl.bit.PwmEn;
	gCurLoopGvn.IdRef = MotoZeroVar.Dref;
	gCurLoopGvn.IqRef = MotoZeroVar.Qref;
	gCurLoopGvn.CtlMode = 1.0f;// park   dref qref
	gCurLoopGvn.Speed = 0.0f;//gSpeed.Speed;
	gCurLoopGvn.QrefLowKn = 1.0f;
	/**************************************************************************/

	/**************************************************************************************/
	MC_HD_Ang_Port_Read();		//
//	gCurLoopGvn.CurAS = MC_HD_Port.CurU;
//	gCurLoopGvn.CurBS = MC_HD_Port.CurV;
	if ( AngVar_Vbus.Out < SysParP7.LVH) gCurLoopGvn.Vbus = SysParP7.LVH;
	else gCurLoopGvn.Vbus = AngVar_Vbus.Out;
	/**************************************************************************************/

	/**************************************************************************/
	CurLoopMacro();
	/**************************************************************************/

	/**************************************************************************/
	MC_HD_Port.PwmCmp1 = (u16)gCurLoopFbk.PwmCmp1;
	MC_HD_Port.PwmCmp2 = (u16)gCurLoopFbk.PwmCmp2;
	MC_HD_Port.PwmCmp3 = (u16)gCurLoopFbk.PwmCmp3;
	MC_HD_Port.PwnEn = gSysCtlVar.Ctl.bit.PwmEn;

	PaseVolt.MfuncV1 = gCurLoopFbk.PwmCmp1;
	PaseVolt.MfuncV2 = gCurLoopFbk.PwmCmp2;
	PaseVolt.MfuncV3 = gCurLoopFbk.PwmCmp3;
	PHASEVOLT_MACRO(PaseVolt)
	/**************************************************************************/
}
/*********************************************************************/


/*********************************************************************/
#define EZDrefAddTim        11000            //3200			//D/8000
#define EZTheatAddTim       20000 //19000            //6400l           //6400		//  /8000 = 0.8s
#define EZHoldTim_FIRST     9000            //1600            //


void ZeroTaskMacro(MOTO_ZERO_TYPEDEF *dp)
{
	if (dp->ZeroEn  == 0) {
		dp->Dref  = 0.0f;
		dp->Qref  = 0.0f;
		dp->Theta  = 0.0f;
		dp->PwmEn  = 0;
		dp->ZeroSts  = 0;
		return;
	}

	switch (dp->ZeroSts)
	{
	case 0:
		dp->Dref  = 0.0f;
		dp->Qref  = 0.0f;
		dp->Theta  = 0.0f;
		dp->PwmEn  = TRUE;
		if (dp->ZeroTicker < 100)
		{
			dp->ZeroTicker ++;
		} else {
			dp->ZeroTicker  = 0;
			dp->ZeroSts  = 1;
		}
		break;
	case 1:
		dp->Dref  += (dp->Dgvn / EZDrefAddTim);		//4KHZ
		if (dp->Dref  > dp->Dgvn ) {
			dp->Dref  = dp->Dgvn ;
			dp->ZeroSts  = 2;
		}
		break;
	case 2:
		dp->ZeroTicker ++;
		if ( dp->ZeroTicker  >= EZHoldTim_FIRST)
		{
			dp->Dref  = dp->Dgvn * 0.8f;
			dp->ZeroTicker  = 0;

			dp->ZeroSts  = 3;
			/*
			if(dp->Mode == ZERO_MODE_ABZ){
			  dp->ZeroSts  = 3;
			}else{
			  dp->ZeroSts  = 8;
			}
			*/

		}
		break;
	case 3:
		dp->ZeroTicker ++;
		dp->Theta  += (1.0f / EZTheatAddTim);
		if (dp->Theta  > 1.0f) dp->Theta  -= 1.0f;
		if (dp->ZeroTicker  > (EZTheatAddTim * 8l)) //8
		{
			if ((dp->EzQepFbk  > dp->QepMax ) || (dp->EzQepFbk  < dp->QepMin ))
			{
				dp->ZeroSts  = 4;
				dp->ZeroTicker  = 0.0f;
			}
		}
		//MontorVar.PosErrQep = gSpeed.SpeedFiterRpm;
		encode_zero =   gSpeed.SpeedFiterRpm;
		break;
	case 4:
		if (dp->Dref  <= (dp->Dgvn * 0.8f) / EZDrefAddTim)
		{
			dp->Dref  = 0;
			dp->Theta  = 0.0f;
			dp->ZeroSts  = 5;

		} else {
			dp->Dref  -= ((dp->Dgvn * 0.8f) / EZDrefAddTim);
		}
		break;
	case 5:
		if (dp->Dref  >= dp->Dgvn )
		{
			dp->Dref  = dp->Dgvn ;
			// dp->ZeroSts  = 6;
			if (dp->Mode == ZERO_MODE_ABZ) {
				dp->ZeroSts  = 6;
			} else {
				dp->ZeroSts  = 8;
			}


		} else {
			dp->Dref  += (dp->Dgvn / EZDrefAddTim);
		}
		break;
	case 6:
		dp->ZeroTicker ++;

		if (dp->EzQepFbk  > (dp->QepLine / 2))
		{
			dp->EzEepErr  = -(s16)((dp->QepLine  - dp->EzQepFbk ) % dp->QepElc );
		} else {
			dp->EzEepErr  = dp->EzQepFbk  % dp->QepElc ;
		}

		// MontorVar.PosErrQep = -(s16)dp->EzEepErr ;
		encode_zero =  -(s16)dp->EzEepErr;

		if (dp->ZeroTicker  > dp->HoldTime )
		{
			dp->ZeroTicker  = 0;
			dp->Dref  = 0;
			dp->PwmEn  = 0;
			dp->ZeroSts  = 7;
		}
		break;
	case 7:
		//  MontorVar.PosErrQep = -(s16)dp->EzEepErr;
		encode_zero =  -(s16)dp->EzEepErr;
		dp->ZeroSts++;
		break;
	//------------------- abs --------------
	case 8:
		dp->ZeroTicker ++;
		if (dp->ZeroTicker  > dp->HoldTime )
		{
			dp->ZeroTicker  = 0;
			dp->Dref  = 0;
			dp->PwmEn  = 0;
			dp->ZeroSts  = 9;
		}
		dp->EzEepErr = dp->EzQepFbk % dp->QepElc;
		break;
	case 9:
		// MontorVar.PosErrQep = (s16)dp->EzEepErr;

		//   AbsPar.WriteInit[0] = dp->EzEepErr &0xFF;
		//   AbsPar.WriteInit[1] = (dp->EzEepErr>>8) &0xFF;
		// AbsPar.WriteInit[2] = (dp->EzEepErr>>16) &0xFF;

		// //ABS_TASK_PTR = &ABS_READ;
		// ABS_TASK_PTR = &ABS_INIT_WRITE;  //ABS_READ	;	//
		eep_par.motor_abs_zero = (s16)dp->EzEepErr;
		flash_save_enable = 1;

		dp->ZeroSts  = 10;
		break;
	case 10:
		encode_zero =  -(s16)dp->EzEepErr;
		MotoZeroVar.Sts = 0;
		dp->ZeroSts = 0;
		break;
	}

}

/*********************************************************************/
// spd 转化为电角度的增量， spd/60 *poles /freq  = spd/60*4 /20000
void openloopTaskMacro(MOTO_ZERO_TYPEDEF *dp)
{
	if (dp->ZeroEn  == 0) {
		dp->Dref  = 0.0f;
		dp->Qref  = 0.0f;
		dp->Theta  = 0.0f;
		dp->PwmEn  = 0;
		dp->ZeroSts  = 0;
		//--------------------------
		dp->DrefAddTim = 10000;
		dp->DrefHoldTim = 10000;
		dp->refSpd = 30;

		dp->stop = 0;


		return;
	}

	switch (dp->ZeroSts)
	{
	case 0:
		dp->Dref  = 0.0f;
		dp->Qref  = 0.0f;
		dp->Theta  = 0.0f;
		dp->PwmEn  = TRUE;
		if (dp->ZeroTicker < 100)
		{
			dp->ZeroTicker ++;
		} else {
			dp->ZeroTicker  = 0;
			dp->ZeroSts  = 1;
		}
		break;
	case 1:
		dp->Dref  += (dp->Dgvn / dp->DrefAddTim);		//4KHZ
		if (dp->Dref  > dp->Dgvn ) {
			dp->Dref  = dp->Dgvn ;
			dp->ZeroSts  = 2;
		}
		break;
	case 2:
		dp->ZeroTicker ++;
		if ( dp->ZeroTicker  >= dp->DrefHoldTim)
		{
			dp->Dref  = dp->Dgvn * 0.8f;
			dp->ZeroTicker  = 0;

			dp->ZeroSts  = 3;
			/*
			if(dp->Mode == ZERO_MODE_ABZ){
			  dp->ZeroSts  = 3;
			}else{
			  dp->ZeroSts  = 8;
			}
			*/

		}
		break;
	case 3:
		dp->ZeroTicker ++;
		dp->Theta  += dp->refSpd * 3.333333333 * 0.000001;
		if (dp->Theta  > 1.0f) dp->Theta  -= 1.0f;


		encode_zero =   gSpeed.SpeedFiterRpm;
		if (dp->stop)
		{
			dp->Dref  = 0;
			dp->PwmEn  = 0;
			dp->ZeroSts++;
		}
		break;
	case 4:

		MotoZeroVar.Sts = 0;
		dp->ZeroSts = 0;

		break;
	}
}