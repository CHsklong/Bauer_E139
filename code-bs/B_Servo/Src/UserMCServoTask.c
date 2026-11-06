/*
 * UserMCServoTask.c
 *
 *  Created on: 2016414
 *      Author: admin
 */

#include "SysMain.h"
//ATTR_RAMFUNC float32_t riscv_dsp_cos_f32(float32_t src);
float id_ref = 0.0f;

void (*MOTO1_SERVO_TASK_Api_Ptr)(void);

//uint8_t spi1_state_rx_chn ;
int32_t runTime3;
ATTR_RAMFUNC void MC_SERVO_TASK(void)
{
	//SYS_LED_HIGH();
        write_csr(CSR_MCYCLE, 0);

	McAngPortRead();
	
//         spi1_state_rx_chn = dma_check_transfer_status(HPM_HDMA, HW_SPI_RX_DMA);
	ABS_MOTOR_READ_ANGLE();
	/**************************************************************************************/
	//
	ANGLE_MACRO_ALL(AbsPar.abs, AbsPar.absIncOut); //
	/**************************************************************************************/


	/**************************************************************************************/
	if (gPwmIsrTacker == SPEED_LOOP_PSR) {
		//  1.6us

		if (gQepAngle.QepType < ENC_ABS_17BIT) {			// ABZ
			gSpeed.FirstIndexFlag = gQepAngle.FirstIndexFlag;
			if (gSpeed.FirstIndexFlag)
			{
				id_ref = 0;
			}
			gSpeed.FirstIndexMechTheta = gQepAngle.FirstIndexMechTheta;

		}


		gSpeed.MechTheta = gQepAngle.MechTheta;
		SPEED_MACRO_ALL(gSpeed)	//1.4us

		// 3.2us

		McSwPortRead();
		McGainHandle();
		SwInPortLogic();
		SwOutPortLogic();
		MC_Mp_Handle();

		


	} else {

	}
	/**************************************************************************************/


	/**************************************************************************************/
	(*MOTO1_SERVO_TASK_Api_Ptr)();
	/**************************************************************************************/
	//


	/**************************************************************************************/
	//1.7us
	//
	gCurLoopGvn.Theta = gQepAngle.ElecTheta;
        
	gCurLoopGvn.Sine =  hpm_dsp_sin_f32(gQepAngle.ElecTheta * (2.0f * PI));
	gCurLoopGvn.Cosine = hpm_dsp_cos_f32(gQepAngle.ElecTheta * (2.0f * PI));
;
	/*
	gCurLoopGvn.Sine =  arm_sin_f32(gQepAngle.ElecThetaN*2.0f*PI);
	gCurLoopGvn.Cosine = arm_cos_f32(gQepAngle.ElecThetaN*2.0f*PI);
	*/
	gCurLoopGvn.Kp = gMcCtlPar.Kpt;
	gCurLoopGvn.Ki = gMcCtlPar.Kit;
	gCurLoopGvn.Kwe = gMcCtlPar.KWe;
	gCurLoopGvn.PwmEn = gSysCtlVar.Ctl.bit.PwmEn;
	gCurLoopGvn.IdRef = id_ref;
	gCurLoopGvn.CtlMode = 0.0f;
	gCurLoopGvn.Speed = gSpeed.Speed;
	/**************************************************************************************/


	/**************************************************************************************/
	MC_HD_Ang_Port_Read();		//
//	gCurLoopGvn.CurAS = MC_HD_Port.CurU;
//	gCurLoopGvn.CurBS = MC_HD_Port.CurV;
	if ( AngVar_Vbus.Out < SysParP7.LVH) gCurLoopGvn.Vbus = SysParP7.LVH;
	else gCurLoopGvn.Vbus = AngVar_Vbus.Out;
	/**************************************************************************************/

	/**************************************************************************************/
	CurLoopMacro();
	/**************************************************************************************/

	/**************************************************************************************/
	//
	if (gPwmIsrTacker == SPEED_LOOP_PSR)
	{
		McSwPortLoad();
	}
	/**************************************************************************************/
        runTime3 = read_csr(CSR_MCYCLE);
        //SYS_LED_LOW();

	/**************************************************************************************/

}


ATTR_RAMFUNC void MOTO1_SERVO_TASK_SEL(void)
{

	if (SysParP0.CTL == SERVO_MODE_POS) //
	{
		MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Pos_Init;
	} else if (SysParP0.CTL == SERVO_MODE_SPD) //
	{
		MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Spd_Init;
	} else if (SysParP0.CTL == SERVO_MODE_TOR) //
	{
		MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Tor_Init;
	} else if (SysParP0.CTL == SERVO_MODE_P_S) //
	{
		if (McSwPort.In.bit.S_P == 0)
		{
			MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Pos_Init;
		} else {
			MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Spd_Init;
		}
	} else if (SysParP0.CTL == SERVO_MODE_S_T) //
	{
		if (McSwPort.In.bit.S_T == 0)
		{
			MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Spd_Init;
		} else {
			MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Tor_Init;
		}
	} else if (SysParP0.CTL == SERVO_MODE_P_T) //
	{
		if (McSwPort.In.bit.T_P == 0)
		{
			MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Pos_Init;
		} else {
			MOTO1_SERVO_TASK_Api_Ptr = &Moto1_Tor_Init;
		}
	}

}


