#include <stdint.h>
#include <float.h>
#include <math.h>
#include "board.h"
#include "motor.h"
#include "m1_sm_ref_sol.h"
#include "upgrade.h"
#include "param.h"
#include "alert.h"
#include "tick.h"
#include "hal_for_upg.h"
#include "servo_can.h"
#include "mcdrv.h"
#include "pwm.h"
#include "work_io.h"
#include "jacking_motor.h"
#include "av_data.h"
#include "pid_regulator.h"
////////////////////const define /////////////////////////////

////////////////////struct define//////////////////////////////

//////////////////local varible define////////////////////////////////
uint16_t mot_id = 1;
Mot_Run_State mot_run_state;
Motor motor;
Motor_RUN motor_run;
Mot_Auto_SR_Pos_T mot_auto_sr_pos_st = {
	.sr_type = 0,
	.send_enable = 0,
	.send_period_ms = 1,
	.resv_pos = 0,
	.sr_canid = 0x02,
	.sensor_port = 2,
	.offset_pos = 0,
};
uint32_t cur_temp_tick = 0;
Mot_Ext_Pos_Ctrl_ST mot_ext_pos_ctrl_st;
PID_Handle_t PID_ext_pos_ctrl =
{
		.hDefKpGain = (int16_t)1, // 5000
		.hDefKiGain = (int16_t)1, // 50
		.hKpGain = (int16_t)1,	  // 5000
		.hKiGain = (int16_t)1,	  // 50
		.hUpperOutputLimit = (int16_t)30000,
		.hLowerOutputLimit = -30000,
		.hKpDivisor = (uint16_t)65535,
		.hKiDivisor = (uint16_t)65535,
		.wUpperIntegralLimit = (int32_t)65535 * (1000), // 1000
		.wLowerIntegralLimit = -(int32_t)65535 * (1000),
		.hKpDivisorPOW2 = (uint16_t)0,
		.hKiDivisorPOW2 = (uint16_t)0,
		.hDefKdGain = 0x0000U,
		.hKdDivisor = 10,
		.hKdDivisorPOW2 = 0x0000U,
		.hIncRamp = 0,
		.hDecRamp = 0,

};
//////////////////local function define////////////////////////////////

static void ledLight(void);

//------------motor run-----------------------------------
void initMotor(void)
{
	memset(&mot_run_state, 0, sizeof(Mot_Run_State));
	mot_run_state.lightSpd = 1000;
	mot_run_state.run_mode = RUN_MODE_RUN;
	mot_run_state.report_type = 0;
	mot_run_state.signal_report = 0;
	mot_run_state.zero_mode = 0;
	mot_run_state.sensor_flag = 0;
	mot_run_state.zeroStPos = 0;
	g_sMID.sMIDEnc.ui16Step = 0;
	g_sMID.sMIDEnc.pui16AlignPos[0] = &g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][0];
	g_sMID.sMIDEnc.pui16AlignPos[1] = &g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[1][0];
	mot_run_state.change_pid_flag = 0;
	mot_run_state.zero_set_pos = 20000;
	mot_run_state.stop_flag = 1;
	mot_run_state.punching_state = 0;
	mot_run_state.punching_cnt = 0;
	mot_run_state.mot_move_time_param.dis_time = 1;
        mot_run_state.fun_test_flag = 0;
	mot_run_state.pos_over_zero_flag = 0;
	mot_run_state.pos_over_zero_report_enable = 0;
	cur_temp_tick = getTicks();
        mot_run_state.ext_ctrl_pos_enable = 0 ;
        mot_run_state.speed_right_pos = FLT_MAX ;
        mot_run_state.speed_left_pos = -FLT_MAX ;
}
void initMotorPos(void)
{
	float circle_max_pos = 0.0f;
	float cur_pos = 0.0f;
	if (mot_run_state.zeroPos != 0)
	{
		mot_run_state.zeroOffset = 0;
		if (mot_run_state.runDir == 1)
			circle_max_pos = posRatioIn2Out(360.0f);
		else
			circle_max_pos = -posRatioIn2Out(360.0f);
		mot_run_state.zeroOffset = mot_run_state.zeroPos;
		cur_pos = posIn2Out(getCurPos());
		if (cur_pos < mot_run_state.leftLmt)
		{
			cur_pos = cur_pos + circle_max_pos;
		}
		else if (cur_pos > mot_run_state.rightLmt)
		{
			cur_pos = cur_pos - circle_max_pos;
		}
		setCurPos(cur_pos);
	}
	else
		mot_run_state.zeroOffset = 0;
}

void moveTorq(float fTorq)
{
	if (fTorq > g_sM1Drive.fltInom)
		fTorq = g_sM1Drive.fltInom;
	else if (fTorq < -g_sM1Drive.fltInom)
		fTorq = -g_sM1Drive.fltInom;

	g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;
	g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ = fTorq;
	mot_run_state.motRun = true;
	mot_run_state.stop_flag = 0;
	mot_run_state.torque_check_over_spd_send_flag = true;
	g_sM1Drive.eControl = kControlMode_CurrentFOC;
}
void moveScalarCtrl(float fSpd, float fTorq)
{
	/*g_sM1Drive.eControl = kControlMode_Scalar ;
	g_sM1Drive.sScalarCtrl.fltFreqRamp = 300.0F;
	g_sM1Drive.sScalarCtrl.sUDQReq.fltQ = 10;
	g_sM1Drive.sScalarCtrl.fltFreqCmd = fSpd ;
	M1_SetAppSwitch(true);*/
	motOpenSpd();
}
void moveSpd(float fSpd, float fTorq)
{
	if (fTorq >= 0)
	{
		if (fTorq > g_sM1Drive.fltInom)
			fTorq = g_sM1Drive.fltInom;

		g_sM1Drive.sSpeed.fltIqLimit = fTorq;
		mot_run_state.torqueReq = fTorq;
	}
	else
	{
		g_sM1Drive.sSpeed.fltIqLimit = FLOAT_PLUS_MAX;
		mot_run_state.torqueReq = g_sM1Drive.fltInom;
	}
	if (fSpd > LIMIT_SPD)
	{
		M1_SetSpeed(LIMIT_SPD);
	}
	else if (fSpd < -LIMIT_SPD)
	{
		M1_SetSpeed(-LIMIT_SPD);
	}
	else
		M1_SetSpeed(fSpd);
	g_sM1Drive.eControl = kControlMode_SpeedFOC;
	mot_run_state.motRun = true;
	mot_run_state.stop_flag = 0;
}

void movePos(float fPos, float fSpd, float fTorq)
{

	if (fSpd >= 0)
	{
		if (fSpd > LIMIT_SPD)
			g_sM1Drive.sPosCtrl.fltSpeedLimit = LIMIT_SPD;
		else if (fSpd < MIN_SPD)
			g_sM1Drive.sPosCtrl.fltSpeedLimit = MIN_SPD;
		else
		{
			g_sM1Drive.sPosCtrl.fltSpeedLimit = fSpd;
		}
	}
	else
		g_sM1Drive.sPosCtrl.fltSpeedLimit = -fSpd;
	g_sM1Drive.sPosCtrl.u8PosRampType = POS_T_RAMP;
	g_sM1Drive.sSpeed.fltIqLimit = FLOAT_PLUS_MAX;
	g_sM1Drive.eControl = kControlMode_PosFOC;
	mot_run_state.posReq = fPos;
	M1_SetPosition(fPos);
	mot_run_state.motRun = true;
	mot_run_state.stop_flag = 0;
	mot_run_state.speedReq = g_sM1Drive.sPosCtrl.fltSpeedLimit;
	mot_run_state.torqueReq = g_sM1Drive.fltInom;
	mot_run_state.arr_pos_flag = 0;
}
void moveSRampPos(float fPos, float fSpd, float fTorq)
{
	if (fSpd >= 0)
	{
		if (fSpd > LIMIT_SPD)
			g_sM1Drive.sPosCtrl.fltSpeedLimit = LIMIT_SPD;
		else if (fSpd < MIN_SPD)
			g_sM1Drive.sPosCtrl.fltSpeedLimit = MIN_SPD;
		else
		{
			g_sM1Drive.sPosCtrl.fltSpeedLimit = fSpd;
		}
	}
	else
		g_sM1Drive.sPosCtrl.fltSpeedLimit = -fSpd;
	g_sM1Drive.sPosCtrl.u8PosRampType = POS_S_RAMP;
	g_sM1Drive.sSpeed.fltIqLimit = FLOAT_PLUS_MAX;
	g_sM1Drive.eControl = kControlMode_PosFOC;
	mot_run_state.posReq = fPos;
	M1_SetPosition(fPos);
	mot_run_state.motRun = true;
	mot_run_state.stop_flag = 0;
	mot_run_state.speedReq = g_sM1Drive.sPosCtrl.fltSpeedLimit;
	mot_run_state.torqueReq = g_sM1Drive.fltInom;
	mot_run_state.arr_pos_flag = 0;
}
void motor_update_speed(float fSpd) // 速度模式和位置模式下 实时更新速度
{

	if (g_sM1Drive.eControl == kControlMode_SpeedFOC)
	{
		if (M1_GetSpeed() > 0)
			M1_SetSpeed(fSpd);
		else if (M1_GetSpeed() == 0)
			M1_SetSpeed(0);
		else
			M1_SetSpeed(-fSpd);
	}
	else if (g_sM1Drive.eControl == kControlMode_PosFOC)
	{
		if (fSpd >= 0)
		{
			if (fSpd > LIMIT_SPD)
				g_sM1Drive.sPosCtrl.fltSpeedLimit = LIMIT_SPD;
			else if (fSpd < MIN_SPD)
				g_sM1Drive.sPosCtrl.fltSpeedLimit = MIN_SPD;
			else
			{
				g_sM1Drive.sPosCtrl.fltSpeedLimit = fSpd;
			}
		}
		else
			g_sM1Drive.sPosCtrl.fltSpeedLimit = -fSpd;
	}
}
void motWorkParamApply(void)
{
	// float_t W = 2.0f * FLOAT_PI / 60.0f;
	// float_t bw = 2.0f * FLOAT_PI * (float)motor_run.spdBW;
	// float_t J = (float)motor_run.inertia / 10000.0f;
	// float_t Kt = 3 * motor.ke / sqrt(3);
	float_t UmaxCoeff = 1.73205f;
#if (BOARD_TYPE == BOARD_E145)
	float_t Att = 2.0f;
#else
	float_t Att = 1.0f;
#endif

	//  g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = W * 2 * bw * J / Kt;
	//  g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = W * bw * bw * J * M1_SLOW_LOOP_TS / (Kt * 10);
	g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = motor_run.spdPidP;
	g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = motor_run.spdPidI / 1000.0f;
	g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = (float_t)motor_run.posPip;

	g_sM1Drive.sSpeed.sSpeedPiParams.fltUpperLim = (float)(motor_run.spdILmt * motor_run.over_load_scale);
	g_sM1Drive.sSpeed.sSpeedPiParams.fltLowerLim = -(float)(motor_run.spdILmt * motor_run.over_load_scale);
	// g_sM1Drive.sSpeed.u8RampType = (uint8_t)motor_run.ramp_type;
	g_sM1Drive.sSpeed.u8RampType = SPEED_T_RAMP;
#if (BOARD_TYPE == BOARD_E914)
	g_sM1Drive.sFocPMSM.sIdPiParams.fltPGain = 2 * Att * 2 * FLOAT_PI * motor_run.IBW * motor.ld - motor.rs;
	g_sM1Drive.sFocPMSM.sIdPiParams.fltIGain = pow((2 * FLOAT_PI * motor_run.IBW), 2) * motor.ld * M1_FAST_LOOP_TS * 0.1f;
	g_sM1Drive.sFocPMSM.sIqPiParams.fltPGain = 2 * Att * 2 * FLOAT_PI * motor_run.IBW * motor.lq - motor.rs;
	g_sM1Drive.sFocPMSM.sIqPiParams.fltIGain = pow((2 * FLOAT_PI * motor_run.IBW), 2) * motor.lq * M1_FAST_LOOP_TS * 0.1f;
#else
	g_sM1Drive.sFocPMSM.sIdPiParams.fltPGain = 2 * Att * 2 * FLOAT_PI * motor_run.IBW * motor.ld - motor.rs;
	g_sM1Drive.sFocPMSM.sIdPiParams.fltIGain = pow((2 * FLOAT_PI * motor_run.IBW), 2) * motor.ld * M1_FAST_LOOP_TS;
	g_sM1Drive.sFocPMSM.sIqPiParams.fltPGain = 2 * Att * 2 * FLOAT_PI * motor_run.IBW * motor.lq - motor.rs;
	g_sM1Drive.sFocPMSM.sIqPiParams.fltIGain = pow((2 * FLOAT_PI * motor_run.IBW), 2) * motor.lq * M1_FAST_LOOP_TS;

#endif
	g_sM1Drive.sFocPMSM.fltDutyCycleLimit = (float)motor_run.ILmt / (100.0f * UmaxCoeff);
	//    g_sM1Drive.sPosCtrl.bVFF = motor_run.VFF;
}

void motSysParamApply(void)
{
	motor.dis_over_load = 0.0f;
	g_fltM1speedAngularScale = (60.0F / (motor.pp * 2.0F * FLOAT_PI));

	g_sM1Drive.ui16Pp = motor.pp;
	g_sM1Drive.sPosCtrl.fltSpeedLimit = motor.N_nom / g_fltM1speedAngularScale;
	g_sM1Drive.sFaultThresholds.fltSpeedNom = motor.N_nom / g_fltM1speedAngularScale;
	// g_sM1Drive.sSpeed.fltSpeedNom = motor.N_nom / g_fltM1speedAngularScale;
	g_fltM1currentScale = motor.I_max;
	g_fltM1DCBvoltageScale = motor.UDCB_max;
	g_sM1Drive.sFaultThresholds.fltUDcBusOver = motor.UDCB_over;
	g_sM1Drive.sFaultThresholds.fltUDcBusUnder = motor.UDCB_under;
	g_sM1Drive.sFaultThresholds.fltUDcBusTrip = motor.UDCB_trip;

	g_sM1Drive.sFaultThresholds.fltSpeedOver = motor.N_over / g_fltM1speedAngularScale;
	g_sM1Drive.sFaultThresholds.fltSpeedMin = motor.N_min / g_fltM1speedAngularScale;
	g_sM1Drive.sFaultThresholds.fltSpeedNom = motor.N_max / g_fltM1speedAngularScale;
	g_sM1Drive.sFaultThresholds.fltUqBemf = motor.E_block;
	g_sM1Drive.sFaultThresholds.ui16BlockedPerNum = motor.E_block_pre;
	g_sM1Drive.fltInom = motor.Iph_nom * motor_run.over_load_scale;
	motor.dis_over_load = (motor_run.over_load_scale - 1.0f) / over_load_groups;
	if (motor.dis_over_load > 0)
	{
		for (int i = 0; i < over_load_groups; i++)
		{

			motor.over_load_thresshold[i] = motor.Iph_nom * (1 + (i + 1) * motor.dis_over_load);
		}
	}
	motWorkParamApply();
}
void motHalfCurrentLock(void)
{
	mot_run_state.stop_flag = 1;
	g_sM1Drive.sSpeed.fltIqLimit = g_sM1Drive.fltInom / 2;
	mot_run_state.torqueReq = g_sM1Drive.fltInom / 2;
	g_sM1Drive.eControl = kControlMode_PosFOC;
	g_sM1Drive.sPosCtrl.fltSpeedLimit = 1000 / g_fltM1speedAngularScale;
	mot_run_state.posReq = g_sM1Drive.sPosCtrl.fltPositionFilt;
	mot_run_state.arr_pos_flag = 0;
	M1_SetPosition(g_sM1Drive.sPosCtrl.fltPositionFilt);
}
void moveStop(uint8_t emerg)
{
	mot_run_state.stop_flag = 1;
	if (emerg)
	{
		M1_SetSpeed(0.0);
#if (BOARD_TYPE == BOARD_E139)
		brake_engage();                   /* 急停先机械抱闸，立即保持负载 */
#endif
		M1_SetAppSwitch(false);           /* 再关驱动 */
		mot_run_state.motRun = false;
	}
	else
	{
		/*	M1_SetSpeed(0.0);
			g_sM1Drive.eControl = kControlMode_SpeedFOC;
			g_sM1Drive.sSpeed.fltIqLimit = FLOAT_PLUS_MAX;    
					mot_run_state.motRun = true;*/
		g_sM1Drive.sSpeed.fltIqLimit = FLOAT_PLUS_MAX;
		mot_run_state.torqueReq = g_sM1Drive.fltInom;
		g_sM1Drive.eControl = kControlMode_PosFOC;
		g_sM1Drive.sPosCtrl.fltSpeedLimit = 1000 / g_fltM1speedAngularScale;
		mot_run_state.posReq = g_sM1Drive.sPosCtrl.fltPositionFilt;
		mot_run_state.arr_pos_flag = 0;
		M1_SetPosition(g_sM1Drive.sPosCtrl.fltPositionFilt);
		if (mot_run_state.motRun)
		{
			/* if(g_sM1Drive.eControl == kControlMode_SpeedFOC)
				M1_SetSpeed(0.0);
				else//if (g_sM1Drive.eControl == kControlMode_PosFOC)
				{
		  g_sM1Drive.eControl = kControlMode_PosFOC;
		  M1_SetPosition(g_sM1Drive.sPosCtrl.fltPositionFilt);
				}				*/

			/*g_sM1Drive.eControl = kControlMode_PosFOC;
                  M1_SetPosition(g_sM1Drive.sPosCtrl.fltPositionFilt); */
		}
	}
}

void moveResume()
{
	switch (g_sM1Drive.eControl)
	{
	case kControlMode_SpeedFOC:
		moveSpd(mot_run_state.speedReq, mot_run_state.torqueReq);
		break;
	case kControlMode_CurrentFOC:
		moveTorq(mot_run_state.torqueReq);
		break;
	case kControlMode_PosFOC:
		movePos(mot_run_state.posReq, mot_run_state.speedReq, mot_run_state.torqueReq);
		break;
	case kControlMode_Scalar:
		break;
	case kControlMode_VoltageFOC:
		break;
	}
}

static void repFindSideEnd(void)
{
	int32_t pos = posIn2Out(getCurPos());
	uint8_t cmd[8];
	cmd[0] = CMD_FIND_DIST;
	cmd[1] = mot_id; // motor id
	SET_DWORD(pos, cmd[2], cmd[3], cmd[4], cmd[5]);
	BOARD_CanSend(CAN_SEND_ID, cmd, 6);
}

// period: check period (us)
// tryTime: try times.
// return:
// < 0 : no mean
// 0: over time
// 1: success
int moveIsStop(uint32_t period, uint32_t tryTime, uint8_t resh)
{
	static uint32_t tickstart = 0;
	static bool reflash = true;
	static float curPos = FLT_MAX;
	int finished = -1;
	static uint32_t tryCnt;
	if (resh)
	{
		reflash = TRUE;
	}
	if (reflash)
	{
		tickstart = getTicks();
		reflash = false;
		tryCnt = 0;
		curPos = g_sM1Drive.sPosCtrl.fltPositionFilt;
	}
	else if (tryCnt > tryTime)
	{
		reflash = true;
		finished = 0;
	}
	else if (expireTicks(tickstart, period))
	{
		if (fabs(curPos - g_sM1Drive.sPosCtrl.fltPositionFilt) < 3)
		{
			reflash = true;
			finished = 1;
		}
		curPos = g_sM1Drive.sPosCtrl.fltPositionFilt;
		tryCnt++;
		tickstart = getTicks();
	}
	return finished;
}

static bool findSideSm(void)
{
	int ret = moveIsStop(1000, 10, 0);
	bool finished = false;

	if (ret == 1)
	{
		moveStop(false);
		repFindSideEnd();
		finished = true;
	}
	else if (ret == 0)
	{
		moveStop(false);
		alertPush(ALT_NOT_FIND_SIZE, 0);
		finished = true;
	}

	return finished;
}

static void zeroStart(float fSpd, float fTorq)
{
	float rev_dir = 1.0f;
	if (mot_run_state.motor_dir_rev == 1)
		rev_dir = -1.0f;
	mot_run_state.run_mode = RUN_MODE_ZERO;
	mot_run_state.state &= ~(MOT_STATE_ZERO_FINISHED);
	// machine_reset();
	// enableMotor(true);
	if (motor.dir == 0)
	{
		if (mot_run_state.zeroDir == 1)
			moveSpd(-fSpd * rev_dir, fTorq);
		else
			moveSpd(fSpd * rev_dir, fTorq);
	}
	else
	{
		if (mot_run_state.zeroDir == 1)
			moveSpd(fSpd * rev_dir, fTorq);
		else
			moveSpd(-fSpd * rev_dir, fTorq);
	}
}

static void repZeroEnd(int32_t pos)
{
	uint8_t cmd[8];

	cmd[0] = CMD_COND_RESP;
	cmd[1] = mot_id;					// motor id
	cmd[2] = mot_run_state.report_type; //
	cmd[3] = 0;

	SET_DWORD(pos, cmd[4], cmd[5], cmd[6], cmd[7]);
	BOARD_CanSend(CAN_SEND_ID, cmd, 8);
}
void saveZeroPos(void)
{
	Param_Data paraData;
	paraData.f4_data = getCurPos();
	if (mot_run_state.motRun == true)
	{
		enableMotor(false);
		hal_delayms(50);
	}

	// mot_run_state.nume = paraData.i2_data ;
	writeParameter(PARNO_ZERO_POS, paraData);
	// if(mot_run_state.motRun == true)
	/*{
		enableMotor(true) ;
	}*/
}

static bool zero_sensor(void)
{
	bool finished = false;
	static int32_t cur_pos = 0;
#if 1
	int ret;

	switch (mot_run_state.sub_mode)
	{
	case 0: //
		// ret = BOARD_Input(0);
		ret = work_input_get_state(IN_INPUT1);
		if (ret == 1)
		{
			// zero position has finded.
			moveStop(0);
			cur_pos = posIn2Out(getCurPos());
			if ((mot_run_state.zero_mode & 1) == 0) //
			{
				setCurPos(mot_run_state.zeroPos);
			}

			if ((mot_run_state.zero_mode & 2) == 0) //
			{
				saveZeroPos();
			}
			mot_run_state.sub_mode = 2;
			mot_run_state.state |= MOT_STATE_ZERO_FINISHED;
		}
		break;

	case 1:										// inverse move to zStop position
		if ((mot_run_state.zero_mode & 8) != 0) //
		{

			if (mot_run_state.zeroStPos != 0)
			{
				if (mot_run_state.runDir != mot_run_state.zeroDir)
					movePos(posOut2In(mot_run_state.zeroStPos + mot_run_state.zeroPos), MLIB_Abs_FLT(mot_run_state.mot_zero_spd), 1000);
				else
					movePos(posOut2In(mot_run_state.zeroPos - mot_run_state.zeroStPos), MLIB_Abs_FLT(mot_run_state.mot_zero_spd), 1000);
			}
			/* else
			 {
				   if (mot_run_state.repZero)
				   {
   hal_delayms(50);
   mot_run_state.repZero = false ;
   repZeroEnd(cur_pos);

				   }

				   finished = true;
				   mot_run_state.state |= MOT_STATE_READY;
				   mot_run_state.sub_mode = 0;
				   break;

			   }*/
		}
		else
		{

			movePos(posOut2In(mot_run_state.zeroStPos), MLIB_Abs_FLT(mot_run_state.mot_zero_spd), 1000);
		}

		mot_run_state.state &= ~(MOT_STATE_READY);
		mot_run_state.sub_mode = 2;
		break;

	case 2: // wait finished
		if (moveIsStop(1000, 10, 0) == 1)
		{
			if (mot_run_state.repZero)
			{
				mot_run_state.repZero = false;
				repZeroEnd(cur_pos);
			}
			mot_run_state.zero_flag = 1;
			finished = true;
			mot_run_state.state |= MOT_STATE_READY;
			mot_run_state.sub_mode = 0;
		}
		break;
	}
#endif
	return finished;
}
static bool zeroTorqSm(void)
{
	bool finished = false;
	static int32_t cur_pos = 0;
#if 1
	int ret;
	switch (mot_run_state.sub_mode)
	{
	case 0: // wait move to side and stop
		ret = moveIsStop(500, 10, 0);
		if (ret == 1)
		{
			// zero position has finded.
			cur_pos = posIn2Out(getCurPos());
			if ((mot_run_state.zero_mode & 1) == 0) //
			{
				setCurPos(mot_run_state.zeroPos);
			}
#if 0
				enableMotor(true);
				if((mot_run_state.zero_mode &2) == 0) //
				{
					
						saveZeroPos();
				}
#endif
			mot_run_state.sub_mode = 1;
			mot_run_state.state |= MOT_STATE_ZERO_FINISHED;
		}
		/*else if (ret == 0)
		{
			alertPush(ALT_ZERO_TIMEOUT);
			finished = true;
		}*/
		break;

	case 1:										// inverse move to zStop position
		if ((mot_run_state.zero_mode & 8) != 0) //
		{
			mot_run_state.target_pos_offset = 0.2f;
			if (mot_run_state.zeroStPos != 0)
			{
				if (mot_run_state.runDir != mot_run_state.zeroDir)
					movePos(posOut2In(mot_run_state.zeroPos + mot_run_state.zeroStPos), MLIB_Abs_FLT(mot_run_state.mot_zero_spd), 1000);
				else
					movePos(posOut2In(mot_run_state.zeroPos - mot_run_state.zeroStPos), MLIB_Abs_FLT(mot_run_state.mot_zero_spd), 1000);
			}
			else
			{
				if (mot_run_state.repZero)
				{

					mot_run_state.repZero = false;
					repZeroEnd(cur_pos);
				}

				finished = true;
				mot_run_state.state |= MOT_STATE_READY;
				mot_run_state.sub_mode = 0;
				break;
			}
		}
		else
		{
			mot_run_state.target_pos_offset = 0.2f;
			if (mot_run_state.runDir != mot_run_state.zeroDir)
				movePos(posOut2In(mot_run_state.zeroPos + mot_run_state.zeroStPos), MLIB_Abs_FLT(mot_run_state.mot_zero_spd), 1000);
			else
				movePos(posOut2In(mot_run_state.zeroPos - mot_run_state.zeroStPos), MLIB_Abs_FLT(mot_run_state.mot_zero_spd), 1000);
		}

		mot_run_state.state &= ~(MOT_STATE_READY);
		mot_run_state.sub_mode = 2;
		break;

	case 2: // wait finished
		if (mot_run_state.arr_pos_flag == 1)
		{

			if (mot_run_state.repZero)
			{
				mot_run_state.repZero = false;
				repZeroEnd(cur_pos);
			}
			mot_run_state.zero_flag = 1;
			finished = true;
			mot_run_state.state |= MOT_STATE_READY;
			mot_run_state.sub_mode = 0;
		}
		break;
	}
#endif
	return finished;
}

float torqIn2Out(float fTorq)
{
	return fTorq * 1000.0f / g_sM1Drive.fltInom;
}

float torqOut2In(float fTorq)
{
	float rev_dir = 1.0f;
	if (mot_run_state.motor_dir_rev == 1)
		rev_dir = -1.0f;
	if (fTorq >= 1000.0f)
		fTorq = 1000.0f;
	else if (fTorq <= -1000.f)
		fTorq = -1000.0f;
	
	//if (mot_run_state.runDir == 0) // 0
         if (mot_run_state.runDir == 1) // 0
		fTorq = (-fTorq) * rev_dir;
	return fTorq / 1000.0f * g_sM1Drive.fltInom;
}

float spdIn2Out(uint8_t mot_msg, float fSpd)
{
	float ret = 0;
	float rev_dir = 1.0f;
	if (mot_run_state.motor_dir_rev == 1)
		rev_dir = -1.0f;
	switch (mot_msg)
	{
	case MOT_MSG_ZERO:
		ret = fSpd * (POS_SPD_SCALE * g_fltM1speedAngularScale);
		break;
	case MOT_MSG_MOVE_POS:
		ret = fSpd * (POS_SPD_SCALE * g_fltM1speedAngularScale);
		break;
	case MOT_MSG_MOVE_SPD:
		ret = fSpd * (motor.speed_loop_rpm01 * POS_SPD_SCALE * g_fltM1speedAngularScale);
		break;
	default:
		ret = fSpd * (1.0f * g_fltM1speedAngularScale);
		break;
	}
	/*if(mot_run_state.runDir ==0)
		  return -ret ;
	  else
		  return  ret ;*/
	if (motor.dir == 0)
	{
		if (mot_run_state.runDir == 0)
			return (-ret) * rev_dir;
		else
			return ret * rev_dir;
	}
	else
	{
		if (mot_run_state.runDir == 1)
			return (-ret) * rev_dir;
		else
			return ret * rev_dir;
	}
}

float spdOut2In(uint8_t mot_msg, float fSpd)
{
	float ret = 0;
	float rev_dir = 1.0f;
	if (mot_run_state.motor_dir_rev == 1)
		rev_dir = -1.0f;
	switch (mot_msg)
	{
	case MOT_MSG_ZERO:
		ret = fSpd * motor.gear_ratio / g_fltM1speedAngularScale;
		return ret;
	case MOT_MSG_MOVE_POS:
		ret = fSpd * motor.gear_ratio / g_fltM1speedAngularScale;
		break;
	case MOT_MSG_MOVE_SPD:

		ret = fSpd * motor.gear_ratio / (motor.speed_loop_rpm01 * g_fltM1speedAngularScale);

		if (ret < MIN_SPD && ret > 0)
			ret = MIN_SPD;
		else if (ret > -MIN_SPD && ret < 0)
			ret = -MIN_SPD;
		break;
	default:
		ret = fSpd / (1.0f * g_fltM1speedAngularScale);
		break;
	}
#if (BOARD_TYPE == BOARD_E145)
#ifdef JACKING_MOTOR_FUN // 升降功能打开
	if (ret > MAX_SPD && ret > 0)
		ret = MAX_SPD;
	else if (ret < -MAX_SPD && ret < 0)
		ret = -MAX_SPD;
#endif
#endif
	if (motor.dir == 0)
	{
		if (mot_run_state.runDir == 1)
			return (-ret) * rev_dir;
		else
			return ret * rev_dir;
	}
	else
	{
		if (mot_run_state.runDir == 0)
			return (-ret) * rev_dir;
		else
			return ret * rev_dir;
	}
}

float posEnc2Deg(float enc)
{

	float rev_dir = 1.0f;
	if (mot_run_state.motor_dir_rev == 1)
		rev_dir = -1.0f;
	if (motor.dir == 0)
	{
#if (BOARD_TYPE == BOARD_E914)
		if (mot_run_state.runDir == 1)
			return (enc * 360.0f * mot_run_state.encDir / 2048.0f) * rev_dir;
		else
			return -(enc * 360.0f * mot_run_state.encDir / 2048.0f) * rev_dir;
#elif (BOARD_TYPE == BOARD_E145)
		if (mot_run_state.runDir == 1)
			return (enc * 360.0f * mot_run_state.encDir / 16384.0f) * rev_dir; // 360
		else
			return -(enc * 360.0f * mot_run_state.encDir / 16384.0f) * rev_dir;
#else
		if (mot_run_state.runDir == 1)
			return (enc * 360.0f * mot_run_state.encDir / 16384.0f) * rev_dir;
		else
			return -(enc * 360.0f * mot_run_state.encDir / 16384.0f) * rev_dir;
#endif
	}
	else
	{
#if (BOARD_TYPE == BOARD_E914)
		if (mot_run_state.runDir == 1)
			return -(enc * 360.0f * mot_run_state.encDir / 2048.0f) * rev_dir;
		else
			return (enc * 360.0f * mot_run_state.encDir / 2048.0f) * rev_dir;
#elif (BOARD_TYPE == BOARD_E145)
		if (mot_run_state.runDir == 1)
			return -(enc * 360.0f * mot_run_state.encDir / 16384.0f) * rev_dir;
		else
			return (enc * 360.0f * mot_run_state.encDir / 16384.0f) * rev_dir;
#else
		if (mot_run_state.runDir == 1)
			return -(enc * 360.0f * mot_run_state.encDir / 16384.0f) * rev_dir;
		else
			return (enc * 360.0f * mot_run_state.encDir / 16384.0f) * rev_dir;
#endif
	}
}

float posDeg2Enc(float deg)
{

	float rev_dir = 1.0f;
	if (mot_run_state.motor_dir_rev == 1)
		rev_dir = -1.0f;
	if (motor.dir == 0)
	{
#if (BOARD_TYPE == BOARD_E914)
		if (mot_run_state.runDir == 1)
			return (deg * 2048.0f * mot_run_state.encDir / 360.0f) * rev_dir;
		else
			return -(deg * 2048.0f * mot_run_state.encDir / 360.0f) * rev_dir;
#elif (BOARD_TYPE == BOARD_E145)
		if (mot_run_state.runDir == 1)
			return (deg * 16384.0f * mot_run_state.encDir / 360.0f) * rev_dir;
		else
			return -(deg * 16384.0f * mot_run_state.encDir / 360.0f) * rev_dir;
#else
		if (mot_run_state.runDir == 1)
			return (deg * 16384.0f * mot_run_state.encDir / 360.0f) * rev_dir;
		else
			return -(deg * 16384.0f * mot_run_state.encDir / 360.0f) * rev_dir;
#endif
	}
	else
	{
#if (BOARD_TYPE == BOARD_E914)
		if (mot_run_state.runDir == 1)
			return -(deg * 2048.0f * mot_run_state.encDir / 360.0f) * rev_dir;
		else
			return (deg * 2048.0f * mot_run_state.encDir / 360.0f) * rev_dir;
#elif (BOARD_TYPE == BOARD_E145)
		if (mot_run_state.runDir == 1)
			return -(deg * 16384.0f * mot_run_state.encDir / 360.0f) * rev_dir;
		else
			return (deg * 16384.0f * mot_run_state.encDir / 360.0f) * rev_dir;
#else
		if (mot_run_state.runDir == 1)
			return -(deg * 16384.0f * mot_run_state.encDir / 360.0f) * rev_dir;
		else
			return (deg * 16384.0f * mot_run_state.encDir / 360.0f) * rev_dir;
#endif
	}
}

float posIn2Out(float fPos)
{
	return (posDeg2Enc(fPos - mot_run_state.zeroOffset) * mot_run_state.deno / mot_run_state.nume);
}

float posOut2In(float fPos)
{
	return (posEnc2Deg(fPos * mot_run_state.nume / mot_run_state.deno) + mot_run_state.zeroOffset);
}
float posRatio2In2Out(float fPos)
{
	return (fPos * mot_run_state.deno2 / mot_run_state.nume2);
}

float posRatio2Out2In(float fPos)
{
	return (fPos * mot_run_state.nume2 / mot_run_state.deno2);
}

float posRatioIn2Out(float fPos)
{
	return (posDeg2Enc(fPos) * mot_run_state.deno / mot_run_state.nume);
}

float posRatioOut2In(float fPos)
{
	return (posEnc2Deg(fPos * mot_run_state.nume / mot_run_state.deno));
}

// note: fpos - out side uint !!!!
void setCurPos(float fPos)
{
	mot_run_state.zeroOffset = g_sM1Drive.sPosCtrl.fltPositionFilt -
							   posEnc2Deg(fPos * mot_run_state.nume / mot_run_state.deno);
}

float getCurPos(void)
{
	// return g_sM1Drive.sPosCtrl.fltPositionFilt;
	return g_sM1Drive.sPosCtrl.fltPosition;
}

float getCurTorq(uint8_t type)
{
	if (type == 0)
		return g_sM1Drive.sFocPMSM.sIDQReq.fltQ;
	else
		return g_sM1Drive.sFocPMSM.sIDQ.fltQ;
}

float getCurSpd(uint8_t type)
{
	if (type == 0)
		return g_sM1Drive.sSpeed.fltSpeedRamp * g_fltM1speedAngularScale;
	else
		return g_sM1Drive.sSpeed.fltSpeedFilt * g_fltM1speedAngularScale;
}

void motSetMasterPos(float pos)
{
	mot_run_state.masterSpd = (pos - mot_run_state.masterPos); // * 120000.0f/360.0f;
	mot_run_state.masterPos = pos;
	syncActionSM();
}
void io_ctrl_set(uint8_t io_idx, uint8_t type, uint16_t lightms)
{
	if (type == 0 || type == 1)
	{
		mot_run_state.io_light_spd = type;
	}
	else
	{
		mot_run_state.io_light_spd = lightms;
	}
}
void io_ctrl(void)
{
	static uint32_t tickstart = 0;
	uint32_t delay = mot_run_state.io_light_spd;

	if (delay == 0)
	{
		USER_LED_OFF();
	}
	else if (delay == 1)
	{
		USER_LED_ON();
	}
	else
	{
		if (expireTicks(tickstart, delay))
		{
			USER_LED_TOGGLE();
			tickstart = getTicks();
		}
	}
}
static void ledLight(void)
{
	static uint32_t tickstart = 0;
	uint32_t delay = mot_run_state.lightSpd;

	if (delay == 0)
	{
		USER_LED_ON();
	}
	else
	{
		if (expireTicks(tickstart, delay))
		{
			USER_LED_TOGGLE();
			tickstart = getTicks();
		}
	}
}

#if (BOARD_TYPE == BOARD_E139)
/* E139 抱闸/使能 安全时序（非阻塞：靠 1ms 节拍推进，不阻塞电机主循环）
 *   启动：先使能驱动建立保持力矩 -> 延时 -> 松闸
 *   停机：先抱闸-> 延时 -> 关闭驱动
 *   把 BRAKE_ENGAGE_HOLD_MS 设为 0 即为“立即抱闸并立即关驱动”。 */
typedef enum {
    BRAKE_SEQ_IDLE = 0,          //空闲时刻
    BRAKE_SEQ_RELEASE_PENDING,   /* 已使能，待松闸 */
    BRAKE_SEQ_RELEASE_BOOST,     /* 松闸强励磁阶段：输出高电平, 持续 BRAKE_BOOST_MS */
    BRAKE_SEQ_ENGAGE_PENDING     /* 已抱闸，待关驱动 */
} brake_seq_t;

static brake_seq_t s_brake_seq = BRAKE_SEQ_IDLE;
static uint32_t   s_brake_seq_tick = 0;
static uint32_t   s_brake_boost_tick = 0;                 /* 强励磁阶段计时基准起点 */
static uint32_t   s_brake_hold_freq = BRAKE_PWM_FREQ_HZ;  /* 强励磁结束后保持的 PWM 频率(Hz) */
static uint8_t    s_brake_hold_duty = BRAKE_PWM_DUTY_PCT; /* 强励磁结束后保持的 PWM 占空比(%) */

#define BRAKE_RELEASE_DELAY_MS   (50U)   /* 使能后等待力矩建立再进入松闸 */
#define BRAKE_ENGAGE_HOLD_MS     (30U)   /* 抱闸后等待机械就位再关驱动，0=立即 */

/* 松闸序列(打开刹车): 先持续输出BRAKE_BOOST_MS（1s）的高电平打开刹车, 再切到50占空比的载波防止电机过热。
 * 非阻塞做法: 用(brake_seq_tick)延时。*/
void brake_open_seq(uint32_t freq_hz, uint8_t duty_pct)
{
    s_brake_seq = BRAKE_SEQ_IDLE;                       /* 撤销任何未执行的启停时序, 置于空闲阶段 */
    s_brake_hold_freq = (freq_hz != 0U) ? freq_hz : BRAKE_PWM_FREQ_HZ;       //判断是否已经外部输入频率
    s_brake_hold_duty = (duty_pct != 0U) ? duty_pct : BRAKE_PWM_DUTY_PCT;
    brake_boost();                                      /* 强励磁：持续输出高电平 */
    s_brake_boost_tick = getTicks();                    // 记下当前时刻a
    s_brake_seq = BRAKE_SEQ_RELEASE_BOOST;              // 记下当前状态为强励磁阶段
}

/* 由 1ms 任务(mot_task_ms_loop)调用，按照时序推进抱闸与松闸 */
void brake_seq_tick(void)
{
    if (s_brake_seq == BRAKE_SEQ_RELEASE_PENDING) 
    {
        if (expireTicks(s_brake_seq_tick, BRAKE_RELEASE_DELAY_MS)) 
        {
            brake_open_seq(BRAKE_PWM_FREQ_HZ, BRAKE_PWM_DUTY_PCT); /* 力矩已建立 -> 进入强励磁->保持序列 */
        }
    } 
    else if (s_brake_seq == BRAKE_SEQ_RELEASE_BOOST) 
    {
        if (expireTicks(s_brake_boost_tick, BRAKE_BOOST_MS))        //判断a时刻到现在有没有BRAKE_BOOST_MS，有则进行下一步
        {
            brake_pwm_config(s_brake_hold_freq, s_brake_hold_duty); /* 配置保持 PWM(默认 10kHz/50%) */
            brake_release();                                       /* 切 GPTMR 输出, 进入保持(打开刹车) */
            s_brake_seq = BRAKE_SEQ_IDLE;
        }
    }
    else if (s_brake_seq == BRAKE_SEQ_ENGAGE_PENDING) 
    {
        if (BRAKE_ENGAGE_HOLD_MS == 0U) 
        {
            M1_SetAppSwitch(false);          /* 立即关驱动（无延时） */
            s_brake_seq = BRAKE_SEQ_IDLE;
        } 
        else if (expireTicks(s_brake_seq_tick, BRAKE_ENGAGE_HOLD_MS)) 
        {
            M1_SetAppSwitch(false);          /* 延时BRAKE_ENGAGE_HOLD_MS关驱动 */
            s_brake_seq = BRAKE_SEQ_IDLE;
        }
    }
}
#endif

void enableMotor(uint8_t enb)
{
#if (BOARD_TYPE == BOARD_E139)
    if (enb) 
    {
        /* 启动：先使能，后松闸，建立保持力矩 */
        M1_SetAppSwitch(true);                      //启动使能
        s_brake_seq = BRAKE_SEQ_RELEASE_PENDING;    //延时后松闸
        s_brake_seq_tick = getTicks();
    } 
    else 
    {   
        /* 关闭：先抱闸，后关使能，建立保持力矩 */
        brake_engage();                             //抱闸
        s_brake_seq = BRAKE_SEQ_ENGAGE_PENDING;     //延时后关闭使能
        s_brake_seq_tick = getTicks();
    }
#else
    M1_SetAppSwitch(enb);
#endif
}

/* CAN协议控制刹车频率 */
void brakeManual(uint8_t release, uint8_t duty_pct, uint8_t freq_khz)
{
#if (BOARD_TYPE == BOARD_E139)
    s_brake_seq = BRAKE_SEQ_IDLE;   /* 撤掉未执行的时序步骤，手动命令优先 */
    if (release) {
        uint8_t  duty = (duty_pct != 0U) ? duty_pct : BRAKE_PWM_DUTY_PCT;
        uint32_t freq = BRAKE_PWM_FREQ_HZ;
        if (freq_khz >= 1U && freq_khz <= 20U) 
        {
            freq = (uint32_t)freq_khz * 1000U;
        }
        brake_open_seq(freq, duty); /* 松闸: 强励磁 1s -> freq/duty% 保持 */
    } else {
        brake_engage();             /* 抱闸: 立即失电锁定(占空比/频率无意义) */
    }
#else
    (void)release;
    (void)duty_pct;
    (void)freq_khz;
#endif
}

int motorIsStop(uint32_t period, uint32_t tryTime, uint8_t resh) // 判断电机是否停止
{
	static uint32_t tickstart = 0;
	static bool reflash = true;
	static float curPos = FLT_MAX;
	int finished = -1;
	static uint32_t tryCnt = 0;
	if (resh)
	{
		reflash = TRUE;
	}
	if (reflash)
	{
		tickstart = getTicks();
		reflash = false;
		tryCnt = 0;
		curPos = getCurPos();
	}
	else if (tryCnt > tryTime)
	{
		reflash = true;
		finished = 1;
	}
	else if (expireTicks(tickstart, period))
	{

		if (fabs(curPos - getCurPos()) < 1)
		{
			tryCnt++;
			finished = 0;
		}
		else
			tryCnt = 0;

		curPos = getCurPos();

		tickstart = getTicks();
	}

	return finished;
}

int moveIsStop1(uint32_t period, uint32_t tryTime, uint8_t resh)
{
	static uint32_t tickstart = 0;
	static bool reflash = true;
	static float curPos = FLT_MAX;
	int finished = -1;
	static uint32_t tryCnt = 0;
	if (resh)
	{
		reflash = TRUE;
	}
	if (reflash)
	{
		tickstart = getTicks();
		reflash = false;
		tryCnt = 0;
		curPos = getCurPos();
	}
	else if (tryCnt > tryTime)
	{
		reflash = true;
		finished = 1;
	}
	else if (expireTicks(tickstart, period))
	{

		if (mot_run_state.arr_pos_flag == 0 || g_sM1Drive.eControl == kControlMode_SpeedFOC)
		{
			if (fabs(curPos - getCurPos()) < 10 && MLIB_Abs_FLT(g_sM1Drive.sFocPMSM.sIDQ.fltQ) > MLIB_Abs_FLT(mot_run_state.torqueReq) * 0.8 && (g_sM1Drive.eControl == kControlMode_PosFOC || g_sM1Drive.eControl == kControlMode_SpeedFOC))
			{
				tryCnt++;
				finished = 0;
			}
			else
				tryCnt = 0;
		}
		curPos = getCurPos();

		tickstart = getTicks();
	}

	return finished;
}
int moveIsLock(uint32_t period, uint32_t tryTime)
{
	static uint32_t tickstart = 0;
	static bool reflash = true;
	static float curPos = FLT_MAX;
	int finished = -1;
	static uint32_t tryCnt = 0;

	if (reflash)
	{
		tickstart = getTicks();
		reflash = false;
		tryCnt = 0;
		curPos = getCurPos();
	}
	else if (tryCnt > tryTime)
	{
		reflash = true;
		finished = 1;
	}
	else if (expireTicks(tickstart, period))
	{

		if (mot_run_state.arr_pos_flag == 0 || g_sM1Drive.eControl == kControlMode_SpeedFOC)
		{
			if (fabs(curPos - getCurPos()) < 10 && MLIB_Abs_FLT(g_sM1Drive.sFocPMSM.sIDQ.fltQ) > MLIB_Abs_FLT(mot_run_state.torqueReq) * 0.8 && mot_run_state.stop_flag == 0 && (g_sM1Drive.eControl == kControlMode_PosFOC || g_sM1Drive.eControl == kControlMode_SpeedFOC))
			{
				tryCnt++;
				finished = 0;
			}
			else
				tryCnt = 0;
		}
		curPos = getCurPos();

		tickstart = getTicks();
	}

	return finished;
}
uint8_t checkMotLock(void)
{
	if (moveIsLock(1000, 5) == 1)
	{
		return 1;
	}

	return 0;
}
uint8_t checkMotOverload(void) //?
{
	static uint32_t cut_tick = 0;
	uint8_t ret = 0;
	float cur_current = MLIB_Abs_FLT(g_sM1Drive.sFocPMSM.sIDQ.fltQ);
	if (expireTicks(cut_tick, 1))
	{
		if (cur_current > motor.Iph_nom)
		{

			if (cur_current >= motor.over_load_thresshold[3])
			{
				motor.over_load_cnt[4]++;
			}
			else if (cur_current >= motor.over_load_thresshold[2])
			{
				motor.over_load_cnt[3]++;
			}
			else if (cur_current >= motor.over_load_thresshold[1])
			{
				motor.over_load_cnt[2]++;
			}
			else if (cur_current >= motor.over_load_thresshold[0])
			{
				motor.over_load_cnt[1]++;
			}
			else
			{
				motor.over_load_cnt[0]++;
			}
		}
		else
		{
			for (int i = 0; i < over_load_groups; i++)
			{
				if (motor.over_load_cnt[i] > 0)
					motor.over_load_cnt[i]--;
			}
		}
		for (int i = 0; i < over_load_groups; i++)
		{
			if (motor.over_load_cnt[i] > motor.over_load_time[i] * 1000)
			{
				return 1;
			}
		}
		cut_tick = getTicks();
	}
	return ret;
}
static void motCheckFault()
{
	if (FAULT_CHECK(g_sM1Drive.sFaultIdPending, FAULT_I_DCBUS_OVER))
		alertPush(ALT_MOT_OVER_CURRENT, 0);
	else if (FAULT_CHECK(g_sM1Drive.sFaultIdPending, FAULT_U_DCBUS_UNDER))
		alertPush(ALT_MOT_UNDER_DCBUS, 0);
	else if (FAULT_CHECK(g_sM1Drive.sFaultIdPending, FAULT_U_DCBUS_OVER))
		alertPush(ALT_MOT_OVER_DCBUS, 0);
	else if (FAULT_CHECK(g_sM1Drive.sFaultIdPending, FAULT_LOAD_OVER))
		alertPush(ALT_MOT_OVER_LOAD, 0);
	else if (FAULT_CHECK(g_sM1Drive.sFaultIdPending, FAULT_SPEED_OVER))
		alertPush(ALT_MOT_OVER_SPEED, 0);
	else if (FAULT_CHECK(g_sM1Drive.sFaultIdPending, FAULT_ROTOR_BLOCKED))
		alertPush(ALT_MOT_ROTOR_BLOCKED, 0);
	else if (FAULT_CHECK(g_sM1Drive.sFaultIdPending, FAULT_OVER_LOAD))
		alertPush(ALT_MOT_OVER_LOAD, 0);

	if (checkMotLock())
	{
		enableMotor(0);
		alertPush(ALT_MOT_ROTOR_BLOCKED, 0);
	}
	if (checkMotOverload())
	{
		enableMotor(0);
		alertPush(ALT_MOT_OVER_LOAD, 0);
	}
	/*else if(g_sM1Drive.sFaultIdPending>0)
		alertPush(g_sM1Drive.sFaultIdPending);*/
}

static void deviceIdStart(bool stOrEnd)
{
	// uint8_t cmd[8];

	if (stOrEnd)
	{
		mot_run_state.run_mode = RUN_MODE_INIT;
		mot_run_state.lightSpd = 0; // always on
		mot_run_state.sub_mode = 0;
	}
	else
	{
		mot_run_state.run_mode = RUN_MODE_RUN;
		mot_run_state.lightSpd = 1000;
		mot_run_state.sub_mode = 0;
	}
	//	cmd[0] = CMD_ID_MODE;
	//	cmd[1] = mot_id;
	//	BOARD_CanSend(CAN_SEND_ID, cmd, 2);
}

static void deviceIdSM(uint32_t msg, Mot_Msg_Param *param)
{
	uint8_t cmd[8];
	static uint32_t period; // us
	static uint32_t tickstart = 0;
	static uint32_t randId = 0;
	Param_Data paraData;
	uint8_t uid[12] ;
	switch (mot_run_state.sub_mode)
	{
	case 0: // wait read rand deivce id command
		if (msg == MOT_MSG_SEND_RANDID)
			mot_run_state.sub_mode = 1;
		break;

	case 1: // send rand device id
	{
		uint32_t seed = (uint32_t)hpm_csr_get_core_cycle()
		^ (uint32_t)(hpm_csr_get_core_cycle() >> 32)
		^ (uint32_t)(getCurPos() * 1000.0f)
		^ (uint32_t)(getCurTorq(1) * 1e6f)
		^ (uint32_t)BOARD_GetUID(uid)
		^ ((uint32_t)getTicks() << 16)
		^ mot_id;
		srand(seed);
		randId = (uint32_t)rand() ^ ((uint32_t)rand() << 16);  // 或连调两次 rand	
		period = randId & 0xf;
		tickstart = getTicks();
		mot_run_state.sub_mode = 2;
	}
		break;
	case 2: // delay period;
		if (expireTicks(tickstart, period))
		{
			cmd[0] = CMD_ID_RESP;
			cmd[1] = mot_id;
			SET_DWORD(randId, cmd[4], cmd[5], cmd[6], cmd[7]);
			BOARD_CanSend(CAN_SEND_ID, cmd, 8);
			mot_run_state.sub_mode = 3;
		}
		break;
	case 3: // wait select
		if (msg == MOT_MSG_SELECT_DEV && randId == param->devSelect.rand)
		{
			if (param->devSelect.sel == 1)
			{
				mot_run_state.lightSpd = 100;
				//  mot_run_state.sub_mode = 4;
			}
			else
			{
				mot_run_state.lightSpd = 0;
			}

			cmd[0] = CMD_SEL_DEVICE;
			cmd[1] = mot_id;
			BOARD_CanSend(CAN_SEND_ID, cmd, 2);
		}
		else if (msg == MOT_MSG_SET_DEVID && randId == param->devIdSetting.rand)
		{
			paraData.u2_data = param->devIdSetting.motId;
			writeParameter(PARNO_MOT_ID, paraData);
			cmd[0] = CMD_WR_ID;
			cmd[1] = mot_id;
			BOARD_CanSend(CAN_SEND_ID, cmd, 2);
			mot_run_state.lightSpd = 1000;
			// mot_run_state.sub_mode = 0;
		}
		break;

		/*case 4: //wait device id setting
		  if (msg == MOT_MSG_SET_DEVID && randId == param->devIdSetting.rand)
		  {
			paraData.u2_data = param->devIdSetting.motId;
			writeParameter(PARNO_MOT_ID, paraData);
			cmd[0] = CMD_WR_ID;
			cmd[1] = mot_id;
			BOARD_CanSend(CAN_SEND_ID, cmd, 2);
			mot_run_state.lightSpd = 0;
			//mot_run_state.sub_mode = 0;
		  }
		  else if (msg == MOT_MSG_SELECT_DEV && randId == param->devSelect.rand)
		  {
			if(param->devSelect.sel == 1)
			{
			  mot_run_state.lightSpd = 100;
			  mot_run_state.sub_mode = 4;
			}
			else
			{
			  mot_run_state.lightSpd = 0;
			}

			cmd[0] = CMD_SEL_DEVICE;
			cmd[1] = mot_id;
			BOARD_CanSend(CAN_SEND_ID, cmd, 2);
		  }
		  break;*/
	}
}

void motAutoRunTestSM(void)
{
	static uint32_t tickstart = 0;
	static float spd = 1000;

	switch (mot_run_state.sub_mode)
	{
	case 0: // wait a memont
		tickstart = getTicks();
		mot_run_state.sub_mode = 1;
		break;
	case 1:
		if (expireTicks(tickstart, 10000))
		{
			mot_run_state.sub_mode = 2;
			spd = -1.0f * spd;
		}
		break;
	case 2:
		//    enableMotor(true);
		moveSpd(spd, -1.0f);
		mot_run_state.sub_mode = 0;
		break;
	default:
		break;
	}
}

static void syncAction(void)
{
	float destPos = mot_run_state.masterPos + mot_run_state.syncOfsset;

	mot_run_state.syncErr = destPos - getCurPos();
	movePos(destPos, -1, -1);
}

void syncActionSM(void)
{
	if (mot_run_state.run_mode == RUN_MODE_SYNC)
	{
		switch (mot_run_state.sub_mode)
		{
		case 0:
			if ((mot_run_state.syncDir && mot_run_state.masterPos >= mot_run_state.syncPos) || ((!mot_run_state.syncDir) && mot_run_state.masterPos <= mot_run_state.syncPos))
				mot_run_state.sub_mode = 1;
			break;

		case 1:
			syncAction();
			break;
		}
	}
}

static bool procMotRunMsg(uint32_t msg, Mot_Msg_Param *param)
{
	bool bProc = true;

	if (msg == MOT_MSG_ENABLE)
	{
		// g_sM1Drive.sPosCtrl.fltPositionCmd = g_sM1Drive.sPosCtrl.fltPositionFilt;
		// g_sM1Drive.eControl = kControlMode_PosFOC;
		enableMotor(param->motEnable);
	}
	else if (msg == MOT_MSG_MOVE_POS)
	{
		enableMotor(true);
		movePos(param->motMove.pos + getCurPos(), param->motMove.speed, param->motMove.torque);
	}
	else if (msg == MOT_MSG_MOVE_POS_TO)
	{
		enableMotor(true);
		movePos(param->motMove.pos, param->motMove.speed, param->motMove.torque);
	}
	else if (msg == MOT_MSG_MOVE_SPD)
	{
		enableMotor(true);
		moveSpd(param->motMove.speed, param->motMove.torque);
	}
	else if (msg == MOT_MSG_MOVE_TORQ)
	{
		enableMotor(true);
		moveTorq(param->motMove.torque);
	}
	else if (msg == MOT_MSG_STOP)
	{
		enableMotor(true);
		moveStop(false);
	}
	else if (msg == MOT_MSG_ZERO)
	{
		enableMotor(true);
		zeroStart(param->motZero.speed, param->motZero.torque);
	}
	else if (msg == MOT_MSG_FIND_SIDE)
	{
		moveSpd(param->motMove.speed, param->motMove.torque);
	}
	else if (msg == MOT_SYNC)
	{
		mot_run_state.syncPos = param->motSync.syncPos;
		mot_run_state.syncOfsset = (int32_t)param->motSync.offset * mot_run_state.ratioSyncOff;
		if ((int32_t)mot_run_state.deno * (int32_t)mot_run_state.nume * mot_run_state.encDir > 0)
			mot_run_state.syncDir = param->motSync.dir;
		else
			mot_run_state.syncDir = !param->motSync.dir;
	}
	else if (msg == MOT_SYNC_STOP)
	{
		movePos(param->motSync.syncPos, -1, -1);
	}
	else if (msg == MOT_MSG_SCALAR_CTRL)
	{
		moveScalarCtrl(param->motMove.speed, param->motMove.torque);
	}
	else
	{
		bProc = false;
	}
	return bProc;
}
void motCheckOverSpd(void)
{
	if (g_sM1Drive.eControl == kControlMode_CurrentFOC)
	{
		// if(fabs(g_sM1Drive.fltSpeedFromSensor/g_fltM1speedAngularScale) >( LIMIT_SPD *2)  )
		// if((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeed )) > motor.torque_over_speed && motor.torque_over_speed!=0)
		if (fabs(g_sM1Drive.fltSpeedFromSensor / g_fltM1speedAngularScale) > motor.torque_over_speed && motor.torque_over_speed != 0 && jacking_motor_get_main_stat() == 0)
		{
			uint8_t cmd[8];
			moveStop(false);
			// moveSpd(0,-1) ;
			// enableMotor(false);
			if (mot_run_state.torque_check_over_spd_send_flag)
			{
				cmd[0] = CMD_TORQ_OVER_SPD_MSG;
				cmd[1] = mot_id; // motor id
				cmd[2] = 0;		 //
				cmd[3] = 0;
				BOARD_CanSend(CAN_SEND_ID, cmd, 4);
				mot_run_state.torque_check_over_spd_send_flag = false;
			}
		}
	}
}
uint8_t motTorqueOverSpd(void)
{
	if (fabs(g_sM1Drive.fltSpeedFromSensor / g_fltM1speedAngularScale) > motor.torque_over_speed && motor.torque_over_speed != 0)
	{
		return 1;
	}
	else
		return 0;
}
void mot_run_circle(void)
{
	static uint8_t flag = 0;
	enableMotor(1);
	mot_run_state.target_pos_offset = 5 * 8;
	if (flag != 0)
	{
		movePos(posOut2In(posRatioIn2Out(0)), spdOut2In(MOT_MSG_MOVE_POS, mot_run_state.punching_speed), -1);
	}
	else
	{
		movePos(posOut2In(posRatioIn2Out(360.0f)), spdOut2In(MOT_MSG_MOVE_POS, mot_run_state.punching_speed), -1);
	}
	flag = ~flag;
	mot_run_state.punching_state = 1;
	mot_run_state.punching_cnt++;
}
void mot_run_punching(void)
{
	float cur_current = getCurTorq(1) * 1000;
	if (cur_current < 0)
		cur_current = -cur_current;
	switch (mot_run_state.punching_state)
	{
	case 0:

		break;
	case 1:

		mot_run_state.punching_max_current = 0;
		mot_run_state.punching_state = 2;
		break;
	case 2:

		if (mot_run_state.arr_pos_flag)
		{
			mot_run_state.punching_state = 0;
		}
		else
		{
			if (mot_run_state.punching_max_current < cur_current)
			{
				mot_run_state.punching_max_current = cur_current;
			}
		}
		break;
	}
}
void motRunSM(uint32_t msg, Mot_Msg_Param *param)
{
	uint8_t signal1, signal2, signal3, signal;
	static uint8_t old_signal1 = 0xff;
	switch (mot_run_state.run_mode)
	{
	case RUN_MODE_INIT:
		deviceIdSM(msg, param);
		break;

	case RUN_MODE_ZERO:
		if (mot_run_state.sensor_flag == 1) //
		{
			mot_run_state.zero_flag = 0;
			if (zero_sensor())
			{
				mot_run_state.run_mode = RUN_MODE_RUN;
			}
		}
		else
		{
			mot_run_state.zero_flag = 0;
			if (zeroTorqSm())
			{
				mot_run_state.run_mode = RUN_MODE_RUN;
			}
		}
		if (msg == MOT_MSG_STOP)
		{
			moveStop(false);
			mot_run_state.run_mode = RUN_MODE_RUN;
		}
		break;
	case RUN_MODE_SIDE:
		if (findSideSm())
		{
			mot_run_state.run_mode = RUN_MODE_RUN;
		}
		break;
	case RUN_MODE_FAULT:
		if (alertGet() == ALT_NONE)
			mot_run_state.run_mode = RUN_MODE_RUN;

		/* if(msg != MOT_MSG_NONE)
		   alertClear();*/

		break;

	case RUN_MODE_RUN:
		if (alertGet() != ALT_NONE)
		{
			mot_run_state.run_mode = RUN_MODE_FAULT;
		}
		else
		{
			procMotRunMsg(msg, param);

			// state tranfer
			if (msg == MOT_MSG_ZERO)
				mot_run_state.run_mode = RUN_MODE_ZERO;
			else if (msg == MOT_MSG_FIND_SIDE)
				mot_run_state.run_mode = RUN_MODE_SIDE;
			else if (msg == MOT_SYNC)
				mot_run_state.run_mode = RUN_MODE_SYNC;

			mot_run_state.sub_mode = 0;
		}

		break;
	case RUN_MODE_POS_TO_TORQUE:
		if (alertGet() != ALT_NONE)
		{
			mot_run_state.run_mode = RUN_MODE_FAULT;
		}
		else
		{

			procMotRunMsg(msg, param);

			if (mot_run_state.arr_pos_flag) // 已经到位
			{
				mot_run_state.run_mode = RUN_MODE_RUN;
				if (mot_run_state.mot_pos_to_torque_st.speed >= 0)
					param->motMove.torque = mot_run_state.mot_pos_to_torque_st.torque;
				else
					param->motMove.torque = -mot_run_state.mot_pos_to_torque_st.torque;
				procMotRunMsg(MOT_MSG_MOVE_TORQ, param);
			}
		}
		break;

	case RUN_MODE_SYNC:
		if (alertGet() != ALT_NONE)
		{
			mot_run_state.run_mode = RUN_MODE_FAULT;
		}
		else
		{
			procMotRunMsg(msg, param);

			// state tranfer
			if (msg == MOT_MSG_ZERO)
				mot_run_state.run_mode = RUN_MODE_ZERO;
			else if (msg == MOT_MSG_FIND_SIDE)
				mot_run_state.run_mode = RUN_MODE_SIDE;
			else if (msg == MOT_MSG_STOP || msg == MOT_MSG_MOVE_POS || msg == MOT_MSG_MOVE_POS_TO || msg == MOT_MSG_MOVE_SPD || msg == MOT_MSG_MOVE_TORQ || msg == MOT_MSG_ENABLE)
				mot_run_state.run_mode = RUN_MODE_RUN;
			else if (msg == MOT_SYNC)
				mot_run_state.sub_mode = 0;
			else if (msg == MOT_SYNC_STOP)
				mot_run_state.run_mode = RUN_MODE_RUN;
		}
		break;

	case RUN_MODE_TEST:
		motAutoRunTestSM();
		break;
	}
	// if(mot_run_state.motRun)
	// response message in all states
	/*if(InputSignalCheck() ==1)
	{
		  moveScalarCtrl(20,100) ;
	}
	else
	{
		  M1_SetAppSwitch(false);
	}*/
	if (msg == MOT_MSG_DEVID_MODE)
		deviceIdStart(param->devIdMode);
	// motCheckOverSpd() ;
	motCheckFault();
	reportTask();
	ledLight();
	mot_run_punching();
#if (BOARD_TYPE == BOARD_E145)
#ifdef JACKING_MOTOR_FUN
	signal1 = work_input_get_state(IN_INPUT2);
	signal2 = work_input_get_state(IN_INPUT1);
        signal3 = work_input_get_state(IN_INPUT3);
#else
	signal1 = work_input_get_state(IN_INPUT1);
	signal2 = work_input_get_state(IN_INPUT2);
        signal3 = work_input_get_state(IN_INPUT3);
#endif
#else
        #if(BOARD_TYPE == BOARD_E884)
        signal1 = (~work_input_get_state(IN_INPUT3) & 0x01);
	signal2 = (~work_input_get_state(IN_INPUT2) & 0x01);
        //signal3 = (~work_input_get_state(IN_INPUT3) & 0x01);   
        signal3 = 0;
        #else
	signal1 = work_input_get_state(IN_INPUT1);
	signal2 = work_input_get_state(IN_INPUT2);
        signal3 = work_input_get_state(IN_INPUT3);
        #endif
#endif

	signal = (signal1 << 5) | (signal2 << 6) | (signal3 << 7);
	work_io_set_fun((int32_t)mot_run_state.io_fun_set);
	if ((mot_run_state.signal != signal))
	{
		mot_run_state.signal = (signal1 << 5) | (signal2 << 6) | (signal3 << 7);
		if (mot_run_state.signal_report == 1)
			reportState(CAN_SEND_ID, mot_id,0);
	}
	if (mot_run_state.change_pid_flag == 1)
	{
		if (expireTicks(mot_run_state.change_pid_begin_tick, 4000))
		{
			mot_run_state.change_pid_flag = 0;
			g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = motor_run.spdPidP;
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = motor_run.posPip;
			g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = motor_run.spdPidI / 1000.0f;
		}
	}

	if (work_io_get_fun().io_fun_st.in1_fun == IO_FUN_PUNCHING) // 冲孔
	{
		if (signal1 == 1 && old_signal1 == 0) // 下降沿
		{

			mot_run_circle();
		}
	}
	else // 零位
	{
		if (mot_run_state.sensor_flag == 1) // 零位传感器内置
		{
                        #ifdef JACKING_MOTOR_FUN
			if (mot_run_state.stop_flag == 0)
			{
				if (signal1 == 0 && old_signal1 == 1) // 零位下降沿
				{
					setCurPos(mot_run_state.zero_set_pos);
					update_target_pos(posOut2In(mot_run_state.pos_target));
				}
			}
                        #endif
		}
	}
        if(g_sM1Drive.eControl == kControlMode_SpeedFOC)
        {
            if((posIn2Out(getCurPos()) > mot_run_state.speed_right_pos) && ((mot_run_state.speedReq >0 && mot_run_state.runDir ==0) ||(mot_run_state.speedReq <0 && mot_run_state.runDir ==1)))
            {
                mot_run_state.speedReq =0 ;
               // alertPush(ALT_MOT_ROTOR_BLOCKED, ALT_RIGHT_LIMIT);
                moveStop(false) ;
            }
            else if((posIn2Out(getCurPos()) <mot_run_state.speed_left_pos) && ((mot_run_state.speedReq <0 && mot_run_state.runDir ==0) ||(mot_run_state.speedReq >0 && mot_run_state.runDir ==1)))
            {
                mot_run_state.speedReq =0 ;
               // alertPush(ALT_MOT_ROTOR_BLOCKED, ALT_LEFT_LIMIT);
                moveStop(false) ;
            }
        }
	old_signal1 = signal1;
}
uint8_t motGetMovePosArrFlag(void)
{
	return mot_run_state.arr_pos_flag;
}

void motEncCalibrate(void)
{
#if (BOARD_TYPE == BOARD_E145) || (BOARD_TYPE == BOARD_E139)
	g_sMID.sMIDEnc.fltIdReqOpenLoop = M1_I_PH_NOM * 0.3f;
	//	g_sMID.sMIDEnc.ui8Pp = M1_MOTOR_PP;
	g_sMID.sMIDEnc.ui8Pp = motor.pp;
	g_sMID.eMeasurementType = kMID_EncSensors;
	g_sMID.sMIDAlignment.ui16AlignDuration = 1;
	g_sMID.sMIDAlignment.fltCurrentAlign = 0.0f;
	g_sMID.ui16EnableMeasurement = 1;
	g_sMID.sMIDEnc.clib_ext = 0;
	M1_SetAppSwitch(true);
#else
	motOpenSpd();
#endif
}
void motEncCalibrateExt(void) //
{
	g_sMID.sMIDEnc.fltIdReqOpenLoop = M1_I_PH_NOM * 0.5f;
	//g_sMID.sMIDEnc.ui8Pp = M1_MOTOR_PP;
        g_sMID.sMIDEnc.ui8Pp = motor.pp;
	g_sMID.eMeasurementType = kMID_EncSensors;
	g_sMID.sMIDAlignment.ui16AlignDuration = 1;
	g_sMID.sMIDAlignment.fltCurrentAlign = 0.0f;
	g_sMID.ui16EnableMeasurement = 1;
	g_sMID.sMIDEnc.clib_ext = 1;
	g_sMID.sMIDEnc.ui16Step = 0;
	M1_SetAppSwitch(true);
}
void motEncClibrateStop(void) //
{
	g_sMID.sMIDEnc.ui16Step = (get_clib_encode(0) >> 8) & 0x3f;
	M1_SetAppSwitch(FALSE);
	g_sMID.ui16EnableMeasurement = 0U;
}
void motEncClibrateData(uint16_t ext_encode, uint16_t in_encode) //
{
    int16_t cur;
        #if (BOARD_TYPE == BOARD_E914)
        if (motor.dir == 1)
        cur = ext_encode - get_clib_encode(0);
    else
        cur = (16383 - ext_encode) - get_clib_encode(0);
        #else
    if (motor.dir == 0)
        cur = ext_encode - get_clib_encode(0);
    else
        cur = (16383 - ext_encode) - get_clib_encode(0);
        #endif

    g_sMID.sMIDEnc.pui16AlignPos[0][g_sMID.sMIDEnc.ui16Step] = cur;
    g_sMID.sMIDEnc.ui16Step++;
    if (g_sMID.sMIDEnc.ui16Step >= MAX_ALIGN_CNT)
        g_sMID.sMIDEnc.ui16Step = 0;
}

void motEncClibrateSaveData(void) //
{
	M1_MCDRV_WRITE_ALIGN_DATA();
	InitEncConfigData();
}
void motOpenSpd(void)
{
	g_sM1Drive.sScalarCtrl.fltVHzGain = 2.5; // 2.1
	g_sM1Drive.sScalarCtrl.fltFreqCmd = 1;
	g_sM1Drive.eControl = kControlMode_Scalar;
	M1_SetAppSwitch(TRUE);
}
void update_target_pos(float fPos)
{
	M1_SetPosition(fPos);
	mot_run_state.posReq = fPos;
}
void motSetGearRatio(int16_t demo, int16_t nume) //
{
	mot_run_state.nume = nume;
	mot_run_state.deno = demo * ENCODER_DEMO;
}
void motChangePid(uint8_t type)
{
	if (type == 3)
	{
		g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = motor_run.spdPidP1;
		g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = motor_run.posPip1;
		g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = motor_run.spdPidI1 / 1000.0f;
	}
	else
	{
		mot_run_state.change_pid_begin_tick = getTicks();
		mot_run_state.change_pid_flag = 1;
	}
}
void mot_task_ms_loop(void) // 1ms任务
{
	float pos = 0;

#if (BOARD_TYPE == BOARD_E139)
    brake_seq_tick();   /* E139刹车 */
#endif

	if (mot_run_state.mot_move_time_param.move_flag == 1)           //分碎位置，减小位置误差
	{
		if (mot_run_state.mot_move_time_param.time == 0)
		{
			mot_run_state.mot_move_time_param.move_flag = 0;
			mot_run_state.mot_move_time_param.cur_pos = mot_run_state.mot_move_time_param.target_pos;
			pos = mot_run_state.mot_move_time_param.cur_pos;
			// reportPosMs(pos) ;
		}
		else
		{
			mot_run_state.mot_move_time_param.cur_pos = mot_run_state.mot_move_time_param.cur_pos + mot_run_state.mot_move_time_param.dis_pos;

			pos = mot_run_state.mot_move_time_param.cur_pos;
			// reportPosMs(pos) ;
			mot_run_state.mot_move_time_param.time--;
		}
		// movePos(pos, 3000, -1);

		movePos(pos, spdOut2In(MOT_MSG_MOVE_POS, 3000), -1);
	}
	if (mot_auto_sr_pos_st.sr_type == MOTOR_MAIN_TYPE && mot_auto_sr_pos_st.send_enable)    //定时can上报
	{
		static uint16_t cnt = 1;
		if (cnt >= mot_auto_sr_pos_st.send_period_ms)
		{
			reportState(mot_auto_sr_pos_st.sr_canid, 0xff,0);
			cnt = 1;
		}
		else
		{
			cnt++;
		}
	}
        if (mot_run_state.pos_over_zero_flag == 1)
	{
		servo_send_over_zero(mot_run_state.pos_zero_ptr);
		mot_run_state.pos_over_zero_flag = 0;
		mot_run_state.pos_over_zero_report_enable = 0;
	}
}
void servo_Auto_SR_Set_Period(uint16_t per) // 自动收发周期
{
	mot_auto_sr_pos_st.send_period_ms = per;
}
void servo_Auto_SR_Set_Type(uint8_t type) // 自动收发 类型
{
	mot_auto_sr_pos_st.sr_type = type;
}
void servo_Auto_SR_Enable(uint8_t en) // 自动收发 使能
{
	mot_auto_sr_pos_st.send_enable = en;
}
void servo_Auto_SR_Set_CanID(uint16_t canid) // 自动收发 CANID
{

	mot_auto_sr_pos_st.sr_canid = canid;
	if (mot_auto_sr_pos_st.sr_type == MOTOR_SLAVE_TYPE)
	{
		// BOARD_InitCan(mot_auto_sr_pos_st.sr_canid);
	}
}
void servo_Auto_SR_Set_Sensor_Port(uint8_t port) // 自动收发 传感器接入 PORT
{
	mot_auto_sr_pos_st.sensor_port = port;
}
void servo_Auto_SR_Set_offset(int32_t offset) // 自动收发 主从位置偏移
{
	mot_auto_sr_pos_st.offset_pos = offset;
}

uint16_t servo_Auto_SR_Get_CanID(void)
{
	return mot_auto_sr_pos_st.sr_canid;
}
void servo_Auto_SR_Set_Resv_Pos(int32_t pos)
{
	mot_auto_sr_pos_st.resv_pos = pos;
}
Motor_Sr_type servo_Auto_SR_Get_Type(void)
{
	return mot_auto_sr_pos_st.sr_type;
}
void servo_Auto_SR_Slave_ctrl(void) // 100us
{
	// uint8_t signal2 = 0;
	uint8_t sensor_edge;
	static uint8_t old_signal = 0;
	uint8_t signal = 0;
	if (mot_auto_sr_pos_st.sensor_port == 1) // 使用第二路传感器
	{

		signal = work_input_get_state(IN_INPUT2);
		sensor_edge = work_input_get_edge(IN_INPUT2);
	}
	else
	{
		signal = work_input_get_state(IN_INPUT1);
		sensor_edge = work_input_get_edge(IN_INPUT1);
	}
	if (old_signal == 0 && signal)
	{
		mot_run_state.pos_zero_ptr = mot_run_state.pos_buf_ptr;
		if (mot_run_state.pos_over_zero_report_enable)
		{
			mot_run_state.pos_over_zero_flag = 1;
		}
	}
	old_signal = signal;
	/* if (sensor_edge == EDGE_RISING)
	 {

	 }*/
	mot_run_state.pos_buf_ptr = (mot_run_state.pos_buf_temp_ptr) * mot_run_state.pos_buf_clib_scale;
	mot_run_state.pos_buf_time++;
	mot_run_state.pos_buf_temp_ptr++;
	if (mot_run_state.pos_buf_ptr >= MAX_POS_BUF)
	{
		mot_run_state.pos_buf_ptr = 0;
		mot_run_state.pos_buf_temp_ptr = 0;
	}
	if (MOTOR_SLAVE_TYPE == mot_auto_sr_pos_st.sr_type)
	{
		if (sensor_edge == EDGE_RISING)
		{
			setCurPos(mot_auto_sr_pos_st.offset_pos + mot_auto_sr_pos_st.resv_pos);
		}
	}
}
void motUpdateSpeedPidI(float fltIntegral)
{
	g_sM1Drive.sSpeed.sSpeedPiParams.fltIAccK_1 = fltIntegral;
}
void Servo_PID_Change(u8 cs)
{

	switch (cs)
	{
	case PID_SYNC:
		g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = motor_run.spdPidP1;
		g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = motor_run.spdPidI1 / 1000.0f;
		g_sM1Drive.sSpeed.sSpeedPiParams.fltUpperLim = (float)(motor_run.spdILmt * motor_run.over_load_scale1);
		g_sM1Drive.sSpeed.sSpeedPiParams.fltLowerLim = -(float)(motor_run.spdILmt * motor_run.over_load_scale1);
		g_sM1Drive.fltInom = motor.Iph_nom * motor_run.over_load_scale1;
		break;
	case PID_NOMAL:
		g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = motor_run.spdPidP;
		g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = motor_run.spdPidI / 1000.0f;
		g_sM1Drive.sSpeed.sSpeedPiParams.fltUpperLim = (float)(motor_run.spdILmt * motor_run.over_load_scale);
		g_sM1Drive.sSpeed.sSpeedPiParams.fltLowerLim = -(float)(motor_run.spdILmt * motor_run.over_load_scale);
		g_sM1Drive.fltInom = motor.Iph_nom * motor_run.over_load_scale;
		break;
	case PID_STOP:

		break;
	}
}
void servo_sync_clib_time(uint32_t time) // 校准同步时间
{
	if (time == 0)
	{
		mot_run_state.pos_buf_time = 0;
	}
	else
	{
		Param_Data paraData;
		mot_run_state.pos_buf_clib_scale = ((float)(time)) / mot_run_state.pos_buf_time;
		paraData.f4_data = mot_run_state.pos_buf_clib_scale;
		writeParameter( PARNO_POS_BUF_CLIB_SCALE, paraData);
	}
}
uint32_t servo_sync_get_clib_time(void)
{
	return mot_run_state.pos_buf_time;
}
void mot_set_ext_pos(int32_t pos)
{
        static int32_t old_pos =0xffffff;
        static int16_t circle_cnt =0 ;
        pos = (pos >>1) ; //转化成16位编码器
        if(old_pos == 0xffffff)
        {
            
            old_pos  = pos ;
            
        }
        else
        {
            if(pos - old_pos > 32768) circle_cnt --;
            else if(pos - old_pos < -32768) circle_cnt ++;
            old_pos = pos ;
        }
        mot_run_state.ext_pos = pos + circle_cnt *65536; 
	
}
int32_t mot_get_ext_pos(void)
{
	return mot_run_state.ext_pos;
}
void servo_sync_buf_zero(void)
{
	mot_run_state.pos_buf_ptr = 0;
	mot_run_state.pos_over_zero_report_enable = 1;
	mot_run_state.pos_buf_temp_ptr = 0;
}
uint16_t servo_get_sync_pos_ptr(void)
{
	return mot_run_state.pos_buf_ptr;
}
void servo_ext_pos_ctrl_set(uint16_t speed, int32_t pos) // 外部控制设置
{
        if(mot_run_state.ext_ctrl_pos_enable==1)
        {
            mot_ext_pos_ctrl_st.speed_limit = MLIB_Abs_FLT(spdOut2In(MOT_MSG_MOVE_POS, speed));
            mot_ext_pos_ctrl_st.torque_limit = MLIB_Abs_FLT(torqOut2In(1000));
          //  mot_ext_pos_ctrl_st.target_pos = mot_ext_pos_ctrl_st.cur_pos + pos;
          //  mot_ext_pos_ctrl_st.test_left_pos = mot_ext_pos_ctrl_st.cur_pos;
          //  mot_ext_pos_ctrl_st.test_right_pos = mot_ext_pos_ctrl_st.cur_pos + pos;
            mot_ext_pos_ctrl_st.target_pos = pos/10.0f * mot_ext_pos_ctrl_st.ext_to_in_scale ;//10mm的值 
            mot_ext_pos_ctrl_st.test_left_pos = 0;
            mot_ext_pos_ctrl_st.test_right_pos =  pos/10.0f * mot_ext_pos_ctrl_st.ext_to_in_scale ;
            mot_ext_pos_ctrl_st.ctrl_flag = 1;
           /* if (speed == 0)
            {
		mot_ext_pos_ctrl_st.test_flag = 0;
            }
            else
            {
		mot_ext_pos_ctrl_st.test_flag = 1;
            }*/
        }
	
}
float servo_ext_to_in_pos(float pos)
{
	return pos * mot_ext_pos_ctrl_st.ext_to_in_scale;
}
void servo_ext_open_read_encode(void)//打开外部编码器读取
{
	uint8_t data[8] = {0x05, 0x01, 0x05, 0xf4, 0x01, 0x00, 0x00, 0x00};
        uint8_t data1[8] ={0x04, 0x01, 0x04, 0xaa, 0x00, 0x00, 0x00, 0x00};
	BOARD_Can1Send(0x1, data, 5);
        hal_delayms(5) ;
        BOARD_Can1Send(0x1, data1, 4);
}
void servo_ext_update_ext_to_in_scale(void)
{
        mot_ext_pos_ctrl_st.ext_to_in_scale = posRatioOut2In(mot_ext_pos_ctrl_st.motor_encoder_10mm)/100.0f ; // 0.1mm多少个电机角度
}
#if(BOARD_TYPE == BOARD_E884)
void servo_ext_pos_ctrl_init(void)
{
    
        int16_t nume=1;
        int16_t deno=1;
        float f_temp =0.0f ;
	mot_ext_pos_ctrl_st.ctrl_period = 5; // 500us
        mot_ext_pos_ctrl_st.encoder_circle_value = 0xffff ;
	mot_ext_pos_ctrl_st.speed_limit = 50;
	mot_ext_pos_ctrl_st.ctrl_flag = 0;
	mot_ext_pos_ctrl_st.test_flag = 0;

        //mot_ext_pos_ctrl_st.ext_to_in_scale = posRatioOut2In(mot_ext_pos_ctrl_st.motor_encoder_10mm)/100.0f ; // 0.1mm多少个电机角度

        f_temp = -(mot_ext_pos_ctrl_st.motor_encoder_10mm * 360.0f * mot_run_state.encDir) ;
        mot_ext_pos_ctrl_st.ext_to_in_scale =  (f_temp / (M1_POSPE_ENC_PULSES * 4))/100.0f;
	servo_ext_open_read_encode();
	PID_HandleInit(&PID_ext_pos_ctrl);
}
#endif
void servo_ext_mm_to_motor_pos(float mm_pos)
{
}
void servo_set_ext_ctrl_enable(uint8_t en,uint8_t level,uint8_t canid)
{
        if(mot_run_state.ext_ctrl_pos_enable==0)
        {
            mot_run_state.ext_ctrl_pos_enable = en ;
        }
        else if(mot_run_state.ext_ctrl_pos_enable ==1) //外部控制
        {
            if(en ==0 && canid==1)
            {
                mot_run_state.ext_ctrl_pos_enable = en ;
            }
        }
        else if(mot_run_state.ext_ctrl_pos_enable ==2) //自动排刀
        {
            if(en ==0 && canid==0)
            {
                mot_run_state.ext_ctrl_pos_enable = en ;
            }
        }
        mot_run_state.ext_ctrl_pos_level = level ;
}
uint8_t servo_get_ctrl_enable(uint8_t canid)
{
        if(mot_run_state.ext_ctrl_pos_enable==0)
        {
            return 1 ;
        }
        else if(mot_run_state.ext_ctrl_pos_enable==2 && canid==0)
        {
            return 1 ;
        }
        return 0 ;
}
#if 0
void servo_ext_pos_ctrl_loop(void) // 外部控制loop
{
	int32_t error = 0;
	int32_t set_pos = 0;
	static uint8_t ctrl_cnt = 0;
	static float cur_pos = 0;
	static uint8_t old_ctrl_flag = 0;
	if (old_ctrl_flag == 0 && mot_ext_pos_ctrl_st.ctrl_flag == 1)
	{
		cur_pos = getCurPos();
	}
	if (ctrl_cnt >= mot_ext_pos_ctrl_st.ctrl_period)
	{
		mot_ext_pos_ctrl_st.cur_pos = mot_run_state.ext_pos;
		if (mot_ext_pos_ctrl_st.ctrl_flag == 1)
		{
			error = mot_ext_pos_ctrl_st.target_pos - mot_ext_pos_ctrl_st.cur_pos; // 转换成角度
			set_pos = PID_Controller(&PID_ext_pos_ctrl, error);
			mot_ext_pos_ctrl_st.set_pos = servo_ext_to_in_pos(set_pos);
			if (fabs(error) < 50)
			{
				mot_ext_pos_ctrl_st.ctrl_flag = 0;
				PID_CLEAR(&PID_ext_pos_ctrl);
				movePos(getCurPos(), mot_ext_pos_ctrl_st.speed_limit, mot_ext_pos_ctrl_st.torque_limit);
			}
			else
			{
				movePos(set_pos + cur_pos, mot_ext_pos_ctrl_st.speed_limit, mot_ext_pos_ctrl_st.torque_limit);
			}
		}
	}
	else
	{
		ctrl_cnt++;
	}
	old_ctrl_flag = mot_ext_pos_ctrl_st.ctrl_flag;
}
#else
void servo_ext_pos_ctrl_loop(void) // 外部控制loop 100us
{
	int32_t error = 0;
	int32_t set_pos = 0;
	static uint8_t ctrl_cnt = 0;
	static float cur_pos = 0;
	static uint32_t begin_ext_pos = 0;
	static uint8_t old_ctrl_flag = 0;
	static uint16_t test_cnt = 0;
	if (ctrl_cnt >= mot_ext_pos_ctrl_st.ctrl_period) // 2K 500US
	{
		mot_ext_pos_ctrl_st.cur_pos = mot_run_state.ext_pos;
		switch (mot_ext_pos_ctrl_st.state)
		{
		case 0:
			if (mot_ext_pos_ctrl_st.ctrl_flag == 1) // 开始控制
			{
				begin_ext_pos = mot_run_state.ext_pos; // 记录当前外部位置
				test_cnt = 0;
                                #if 0
				if (mot_ext_pos_ctrl_st.target_pos > mot_ext_pos_ctrl_st.cur_pos)
				{
					enableMotor(1);
					movePos(getCurPos() + servo_ext_to_in_pos(mot_ext_pos_ctrl_st.target_pos - mot_ext_pos_ctrl_st.cur_pos) + MAX_GRAP_POS, mot_ext_pos_ctrl_st.speed_limit, mot_ext_pos_ctrl_st.torque_limit); // 最大走间隙+控制位置
					mot_ext_pos_ctrl_st.state = 1;
				}
				else if (mot_ext_pos_ctrl_st.target_pos < mot_ext_pos_ctrl_st.cur_pos)
				{
					movePos(getCurPos() + servo_ext_to_in_pos(mot_ext_pos_ctrl_st.target_pos - mot_ext_pos_ctrl_st.cur_pos) - MAX_GRAP_POS, mot_ext_pos_ctrl_st.speed_limit, mot_ext_pos_ctrl_st.torque_limit); // 最大走间隙+控制位置
					mot_ext_pos_ctrl_st.state = 1;
				}
                                #else
                                if (mot_ext_pos_ctrl_st.target_pos >0)
				{
					enableMotor(1);
                                        mot_run_state.target_pos_offset = 10 * 8;
                                        mot_run_state.target_pos_ack_flag = 1 ;
					movePos(getCurPos() + mot_ext_pos_ctrl_st.target_pos  + MAX_GRAP_POS, mot_ext_pos_ctrl_st.speed_limit, mot_ext_pos_ctrl_st.torque_limit); // 最大走间隙+控制位置
					mot_ext_pos_ctrl_st.state = 1;
				}
				else if(mot_ext_pos_ctrl_st.target_pos <0)
				{
                                        enableMotor(1);
                                        mot_run_state.target_pos_offset = 10 * 8;
                                        mot_run_state.target_pos_ack_flag = 1 ;
					movePos(getCurPos() + mot_ext_pos_ctrl_st.target_pos - MAX_GRAP_POS, mot_ext_pos_ctrl_st.speed_limit, mot_ext_pos_ctrl_st.torque_limit); // 最大走间隙+控制位置
					mot_ext_pos_ctrl_st.state = 1;
				}
                                #endif
			}
			else
			{
				/*if (mot_ext_pos_ctrl_st.test_flag == 1)
				{
					if (test_cnt > 1000) // 200ms
					{
						if (mot_ext_pos_ctrl_st.target_pos == mot_ext_pos_ctrl_st.test_left_pos)
						{
							mot_ext_pos_ctrl_st.target_pos = mot_ext_pos_ctrl_st.test_right_pos;
						}
						else
						{
							mot_ext_pos_ctrl_st.target_pos = mot_ext_pos_ctrl_st.test_left_pos;
						}
						mot_ext_pos_ctrl_st.ctrl_flag = 1;
					}
					else
						test_cnt++;
				}*/
			}
			break;
		case 1:
			error = begin_ext_pos - mot_ext_pos_ctrl_st.cur_pos; // 转换成角度
			if (fabs(error) > 20)								 // 已经开始消除间隙
			{
				enableMotor(1);
                                mot_ext_pos_ctrl_st.move_target_cur_pos = getCurPos() ;
                                mot_run_state.target_pos_offset = 10 * 8;
                                mot_run_state.target_pos_ack_flag = 1 ;
                                mot_ext_pos_ctrl_st.move_target_pos = mot_ext_pos_ctrl_st.move_target_cur_pos + mot_ext_pos_ctrl_st.target_pos ;
				movePos(mot_ext_pos_ctrl_st.move_target_pos, mot_ext_pos_ctrl_st.speed_limit, mot_ext_pos_ctrl_st.torque_limit); // 最大走间隙+控制位置
				mot_ext_pos_ctrl_st.ctrl_flag = 0;
				mot_ext_pos_ctrl_st.state = 0;
			}
			break;

		default:
			break;
		}
	}
	else
	{
		ctrl_cnt++;
	}
}
#endif