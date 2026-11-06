
#include "jacking_motor.h"
#include "motor.h"
#include "alert.h"
#include "work_io.h"
#include "math.h"
#include "tick.h"
#include "servo_can.h"
#ifdef JACKING_MOTOR_FUN // 升降功能打开
JACKING_MOTOR_RESET_ST jm_reset_st = {
	.reset_down_speed = 30,		 // 复位向下速度
	.reset_limit_pos = 30000,	 // 复位最大行程
	.reset_zero_pos = 20000,	 // 零位初始值
	.reset_signal_max_pos = 500, // 传感器最大容差值
};

JACKING_MOTOR_INIT_ST jm_init_st = {
	.exit_flag = 0,
	.pause_flag = 0,
	.set_para_finish_flag = 0,
	.debug_speed = 20,
	.zero_to_safe_door_speed = 20,
	.safe_door_to_cast_off_speed = 30,
	.cast_off_to_safe_speed = 30,
	.safe_to_dossal_speed = 15,
	.dossal_st.dossal_low_speed = 15,
	.dossal_st.dossal_high_speed = 40,
	.signal_enable_speed = 20,
	.safe_signal_st.safe_max_offset_pos = 300,
	.safe_signal_st.safe_signal_check_pos = 300,
	.dossal_st.dossal_check_signal_limit_pos = 300,
	.dossal_pos = 23680,
	.dossal_st.dossal_time_out_max = 20000,
	.safe_signal_pos = 23160,
	.dossal_st.dossal_safe_dec_pos = 22500,
	.cast_off_pos = 22000,
	.safe_door_pos = 21000,
	.safe_signal_st.safe_torque = 2000,
	.safe_signal_st.safe_speed = 20,
	.pdc_st.pdc_torque = -100,
	.pdc_st.pdc_big_torque = -500,
	.pdc_st.pdc_torque_time = 10,
	.pdc_st.pdc_big_torque_time = 90,
	.pdc_st.pdc_change_torque_enable = 0,
	.move_speed_torque = 5000,
	.move_target_pos = 21000,
	.pdc_st.pdc_pull_max_pos = 100,			   // 拉布时最大行程位置限制
	.safe_signal_st.safe_move_offset_pos = 15, // 安全位多移动的距离
	.pre_safe_signal = 0xff,
};
int32_t jacking_motor_abs(int32_t value) // 绝对值
{
	int32_t ret;
	if (value >= 0)
		ret = value;
	else
		ret = -value;
	return ret;
}
uint8_t jacking_motor_get_alert(void)
{
	return alertGet();
}
void jacking_motor_change_pid(uint8_t type) // 切电机PID刚性
{
        #if (BOARD_TYPE == BOARD_E139)
	motChangePid(type);
        #endif
}
uint8_t jacking_motor_check_zero_signal(void) // 零位传感器检测 1有效 0无效
{
        #if (BOARD_TYPE == BOARD_E139)
        return work_input_get_state(IN_INPUT1);
        #else
        return work_input_get_state(IN_INPUT2);
        #endif
}
uint8_t jacking_motor_check_safe_signal(void) // 安全位传感器检测 1有效 0 无效
{
        #if (BOARD_TYPE == BOARD_E139)
	return work_input_get_state(IN_INPUT2);
        #else
        return work_input_get_state(IN_INPUT1);
        #endif
}
void jacking_motor_set_cur_pos(int32_t pos) // 设置升降电机当前位置
{
	setCurPos(pos);
}
int16_t wire_feed_motor_get_current(void) // 获取升降电机当前电流
{
	return torqIn2Out(fabs(getCurTorq(1)));
}

void jacking_motor_init(void) // 升降电机初始化
{
	jm_init_st.pCkZeroSignal = jacking_motor_check_zero_signal;
	jm_init_st.pCkSafeSignal = jacking_motor_check_safe_signal;
	jm_init_st.report_dis_pos = 0;

	// jacking_motor_set_cur_pos(jm_reset_st.reset_zero_pos) ;
	// motSetGearRatio(1,60) ;
}
void jacking_motor_clear_alert(void) // 升降电机初始化
{
	alertClear();
}
void jackling_motor_enable(uint8_t enb) // 升降电机使能 1有效 0无效
{
	enableMotor(enb);
}
void jacking_motor_move_speed(int16_t speed, uint16_t torque) // 升降电机速度模式 speed = rpm  torque = ma
{
	float fTorque = torque;
	jacking_motor_clear_alert();
	jackling_motor_enable(1);

	fTorque = fTorque / 1000.0f;
	moveSpd(spdOut2In(MOT_MSG_MOVE_SPD, speed * 10.0f), fTorque);
}
void jacking_motor_move_pos(int32_t pos, uint16_t speed, int16_t torque) // 升降电机位置模式 pos =encoder  speed = rpm    0623
{
	float fTorque = torque;
	jacking_motor_clear_alert();
	jackling_motor_enable(1);
	mot_run_state.target_pos_offset = 0.5; //
										   // mot_run_state.posReq = posOut2In(pos) ;
	if (fTorque == -1)
	{
		movePos(posOut2In(pos), spdOut2In(MOT_MSG_MOVE_POS, speed ), -1);
	}
	else
	{
		fTorque = fTorque / 1000.0f;
		movePos(posOut2In(pos), spdOut2In(MOT_MSG_MOVE_POS, speed ), fTorque);
	}
}
void jacking_motor_stop(uint8_t stop_or_disable) //  升降电机停止 1 放使能 0 位置模式锁定
{
	jacking_motor_clear_alert();
	jackling_motor_enable(1);
	moveStop(stop_or_disable);
}
void jacking_motor_half_current_lock(void) // 升降电机半流锁定
{
	float torque_limit = 0.0f;
	jackling_motor_enable(1);
	// board_delay_ms(3) ;
	/* if(jm_init_st.safe_to_dossal_torque==-1)
	 {
		 torque_limit =  g_sM1Drive.fltInom / 2 ;
	 }
	 else
	 {
		 if((g_sM1Drive.fltInom / 2) > jm_init_st.safe_to_dossal_torque )
		 {
			 torque_limit = jm_init_st.safe_to_dossal_torque ;
		 }
		 else
		 {
			 torque_limit = g_sM1Drive.fltInom / 2 ;
		 }
	 }*/
	torque_limit = g_sM1Drive.fltInom / 3;
	motHalfCurrentLock();
}
void jacking_motor_move_torque(int16_t torque) // 升降电机力矩模式 torque = ma
{
	float fTorque = torque;
	jacking_motor_clear_alert();

	jackling_motor_enable(1);
	fTorque = fTorque / 1000.0f;
	moveTorq(fTorque);
}
int32_t jacking_motor_get_cur_pos(void) // 获取升降电机当前位置
{
	return posIn2Out(getCurPos());
}
void jacking_motor_report_status(void) // 升降电机上报状态
{
	uint32_t status = 0;
	status = jm_init_st.ctrl_main_status | (jm_init_st.ctrl_sub_status << 8);
	reportJkMotorStatus(status);
}
void jacking_motor_report_alert(uint8_t alert_id, uint8_t rank) // 升降电机报警
{
	alertPush(alert_id, rank);
	jm_init_st.ctrl_main_status = JM_MAIN_STATUS_ERROR;
	jacking_motor_report_status();
}

void jacking_motor_motor_move_is_stop_resh() // 重新刷新停止时间
{
	moveIsStop(1800, 5, 1); // 2000
}
uint8_t jacking_motor_move_is_stop() // 升降电机是否停止 1停止 0未停止
{
	if (moveIsStop1(1000, 3,0) == 1)
	{
		return 1;
	}
	else
		return 0;
	/*if(moveIsStop(1800, 5,0)==1)//1000
		return 1 ;
	else
		return 0 ;*/
}
uint8_t jacking_motor_torque_over_speed(void) // 升降电机 力矩模式下是否过速 1过速 0未过速
{
	return motTorqueOverSpd();
}
uint8_t jacking_motor_ck_rst_overtravel(void) // 升降电机 检测复位最大容差 1过容差 0未过
{
	if (jacking_motor_abs(jacking_motor_get_cur_pos() - jm_reset_st.reset_begin_pos) > jm_reset_st.reset_limit_pos)
	{
		return 1;
	}
	else
		return 0;
}
uint8_t jacking_motor_ck_rst_siganl_enable_overtravel(void) // 升降电机 检测复位传感器最大容差 1过容差 0未过
{
	if (jacking_motor_abs(jacking_motor_get_cur_pos() - jm_reset_st.reset_signal_enable_begin_pos) > jm_reset_st.reset_signal_max_pos)
	{
		return 1;
	}
	else
		return 0;
}

uint8_t jacking_motor_check_safe_signal_offset(void) // 升降电机 检测安全位传感器最大容差 1过 0未过
{
	if (jacking_motor_abs(jacking_motor_get_cur_pos() - jm_init_st.dossal_st.dossal_check_signal_begin_pos) > jm_init_st.safe_signal_st.safe_signal_check_pos)
	{
		return 1;
	}
	else
		return 0;
}
uint8_t jacking_motor_check_safe_pos(void) // 升降电机 检测安全位最大容差 1过 0未过
{
	if (jacking_motor_abs(jacking_motor_get_cur_pos() - jm_init_st.safe_signal_pos) > jm_init_st.safe_signal_st.safe_max_offset_pos)
	{
		return 1;
	}
	else
		return 0;
}
int16_t g_pull_down_torque = 0;
uint8_t jacking_motor_check_torque_pull_down(int16_t pull_down_torque) // 拉布力是否小于当前的力 0表示小 1表示大 可以正常拉
{
	int16_t scale_torque = getCurTorq(1) * 1000; // 当前电流
	g_pull_down_torque = pull_down_torque;
	if (scale_torque < 0 && pull_down_torque < 0)
	{
		if (jacking_motor_abs(scale_torque) < jacking_motor_abs(pull_down_torque))
		{
			return 1;
		}
		else
			return 0;
	}
	else
		return 1;
}
uint8_t jacking_motor_move_pos_arr_flag(void) // 升降电机 移动是否到位 1到位 0未到位
{
	return motGetMovePosArrFlag();
}

void jacking_motor_report_current_from_pos(uint8_t dis_pos) // 根据位置上报电流
{
	jm_init_st.report_dis_pos = dis_pos;
	if (dis_pos == 0)
	{
		reportMotorCurrent(jacking_motor_get_cur_pos(), wire_feed_motor_get_current());
	}
	else
	{

		jm_init_st.report_cur_ptr = (jacking_motor_get_cur_pos() - jm_reset_st.reset_zero_pos) / dis_pos;
	}
}
void jacking_motor_run_report_current(void) // 实时上报位置和电流信息
{
	if (jm_init_st.report_dis_pos > 0)
	{
		uint16_t cur_ptr = (jacking_motor_get_cur_pos() - jm_reset_st.reset_zero_pos) / jm_init_st.report_dis_pos;
		if (cur_ptr != jm_init_st.report_cur_ptr)
		{
			reportMotorCurrent(cur_ptr * jm_init_st.report_dis_pos + jm_reset_st.reset_zero_pos, wire_feed_motor_get_current());
			jm_init_st.report_cur_ptr = cur_ptr;
		}
	}
}

uint16_t jacking_motor_get_speed_from_pos(void) // 根据位置不同获取不同速度 RPM
{
	int32_t cur_pos = jacking_motor_get_cur_pos();
	uint16_t speed = jm_init_st.debug_speed;
	if (jm_init_st.set_para_finish_flag) // 设置参数完成标记
	{
		if (cur_pos < jm_init_st.zero_pos) //
		{
			speed = jm_init_st.zero_to_safe_door_speed;
		}
		else if (cur_pos >= jm_init_st.zero_pos && cur_pos < jm_init_st.safe_door_pos) //
		{
			speed = jm_init_st.zero_to_safe_door_speed;
		}
		else if (cur_pos >= jm_init_st.safe_door_pos && cur_pos < jm_init_st.cast_off_pos) //
		{
			speed = jm_init_st.safe_door_to_cast_off_speed;
		}
		else if (cur_pos >= jm_init_st.cast_off_pos && cur_pos < (jm_init_st.safe_signal_pos - jm_init_st.safe_signal_st.safe_move_offset_pos - 100)) // 为了有减速效果
		{
			speed = jm_init_st.cast_off_to_safe_speed;
		}
		else if (cur_pos >= (jm_init_st.safe_signal_pos - jm_init_st.safe_signal_st.safe_move_offset_pos - 100) && cur_pos < jm_init_st.dossal_pos) //
		{
			speed = jm_init_st.safe_to_dossal_speed;
		}
		else //
		{
			speed = jm_init_st.safe_to_dossal_speed;
		}
	}
	return speed;
}
void jacking_motor_update_speed(uint16_t spd) // 更新速度
{
	motor_update_speed(spdOut2In(MOT_MSG_MOVE_POS, spd));
}
void jacking_motor_set_para(PARA_SET_TYPE type, int32_t pos) // 设置参数
{

	jm_init_st.set_para_type = type;
	switch (jm_init_st.set_para_type)
	{
	case PARA_SET_SAFE_DOOR:
		jm_init_st.safe_door_pos = pos;
		break;
	case PARA_SET_CAST_OFF:
		jm_init_st.cast_off_pos = pos;
		break;
	case PARA_SET_DOSSAL_POS:
		if (jm_init_st.pCkSafeSignal()) // 检测安全位信号
		{
			jm_init_st.dossal_pos = pos;
		}
		else
		{
			jacking_motor_report_alert(JM_MSG_NO_FIND_SAFE_SIGNAL, 0);
		}
		break;
	}
}

uint8_t jacking_motor_reset(void) // 复位流程
{
	uint8_t ret = 0;
	if (jm_init_st.exit_flag || jacking_motor_get_alert() != 0) // 退出标记
	{
		jm_init_st.exit_flag = 0;
		jm_init_st.pause_flag = 0;
		jm_reset_st.reset_state = JM_RESET_EXIT;
		jacking_motor_stop(0);
	}
	switch (jm_reset_st.reset_state)
	{
	case JM_RESET_INIT:					// 复位初始化
		if (jm_init_st.pCkZeroSignal()) // 检测零位传感器
		{
			jm_reset_st.reset_state = JM_RESET_TRANS_MOVE_UP;
			jm_reset_st.reset_signal_enable_begin_pos = jacking_motor_get_cur_pos();
		}
		else
		{
			jm_reset_st.reset_state = JM_RESET_TRANS_MOVE_DOWN;
			jm_reset_st.reset_begin_pos = jacking_motor_get_cur_pos();
		}
		break;
	case JM_RESET_TRANS_MOVE_UP: // 复位向上动作
		jacking_motor_move_speed(jm_init_st.signal_enable_speed, jm_init_st.move_speed_torque);
		// jacking_motor_motor_move_is_stop_resh();
		jm_reset_st.reset_state = JM_RESET_MOVE_UP;
		break;
	case JM_RESET_TRANS_MOVE_DOWN: // 复位向下中
		jacking_motor_move_speed(-jm_reset_st.reset_down_speed, jm_init_st.move_speed_torque);
		// jacking_motor_motor_move_is_stop_resh();
		jm_reset_st.reset_state = JM_RESET_MOVE_DOWN;
		break;
	case JM_RESET_MOVE_UP:					 // 复位向上
		if (jm_init_st.pCkZeroSignal() == 0) // 检测零位无效
		{
			jacking_motor_set_cur_pos(jm_reset_st.reset_zero_pos);
			jacking_motor_stop(0);
			jm_init_st.set_para_finish_flag = 1; //
			jm_reset_st.reset_state = JM_RESET_EXIT;
			jm_init_st.ctrl_main_status = JM_MAIN_STATUS_FINISH;
			jacking_motor_report_status();
			// jacking_motor_report_main_ctrl(JM_MSG_RESET_FINISH) ;//
		}
		else // 检测零位有效
		{
			if (jm_init_st.pause_flag) // 暂停标记
			{
				jm_reset_st.reset_pause_state = JM_RESET_TRANS_MOVE_UP;
				jm_reset_st.reset_state = JM_RESET_PAUSE;
			}
			if (jacking_motor_ck_rst_siganl_enable_overtravel()) // 检测复位是否超行程
			{
				jacking_motor_stop(0);
				jm_reset_st.reset_state = JM_RESET_EXIT;
				jacking_motor_report_alert(JM_MSG_SIGANL_ERROR, 0); //
			}
		}
		break;
	case JM_RESET_TRANS_MOVE_UP_FROM_STOP: // 撞到底部停止以后往上运行
		jacking_motor_move_speed(jm_init_st.signal_enable_speed, jm_init_st.move_speed_torque);
		// jacking_motor_motor_move_is_stop_resh();
		jm_reset_st.reset_state = JM_RESET_MOVE_UP_FROM_STOP;

		break;
	case JM_RESET_MOVE_UP_FROM_STOP:
		if (jm_init_st.pCkZeroSignal() == 1) // 检测零位有效
		{
			jm_reset_st.reset_state = JM_RESET_TRANS_MOVE_UP;
			jm_reset_st.reset_signal_enable_begin_pos = jacking_motor_get_cur_pos();
		}
		else
		{
			if (jacking_motor_ck_rst_siganl_enable_overtravel()) // 检测传感器信号是否超容差
			{
				jacking_motor_stop(0);
				jm_reset_st.reset_state = JM_RESET_EXIT;
				jacking_motor_report_alert(JM_MSG_NO_FIND_ZERO_SIGNAL, 0); //
			}
		}
		break;
	case JM_RESET_MOVE_DOWN: // 复位向下
		jacking_motor_move_speed(-jm_init_st.motor_target_speed, jm_init_st.move_speed_torque);
		if (jm_init_st.pCkZeroSignal()) // 检测零位信号 有效
		{
			jm_reset_st.reset_state = JM_RESET_TRANS_MOVE_UP;
			jm_reset_st.reset_signal_enable_begin_pos = jacking_motor_get_cur_pos();
		}
		else // 检测零位信号 无效
		{
			if (jm_init_st.pause_flag) // 暂停标记
			{
				jm_reset_st.reset_pause_state = JM_RESET_TRANS_MOVE_DOWN;
				jm_reset_st.reset_state = JM_RESET_PAUSE;
			}

			// if (moveIsStop1(500, 3)==1)
			if (jacking_motor_move_is_stop()) // 电机是否停止
			{
				if (jacking_motor_get_cur_pos() < jm_init_st.safe_door_pos)
				{
					jm_reset_st.reset_state = JM_RESET_TRANS_MOVE_UP_FROM_STOP;
					jm_reset_st.reset_signal_enable_begin_pos = jacking_motor_get_cur_pos();
				}
				else
				{
					jm_reset_st.reset_state = JM_RESET_EXIT;
					jacking_motor_half_current_lock();
					jacking_motor_report_alert(JM_MSG_NO_FIND_ZERO_SIGNAL, 0);
				}
			}
			if (jacking_motor_ck_rst_overtravel()) // 检测传感器信号是否超容差
			{
				jacking_motor_stop(0);
				jm_reset_st.reset_state = JM_RESET_EXIT;
				jacking_motor_report_alert(JM_MSG_BELT_BROKEN, 0); //
			}
		}
		break;
	case JM_RESET_PAUSE:		   // 暂停状态
		if (jm_init_st.pause_flag) // 暂停标记
		{
			jacking_motor_stop(0);
			if (jm_init_st.exit_flag) // 退出标记
			{
				jm_init_st.exit_flag = 0;
				jm_init_st.pause_flag = 0;
				jm_reset_st.reset_state = JM_RESET_EXIT;
			}
		}
		else // 恢复
		{
			jm_reset_st.reset_state = jm_reset_st.reset_pause_state; // 把暂停前的状态 赋值给当前状态
		}
		break;
	case JM_RESET_EXIT: // 退出状态

		// jacking_motor_stop() ;
		jm_reset_st.reset_state = JM_RESET_INIT;
		ret = 1;
		break;
	}
	return ret;
}

uint8_t jacking_motor_para_setting(void) // 参数设置流程
{
	uint8_t ret = 0;
	static uint8_t safe_set_flag = 0;
	if (jm_init_st.handle_ctrl_dir == HD_DIR_UP) // 向上运行
	{
		if (jm_init_st.pCkSafeSignal() && safe_set_flag == 0 && jm_init_st.pre_safe_signal == 0) // 感应到传感器 一次设置只能设置一次安全位 重新设置需退出
		{
			jm_init_st.safe_signal_pos = jacking_motor_get_cur_pos();
			safe_set_flag = 1;
		}
	}
	switch (jm_init_st.sub_ctrl_type)
	{
	case JM_SUB_CTRL_STOP: // 停止
		jacking_motor_stop(0);
		break;
	case JM_SUB_CTRL_HANDLE_UP: // 向上

		jm_init_st.handle_ctrl_dir = HD_DIR_UP;
		jacking_motor_move_speed(jm_init_st.handle_speed, jm_init_st.move_speed_torque);
		break;
	case JM_SUB_CTRL_HANDLE_DOWN: // 向下
		jm_init_st.handle_ctrl_dir = HD_DIR_DOWN;
		jacking_motor_move_speed(-jm_init_st.handle_speed, jm_init_st.move_speed_torque);
		break;
	case JM_SUB_SAVE_DOSSAL_POS: // 保存最高位
		jacking_motor_set_para(PARA_SET_DOSSAL_POS, jacking_motor_get_cur_pos());
		break;
	case JM_SUB_SAVE_CAST_OFF_POS: // 保存脱圈位
		jacking_motor_set_para(PARA_SET_CAST_OFF, jacking_motor_get_cur_pos());
		break;
	case JM_SUB_SAVE_SAFE_DOOR_POS: // 保存安全门
		jacking_motor_set_para(PARA_SET_SAFE_DOOR, jacking_motor_get_cur_pos());
		break;
	}
	jm_init_st.sub_ctrl_type = 0;

	if (jm_init_st.exit_flag)
	{
		jacking_motor_stop(0);
		jm_init_st.exit_flag = 0;
		safe_set_flag = 0;
		ret = 1;
	}
	return ret;
}
uint8_t jacking_motor_move_cast_off(void) // 移动到脱圈位流程
{
	uint8_t ret = 0;
	switch (jm_init_st.move_to_cass_off_st.state)
	{

	case JM_MT_POS_STATE_TRANS_MOVE: // 开始移动
		jacking_motor_move_pos(jm_init_st.cast_off_pos, jm_init_st.motor_target_speed, -1);
		jm_init_st.move_to_cass_off_st.state = JM_MT_POS_STATE_MOVE;
		break;
	case JM_MT_POS_STATE_MOVE:	   // 移动中
		if (jm_init_st.pause_flag) // 暂停标记
		{
			jm_init_st.move_to_cass_off_st.state = JM_MT_POS_STATE_PAUSE;
			jm_init_st.move_to_cass_off_st.pause_state = JM_MT_POS_STATE_TRANS_MOVE;
		}
		if (jacking_motor_move_pos_arr_flag()) // 电机到位
		{
			jm_init_st.move_to_cass_off_st.state = JM_MT_POS_STATE_EXIT;
			jm_init_st.ctrl_main_status = JM_MAIN_STATUS_FINISH;
			jacking_motor_report_status();
		}
		else
		{
			if (jacking_motor_move_is_stop()) // 电机停止
			{
				jm_init_st.move_to_cass_off_st.state = JM_MT_POS_STATE_EXIT;
				// jacking_motor_stop(0) ;
				jacking_motor_half_current_lock();
				jacking_motor_report_alert(JM_MSG_BLOCK_OR_DOSSAL_SET_EEROR, 0);
			}
		}
		break;
	case JM_MT_POS_STATE_PAUSE:	   // 暂停
		if (jm_init_st.pause_flag) // 暂停标记
		{
			jacking_motor_stop(0);
			if (jm_init_st.exit_flag) // 退出标记
			{
				jm_init_st.exit_flag = 0;
				jm_init_st.pause_flag = 0;
				jm_init_st.move_to_cass_off_st.state = JM_MT_POS_STATE_EXIT;
			}
		}
		else // 恢复
		{
			jm_init_st.move_to_cass_off_st.state = jm_init_st.move_to_cass_off_st.pause_state;
		}
		break;
	case JM_MT_POS_STATE_EXIT: // 退出状态
		ret = 1;
		jm_init_st.move_to_cass_off_st.state = JM_MT_POS_STATE_TRANS_MOVE;
		break;
	}
	return ret;
}

uint8_t jacking_motor_move_to_target_pos(void) // 移动到任意位置流程
{
	uint8_t ret = 0;
	static int32_t back_pos = 0;

	if (jm_init_st.exit_flag || jacking_motor_get_alert() != 0) // 退出标记
	{
		jacking_motor_stop(0);
		jm_init_st.exit_flag = 0;
		jm_init_st.pause_flag = 0;
		jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_EXIT;
	}

	switch (jm_init_st.move_to_target_pos_st.state)
	{

	case JM_MT_POS_STATE_TRANS_MOVE: // 开始移动
		jacking_motor_move_pos(jm_init_st.move_target_pos, jm_init_st.motor_target_speed, -1);
		jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_MOVE;
		break;
	case JM_MT_POS_STATE_MOVE: // 移动中
	{
		/*	back_pos = (jm_init_st.dossal_pos - jm_init_st.safe_signal_pos )/3;

			if(jm_init_st.move_to_target_pos_st.jm_mt_pos_stae_move_flag==1)    //目标位置在安全位不亮位置与亮位置之间，回弹1/3
			{
					if((jm_init_st.safe_signal_pos - jm_init_st.safe_signal_st.safe_move_offset_pos <= jm_init_st.move_target_pos)&& (jm_init_st.move_target_pos<=jm_init_st.dossal_pos))
					{
						jacking_motor_move_pos(jm_init_st.safe_signal_pos+back_pos,jm_init_st.motor_target_speed,-1) ;

					}
					if(jacking_motor_get_cur_pos() >= jm_init_st.dossal_pos)  //如果当前位置超过最高位，且安全位传感器亮就停
						{
							if(jm_init_st.pCkSafeSignal())      //安全位灯亮，就完成
							{
								jm_init_st.move_to_target_pos_st.jm_mt_pos_stae_move_flag=0;
								jacking_motor_stop(0) ;
								jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_EXIT ;
								jm_init_st.ctrl_main_status = JM_MAIN_STATUS_FINISH ;
								jacking_motor_report_status() ;
							}
						}
			}
		*/
		static uint8_t pre_zero_signal = 0;
		if (pre_zero_signal == 1 && jm_init_st.pCkZeroSignal() == 0) // 如果零位从有信号到无信号需要更新到初始位置 防止 零位传感器到底部有缝隙 感应不到
		{
			jacking_motor_set_cur_pos(jm_reset_st.reset_zero_pos);
			update_target_pos(posOut2In(jm_init_st.move_target_pos));
		}
		pre_zero_signal = jm_init_st.pCkZeroSignal();

		if (jm_init_st.move_to_target_pos_st.jm_mt_pos_stae_move_flag == 1) // 表示回弹标志，超过安全位就停止
		{

			if (jm_init_st.move_target_pos >= jm_init_st.safe_signal_pos) // 目标位置超过安全位
			{
				if (jm_init_st.pCkSafeSignal()) // 安全位灯亮，就完成
				{
					jm_init_st.move_to_target_pos_st.jm_mt_pos_stae_move_flag = 0;
					jacking_motor_stop(0);
					jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_EXIT;
					jm_init_st.ctrl_main_status = JM_MAIN_STATUS_FINISH;
					jacking_motor_report_status();
				}
			}
		}

		if (jm_init_st.pause_flag) // 暂停标记
		{
			jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_PAUSE;
			jm_init_st.move_to_target_pos_st.pause_state = JM_MT_POS_STATE_TRANS_MOVE;
		}

		if (jacking_motor_move_pos_arr_flag()) // 电机到位
		{
			jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_EXIT;
			jm_init_st.ctrl_main_status = JM_MAIN_STATUS_FINISH;
			jacking_motor_report_status();
			jm_init_st.move_to_target_pos_st.jm_mt_pos_stae_move_flag = 0;
		}
		else
		{
			if (jacking_motor_move_is_stop()) // 电机停止
			{
				jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_EXIT;
				// jacking_motor_stop(0) ;
				jacking_motor_half_current_lock();
				jacking_motor_report_alert(JM_MSG_BLOCK_OR_DOSSAL_SET_EEROR, 0);
			}
		}
	}
	break;
	case JM_MT_POS_STATE_PAUSE:	   // 暂停
		if (jm_init_st.pause_flag) // 暂停标记
		{
			jacking_motor_stop(0);
			if (jm_init_st.exit_flag) // 退出标记
			{
				jm_init_st.exit_flag = 0;
				jm_init_st.pause_flag = 0;
				jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_EXIT;
			}
		}
		else // 恢复
		{
			jm_init_st.move_to_target_pos_st.state = jm_init_st.move_to_target_pos_st.pause_state;
		}
		break;
	case JM_MT_POS_STATE_EXIT: // 退出状态
		ret = 1;
		jm_init_st.move_to_target_pos_st.state = JM_MT_POS_STATE_TRANS_MOVE;
		break;
	}
	return ret;
}

uint8_t jacking_motor_move_safe(void) // 移动到安全位流程
{
	uint8_t ret = 0;

	if (jm_init_st.exit_flag || jacking_motor_get_alert() != 0) // 退出标记
	{
		jacking_motor_stop(0);
		jm_init_st.exit_flag = 0;
		jm_init_st.pause_flag = 0;
		jm_init_st.move_to_safe_st.state = JM_MT_POS_STATE_EXIT;
	}
	switch (jm_init_st.move_to_safe_st.state)
	{

	case JM_MT_POS_STATE_TRANS_MOVE: // 开始移动
		jacking_motor_move_pos(jm_init_st.safe_signal_pos, jm_init_st.motor_target_speed, -1);
		jm_init_st.move_to_safe_st.state = JM_MT_POS_STATE_MOVE;
		break;
	case JM_MT_POS_STATE_MOVE:	   // 移动中
		if (jm_init_st.pause_flag) // 暂停标记
		{
			jm_init_st.move_to_safe_st.state = JM_MT_POS_STATE_PAUSE;
			jm_init_st.move_to_safe_st.pause_state = JM_MT_POS_STATE_TRANS_MOVE;
		}
		if (jacking_motor_move_pos_arr_flag()) // 电机到位
		{
			jm_init_st.move_to_safe_st.state = JM_MT_POS_STATE_EXIT;
			jm_init_st.ctrl_main_status = JM_MAIN_STATUS_FINISH;
			jacking_motor_report_status();
		}
		else
		{
			if (jacking_motor_move_is_stop()) // 电机停止
			{
				jm_init_st.move_to_safe_st.state = JM_MT_POS_STATE_EXIT;
				// jacking_motor_stop(0) ;
				jacking_motor_half_current_lock();
				jacking_motor_report_alert(JM_MSG_BLOCK_OR_DOSSAL_SET_EEROR, 0);
			}
		}
		break;
	case JM_MT_POS_STATE_PAUSE:	   // 暂停状态
		if (jm_init_st.pause_flag) // 暂停标记
		{
			jacking_motor_stop(0);
			if (jm_init_st.exit_flag) // 退出标记
			{
				jm_init_st.exit_flag = 0;
				jm_init_st.pause_flag = 0;
				jm_init_st.move_to_safe_st.state = JM_MT_POS_STATE_EXIT;
			}
		}
		else // 恢复
		{
			jm_init_st.move_to_safe_st.state = jm_init_st.move_to_safe_st.pause_state;
		}
		break;
	case JM_MT_POS_STATE_EXIT: // 退出
		ret = 1;
		jm_init_st.move_to_safe_st.state = JM_MT_POS_STATE_TRANS_MOVE;
		break;
	}
	return ret;
}

uint8_t jacking_motor_move_dossal(void) // 移动到最高位
{
	uint8_t ret = 0;

	if (jm_init_st.exit_flag || jacking_motor_get_alert() != 0) // 退出标记
	{
		jacking_motor_stop(0);
		jm_init_st.exit_flag = 0;
		jm_init_st.pause_flag = 0;
		jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
	}
	switch (jm_init_st.dossal_st.dossal_state)
	{
	case JM_DOSSAL_STATE_INIT:			// 初始状态
		if (jm_init_st.pCkSafeSignal()) // 检测安全位有效
		{
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_TRANS_MOVE_DOWN;
		}
		else // 安全位无效
		{
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_TRANS_HIGH_MOVE_UP;
		}
		break;
	case JM_DOSSAL_STATE_TRANS_MOVE_DOWN: // 向下动作
		jm_init_st.dossal_st.dossal_check_signal_begin_pos = jacking_motor_get_cur_pos();
		jacking_motor_move_speed(-jm_init_st.signal_enable_speed, jm_init_st.move_speed_torque);

		jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_MOVE_DOWN;
		break;
	case JM_DOSSAL_STATE_MOVE_DOWN:		// 向下移动中
		if (jm_init_st.pCkSafeSignal()) // 检测安全位传感器有效
		{
			if (jacking_motor_check_safe_signal_offset()) // 检测安全位传感器 超出容差
			{
				jacking_motor_report_alert(JM_MSG_SIGANL_SAFE_ERROR, 0);
				jacking_motor_stop(0);
				jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
			}
		}
		else // 未超出容差
		{
			jacking_motor_stop(0);
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_TRANS_HIGH_MOVE_UP;
		}
		break;
	case JM_DOSSAL_STATE_TRANS_HIGH_MOVE_UP: // 向上运行动作
		jacking_motor_move_pos(jm_init_st.dossal_pos, jm_init_st.dossal_st.dossal_high_speed, -1);
		jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_HIGH_MOVE_UP;
		break;
	case JM_DOSSAL_STATE_HIGH_MOVE_UP: // 向上运行中
	{
		static uint8_t pre_zero_signal = 0;
		if (pre_zero_signal == 1 && jm_init_st.pCkZeroSignal() == 0) // 如果零位从有信号到无信号需要更新到初始位置 防止 零位传感器到底部有缝隙 感应不到
		{
			jacking_motor_set_cur_pos(jm_reset_st.reset_zero_pos);
			update_target_pos(posOut2In(jm_init_st.dossal_pos));
		}
		pre_zero_signal = jm_init_st.pCkZeroSignal();
		if (jacking_motor_get_cur_pos() > jm_init_st.dossal_st.dossal_safe_dec_pos) // 大于安全位减速点
		{
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_TRANS_LOW_MOVE_UP;
		}
		else
		{
			if (jm_init_st.pause_flag) // 暂停标记
			{
				jm_init_st.dossal_st.dossal_pause_state = JM_DOSSAL_STATE_TRANS_HIGH_MOVE_UP;
				jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_PAUSE;
			}
			if (jacking_motor_move_is_stop()) // 电机停止
			{
				jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
				// jacking_motor_stop(0) ;
				// jacking_motor_half_current_lock();
				jacking_motor_move_pos(jacking_motor_get_cur_pos() - 50, jm_init_st.dossal_st.dossal_high_speed, g_sM1Drive.fltInom / 3);
				jacking_motor_report_alert(JM_MSG_BLOCK_OR_DOSSAL_SET_EEROR, 0);
			}
		}
	}
	break;
	case JM_DOSSAL_STATE_TRANS_LOW_MOVE_UP: // 低速运行
		jacking_motor_move_pos(jm_init_st.dossal_pos, jm_init_st.dossal_st.dossal_low_speed, -1);
		jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_LOW_MOVE_UP;
		break;
	case JM_DOSSAL_STATE_LOW_MOVE_UP:	// 低速运行中
		if (jm_init_st.pCkSafeSignal()) // 检测安全位信号 有效
		{
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_RECORD_SAFE_POS; // 记录安全位位置
		}
		else // 安全位信号 无效
		{
			//
			if (jm_init_st.pause_flag) // 暂停标记
			{
				jm_init_st.dossal_st.dossal_pause_state = JM_DOSSAL_STATE_TRANS_HIGH_MOVE_UP;
				jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_PAUSE;
			}
			if (jacking_motor_move_pos_arr_flag()) // 电机已到位
			{
				// jacking_motor_stop(0) ;
				jacking_motor_report_alert(JM_MSG_NO_FIND_SAFE_SIGNAL, 0);
				jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
			}
			else
			{
				if (jacking_motor_move_is_stop()) // 电机停止
				{
					jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
					// jacking_motor_stop(0) ;
					// jacking_motor_half_current_lock();
					jacking_motor_move_pos(jacking_motor_get_cur_pos() - 50, jm_init_st.dossal_st.dossal_high_speed, g_sM1Drive.fltInom / 3);
					jacking_motor_report_alert(JM_MSG_BLOCK_OR_DOSSAL_SET_EEROR, 0);
				}
			}
		}
		break;
	case JM_DOSSAL_STATE_RECORD_SAFE_POS:	// 记录当前位置
		if (jacking_motor_check_safe_pos()) // 检测安全位 超容差
		{
			jacking_motor_stop(0);
			jacking_motor_report_alert(JM_MSG_BELT_BROKEN, 0);
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
		}
		else // //检测安全位 未超容差
		{
			jm_init_st.safe_signal_st.safe_offset_pos = jacking_motor_get_cur_pos() - jm_init_st.safe_signal_pos;
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_TRANS_SAFE_TO_DOSSAL;
		}
		break;
	case JM_DOSSAL_STATE_TRANS_SAFE_TO_DOSSAL: // 安全位到最高位开始 已安全位位置为参考
		/*	if(jm_init_st.safe_signal_st.safe_offset_pos>0)
			  {
				   jm_init_st.dossal_st.dossal_clib_pos =jm_init_st.dossal_pos  ;
			  }
			  else*/
		{
			jm_init_st.dossal_st.dossal_clib_pos = jm_init_st.dossal_pos + jm_init_st.safe_signal_st.safe_offset_pos;
		}
		//  jm_init_st.dossal_st.dossal_clib_pos =jm_init_st.dossal_pos + jm_init_st.safe_signal_st.safe_offset_pos ;
		jacking_motor_move_pos(jm_init_st.dossal_st.dossal_clib_pos, jm_init_st.dossal_st.dossal_low_speed, -1);
		// jacking_motor_move_pos(jm_init_st.dossal_st.dossal_clib_pos, jm_init_st.dossal_st.dossal_low_speed, jm_init_st.safe_to_dossal_torque);

		jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_SAFE_TO_DOSSAL;
		jm_init_st.dossal_st.dossal_begin_time = getTicks();
		jacking_motor_change_pid(3);
		break;
	case JM_DOSSAL_STATE_SAFE_TO_DOSSAL: // 安全位到最高位中
		if (jm_init_st.pause_flag)		 // 暂停标记
		{
			jacking_motor_change_pid(8);
			jm_init_st.dossal_st.dossal_pause_state = JM_DOSSAL_STATE_TRANS_SAFE_TO_DOSSAL;
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_PAUSE;
		}
		if (jacking_motor_move_pos_arr_flag()) // 电机已到位
		{
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
			jm_init_st.ctrl_main_status = JM_MAIN_STATUS_FINISH;
			jacking_motor_report_status();
		}
		else
		{
			if (jacking_motor_move_is_stop()) // 电机停止
			{
				jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
				// jacking_motor_stop(0) ;
				jacking_motor_half_current_lock();
				jacking_motor_report_alert(JM_MSG_BLOCK_OR_DOSSAL_SET_EEROR, 0);
			}
		}
		if (expireTicks(jm_init_st.dossal_st.dossal_begin_time, jm_init_st.dossal_st.dossal_time_out_max)) // 最高位超时
		{
			jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
			// jacking_motor_stop(1) ;
			// jacking_motor_stop(0) ;
			jacking_motor_half_current_lock();
			jacking_motor_report_alert(JM_MSG_DOSSAL_SET_ERROR, 0);
		}
		break;
	case JM_DOSSAL_STATE_PAUSE:	   // 暂停状态
		if (jm_init_st.pause_flag) // 暂停标记
		{
			jacking_motor_stop(0);
			if (jm_init_st.exit_flag) // 退出标记
			{
				jm_init_st.exit_flag = 0;
				jm_init_st.pause_flag = 0;
				jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_EXIT;
			}
		}
		else // 恢复
		{
			jm_init_st.dossal_st.dossal_state = jm_init_st.dossal_st.dossal_pause_state; // 暂停时状态恢复到当前状态
		}
		break;
	case JM_DOSSAL_STATE_EXIT: // 退出标记
		ret = 1;
		jacking_motor_change_pid(8);
		jm_init_st.dossal_st.dossal_state = JM_DOSSAL_STATE_INIT;
		break;
	}
	return ret;
}

uint8_t jacking_motor_move_dossal_to_safe(void) // 挂布位移动到安全位
{
	uint8_t ret = 0;
	static uint8_t error_cnt = 0;
	static uint32_t cur_tick = 0;
	if (jm_init_st.exit_flag || jacking_motor_get_alert() != 0) // 退出标记
	{
		jacking_motor_stop(0);
		jm_init_st.exit_flag = 0;
		jm_init_st.pause_flag = 0;
		jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_EXIT;
	}
	switch (jm_init_st.safe_signal_st.safe_state)
	{
	case JM_SAFE_STATE_INIT: // 初始状态
		error_cnt = 0;
		jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_TRANS_MOVE_SAFE;
		// jacking_motor_motor_move_is_stop_resh();
		break;
	case JM_SAFE_STATE_TRANS_MOVE_SAFE:																			// 开始向下运行
		jacking_motor_move_speed(-jm_init_st.safe_signal_st.safe_speed, jm_init_st.safe_signal_st.safe_torque); //
		jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_MOVE_SAFE;
		break;
	case JM_SAFE_STATE_MOVE_SAFE:		// 向下运行中
		if (jm_init_st.pCkSafeSignal()) // 检测安全位信号 有效
		{
			if (jacking_motor_move_is_stop()) // 电机停止
			{
				jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_TRANS_MOVE_DOSSAL;
			}
			else // 未停止
			{
				if (jacking_motor_check_safe_pos()) // 检测安全位传感器达到容差
				{
					jacking_motor_stop(0);
					jacking_motor_report_alert(JM_MSG_SAFE_SIGNAL_OR_POS_ERROR, 0);
					jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_EXIT;
				}
				else // 未到容差
				{
					if (jm_init_st.pause_flag) // 暂停标记
					{
						jm_init_st.safe_signal_st.safe_pause_state = JM_SAFE_STATE_TRANS_MOVE_SAFE;
						jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_PAUSE;
					}
				}
			}
		}
		else // 检测安全位信号无效
		{

			if (jacking_motor_check_safe_pos()) // 检测安全位传感器达到容差
			{
				jacking_motor_stop(0);
				jacking_motor_report_alert(JM_MSG_SAFE_SIGNAL_OR_POS_ERROR, 0);
				jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_EXIT;
			}
			else // 未到容差
			{
				// cur_tick = getTicks() ;                //安全位的偏置= jm_init_st.safe_signal_st.safe_move_offset_pos
				jacking_motor_move_pos(jm_init_st.safe_signal_pos - jm_init_st.safe_signal_st.safe_move_offset_pos, jm_init_st.motor_target_speed, jm_init_st.safe_signal_st.safe_torque); // jm_init_st.safe_signal_st.safe_torque
				jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_READY_FINISH;
			}
		}
		break;
	case JM_SAFE_STATE_READY_FINISH: // 准备停止，防止停止时安全位灯不亮 ，走偏移量

		if (jacking_motor_move_pos_arr_flag())
		{
			jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_SAFE_POS_TO_PULL_DOWN;
			jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_EXIT;
			jm_init_st.ctrl_main_status = JM_MAIN_STATUS_FINISH;
			reportState(CAN_SEND_ID,mot_id,0);
			jacking_motor_report_status();
		}
		else
		{
			if (jacking_motor_move_is_stop()) // 电机停止
			{
				jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_TRANS_MOVE_DOSSAL;
			}
		}
		break;
	case JM_SAFE_STATE_TRANS_MOVE_DOSSAL: // 开始移动到最高位
		jacking_motor_move_pos(jm_init_st.dossal_st.dossal_clib_pos, jm_init_st.dossal_st.dossal_low_speed, -1);
		jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_MOVE_DOSSAL;
		break;
	case JM_SAFE_STATE_MOVE_DOSSAL:			   // 移动到最高位中
		if (jacking_motor_move_pos_arr_flag()) // 电机已到位
		{
			jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_ERROR_AGAIN; //
		}
		else // 电机未到位
		{
			if (jm_init_st.pause_flag)
			{
				jm_init_st.safe_signal_st.safe_pause_state = JM_SAFE_STATE_TRANS_MOVE_DOSSAL;
				jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_PAUSE;
			}
		}
		break;
	case JM_SAFE_STATE_ERROR_AGAIN: // 重试安全位
		error_cnt++;
		if (error_cnt > 3)
		{
			jacking_motor_stop(0);
			jacking_motor_report_alert(JM_MSG_MOVE_SAFE_TORQUE_SMALL, 0);
			jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_EXIT;
		}
		else
		{
			jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_TRANS_MOVE_SAFE;
		}
		break;
	case JM_SAFE_STATE_PAUSE:	   // 暂停状态
		if (jm_init_st.pause_flag) // 暂停标记
		{
			// jacking_motor_stop(0) ;
			jacking_motor_half_current_lock();
			if (jm_init_st.exit_flag) // 退出标记
			{
				jm_init_st.exit_flag = 0;
				jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_EXIT;
			}
		}
		else //
		{
			jm_init_st.safe_signal_st.safe_state = jm_init_st.safe_signal_st.safe_pause_state;
		}
		break;
	case JM_SAFE_STATE_EXIT: // 退出状态
		ret = 1;
		jm_init_st.safe_signal_st.safe_state = JM_SAFE_STATE_INIT;
		break;
	}
	return ret;
}
uint8_t jacking_motor_pull_down_cloth(void) // 拉布流程
{
	uint8_t ret = 0;
	static uint8_t sub_state = 0;
	switch (jm_init_st.pdc_st.pdc_state) // 拉布状态机
	{
	case JM_PDC_STATE_SAFE_POS_TO_PULL_DOWN:									// 安全位切拉布时的状态
		if (jacking_motor_check_torque_pull_down(jm_init_st.pdc_st.pdc_torque)) // 检测是否可以达到可拉布的状态
		{
			jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_INIT;
		}
		break;
	case JM_PDC_STATE_INIT: // 拉布初始化
		jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_TRANS_TORQUE;
		jm_init_st.pdc_st.pdc_begin_pos = jacking_motor_get_cur_pos();
		jm_init_st.pdc_st.pdc_begin_pos_limit = jacking_motor_get_cur_pos();
		break;
	case JM_PDC_STATE_TRANS_TORQUE: // 拉布开始力矩模式
		jacking_motor_move_torque(jm_init_st.pdc_st.pdc_torque);
		sub_state = 0;
		jm_init_st.pdc_st.pdc_change_torque_begin_time = getTicks();
		jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_TORQUE;
		break;
	case JM_PDC_STATE_TORQUE:							// 拉布中
		if (jm_init_st.pdc_st.pdc_change_torque_enable) // 大小力矩切换使能
		{
			if (jm_init_st.pdc_st.pdc_main_ctrl_flag == 1)
			{
				jm_init_st.pdc_st.pdc_begin_pos_limit = jacking_motor_get_cur_pos();
				jm_init_st.pdc_st.pdc_main_ctrl_flag = 0;
			}
			switch (sub_state)
			{
			case 0:
				if (expireTicks(jm_init_st.pdc_st.pdc_change_torque_begin_time, jm_init_st.pdc_st.pdc_torque_time))
				{
					jm_init_st.pdc_st.pdc_change_torque_begin_time = getTicks();
					jacking_motor_move_torque(jm_init_st.pdc_st.pdc_big_torque);
					sub_state = 1;
				}
				break;
			case 1:
				if (expireTicks(jm_init_st.pdc_st.pdc_change_torque_begin_time, jm_init_st.pdc_st.pdc_big_torque_time))
				{
					jm_init_st.pdc_st.pdc_change_torque_begin_time = getTicks();
					jacking_motor_move_torque(jm_init_st.pdc_st.pdc_torque);
					sub_state = 0;
				}
				break;
			}
		}
		else
		{
			if (jm_init_st.pdc_st.pdc_main_ctrl_flag == 1)
			{
				jm_init_st.pdc_st.pdc_begin_pos_limit = jacking_motor_get_cur_pos();
				jacking_motor_move_torque(jm_init_st.pdc_st.pdc_torque);
				jm_init_st.pdc_st.pdc_main_ctrl_flag = 0;
			}
		}

		if (jm_init_st.pdc_st.pdc_pull_max_pos != 0)
		{
			if ((jm_init_st.pdc_st.pdc_begin_pos_limit - jacking_motor_get_cur_pos()) > jm_init_st.pdc_st.pdc_pull_max_pos) // 这里增加拉布最大位置限制-----拉布超速
			{
				jacking_motor_stop(0);
				jacking_motor_report_alert(JM_MSG_TORQUE_OVER_SPEED, ALT_RESET_FLAG);
				jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_EXIT;
			}
		}
		// if ((jacking_motor_get_cur_pos() > jm_init_st.pdc_st.pdc_begin_pos + 20) && (jacking_motor_get_cur_pos() >jm_init_st.safe_signal_pos) ) // 如果拉布反弹 且 当前位置大于安全位传感器亮的位置
		if ((jacking_motor_get_cur_pos() > jm_init_st.pdc_st.pdc_begin_pos + 20) && (jacking_motor_get_cur_pos() > jm_init_st.safe_signal_pos))
		{
                #if (BOARD_TYPE == BOARD_E139)
			motUpdateSpeedPidI(getCurTorq(1));
                #endif
			//	jacking_motor_move_pos(jm_init_st.pdc_st.pdc_begin_pos,jm_init_st.handle_speed) ;
			jacking_motor_stop(0);
			// jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_BLANCE_TORQUE_BIG ;
			jacking_motor_report_alert(JM_MSG_BALANCE_TOO_BIG, ALT_RESET_FLAG); // ALT_RESET_FLAG
			jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_EXIT;
		}
		else if (jacking_motor_torque_over_speed()) // 如果拉布超速
		{
			jacking_motor_stop(0);
			jacking_motor_report_alert(JM_MSG_TORQUE_OVER_SPEED, ALT_RESET_FLAG);
			jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_EXIT;
		}
		/*else if(jacking_motor_get_cur_pos()<= jm_init_st.cast_off_pos) //拉布到达脱圈位
		{
			jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_EXIT ;
		}*/
		else if (jm_init_st.pCkZeroSignal()) // 检测零位传感器
		{
			jacking_motor_stop(0);
			jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_EXIT; // 20250619
		}
		// else if(jm_init_st.pdc_st.pdc_torque_change_flag) //拉布切换力矩
		//{
		//	//jm_init_st.pdc_st.pdc_begin_pos = jacking_motor_get_cur_pos() ;
		//                             jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_TRANS_TORQUE ;
		//	jm_init_st.pdc_st.pdc_torque_change_flag =0 ;
		//}
		else
		{
			if (jm_init_st.pause_flag) // 暂停标记
			{
				jm_init_st.pdc_st.pdc_pause_state = JM_PDC_STATE_TRANS_TORQUE;
				jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_PAUSE;
			}
		}
		if (jm_init_st.exit_flag) // 退出标记
		{
                 #if (BOARD_TYPE == BOARD_E139)
			motUpdateSpeedPidI(getCurTorq(1));
                  #endif
			jacking_motor_stop(0);
			jm_init_st.exit_flag = 0;
			jm_init_st.pause_flag = 0;
			jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_EXIT;
		}
		break;
	case JM_PDC_STATE_BLANCE_TORQUE_BIG:	   // 平衡力矩不对
		if (jacking_motor_move_pos_arr_flag()) // 电机已到位
		{
			jacking_motor_report_alert(JM_MSG_BALANCE_TOO_BIG, 0);
			jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_EXIT;
		}
		break;
	case JM_PDC_STATE_PAUSE:	   // 暂停状态
		if (jm_init_st.pause_flag) // 暂停标记
		{
                        #if (BOARD_TYPE == BOARD_E139)
			motUpdateSpeedPidI(getCurTorq(1));
                        #endif
			jacking_motor_stop(0);
			if (jm_init_st.exit_flag) // 退出标记
			{
				jm_init_st.exit_flag = 0;
				jm_init_st.pause_flag = 0;
				jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_EXIT;
			}
		}
		else // 恢复
		{
			jm_init_st.pdc_st.pdc_state = jm_init_st.pdc_st.pdc_pause_state;
		}
		break;
	case JM_PDC_STATE_EXIT: // 退出状态
		ret = 1;
		jm_init_st.pdc_st.pdc_state = JM_PDC_STATE_INIT;
		break;
	}
	return ret;
}
void jacking_motor_task_loop(void) // 升降电机主流程
{
	static uint8_t main_state = 0;
	static uint8_t sub_state = 0;
	jm_init_st.zero_signal = jm_init_st.pCkZeroSignal();
	jm_init_st.safe_signal = jm_init_st.pCkSafeSignal();
	jm_init_st.motor_target_speed = jacking_motor_get_speed_from_pos();
	jacking_motor_run_report_current();
	if (jm_init_st.state == JM_STATE_INIT || jm_init_st.state == JM_STATE_HANDLE_UP || jm_init_st.state == JM_STATE_HANDLE_DOWN || jm_init_st.state == JM_STATE_PARA_SET)
	{
	}
	else
	{
		jacking_motor_update_speed(jm_init_st.motor_target_speed);
	}
	if (jm_init_st.ctrl_type == JM_CTRL_PAUSE) // 暂停
	{
		jm_init_st.pause_flag = 1;
		jm_init_st.ctrl_type = JM_CTRL_NULL;
	}
	else if (jm_init_st.ctrl_type == JM_CTRL_RECOVER) // 恢复
	{
		jm_init_st.pause_flag = 0;
		jm_init_st.ctrl_type = JM_CTRL_NULL;
	}
	else if (jm_init_st.ctrl_type == JM_CTRL_EXIT) // 退出
	{
		jm_init_st.exit_flag = 1;
		jm_init_st.ctrl_type = JM_CTRL_NULL;
	}
	jm_init_st.ctrl_main_status = jm_init_st.state;
	switch (jm_init_st.state)
	{
	case JM_STATE_INIT: //  初始状态
		jm_init_st.pause_flag = 0;
		switch (jm_init_st.ctrl_type) // 控制指令
		{

		case JM_CTRL_RESET: // 进入复位
			jm_init_st.state = JM_STATE_RESET;

			break;
		case JM_CTRL_MOVE_TO_DOSSAL: // 进入最高位

			jm_init_st.state = JM_STATE_MOVE_DOSSAL;

			break;
		case JM_CTRL_MOVE_DOSSAL_TO_SAFE: // 最高位到安全位
			jm_init_st.state = JM_STATE_MOVE_DOSSAL_TO_SAFE;

			break;
		case JM_CTRL_PULL_DOWN_CLOTH: // 进入拉布
			jm_init_st.state = JM_STATE_PULL_DOWN_CLOTH;

			break;
		case JM_CTRL_MOVE_TO_SAFE: // 移动到安全位
			jm_init_st.state = JM_STATE_MOVE_TO_SAFE;

			break;
		case JM_CTRL_MOVE_TO_CAST_OFF: // 移动到脱布位
			jm_init_st.state = JM_STATE_MOVE_TO_CAST_OFF;

			break;
		case JM_CTRL_SET_PARA: // 设置参数
			jm_init_st.state = JM_STATE_PARA_SET;

			break;
		case JM_CTRL_MOVE_TO_TARGET_POS: // 移动到任意位置
			jm_init_st.state = JM_STATE_MOVE_TO_TARGET_POS;
			break;
		default:
			if (jm_init_st.sub_ctrl_type == JM_SUB_CTRL_HANDLE_UP) // 手动上升
			{
				jm_init_st.state = JM_STATE_HANDLE_UP;
				jm_init_st.handle_ctrl_dir = HD_DIR_UP;
				// jacking_motor_move_speed(jm_init_st.motor_target_speed,jm_init_st.move_speed_torque) ;
				jacking_motor_move_speed(jm_init_st.handle_speed, jm_init_st.move_speed_torque);
			}
			else if (jm_init_st.sub_ctrl_type == JM_SUB_CTRL_HANDLE_DOWN) // 手动下降
			{
				jm_init_st.state = JM_STATE_HANDLE_DOWN;
				jm_init_st.handle_ctrl_dir = HD_DIR_DOWN;
				// jacking_motor_move_speed(-jm_init_st.motor_target_speed,jm_init_st.move_speed_torque) ;
				jacking_motor_move_speed(-jm_init_st.handle_speed, jm_init_st.move_speed_torque);
			}
			break;
		}

		break;
	case JM_STATE_RESET: // 复位中
		if (jacking_motor_reset())
		{
			jm_init_st.state = JM_STATE_INIT;
		}
		jm_init_st.ctrl_sub_status = jm_reset_st.reset_state;
		break;
	case JM_STATE_PARA_SET: // 设置参数中
		if (jacking_motor_para_setting())
		{
			jm_init_st.state = JM_STATE_INIT;
		}
		jm_init_st.ctrl_sub_status = 0;
		break;
	case JM_STATE_MOVE_DOSSAL: // 移动到最高位中
		if (jacking_motor_move_dossal())
		{
			jm_init_st.state = JM_STATE_INIT;
		}
		jm_init_st.ctrl_sub_status = jm_init_st.dossal_st.dossal_state;
		break;
	case JM_STATE_MOVE_DOSSAL_TO_SAFE: // 最高位移动到安全位中
		if (jacking_motor_move_dossal_to_safe())
		{
			jm_init_st.state = JM_STATE_INIT;
		}
		jm_init_st.ctrl_sub_status = jm_init_st.safe_signal_st.safe_state;
		break;
	case JM_STATE_PULL_DOWN_CLOTH: // 拉布中
		if (jacking_motor_pull_down_cloth())
		{
			jm_init_st.state = JM_STATE_INIT;
		}
		jm_init_st.ctrl_sub_status = jm_init_st.pdc_st.pdc_state;
		break;
	case JM_STATE_MOVE_TO_SAFE: // 移动到安全位中
		if (jacking_motor_move_safe())
		{
			jm_init_st.state = JM_STATE_INIT;
		}
		jm_init_st.ctrl_sub_status = jm_init_st.move_to_safe_st.state;
		break;
	case JM_STATE_MOVE_TO_CAST_OFF: // 移动到脱圈位中
		if (jacking_motor_move_cast_off())
		{
			jm_init_st.state = JM_STATE_INIT;
		}
		jm_init_st.ctrl_sub_status = jm_init_st.move_to_cass_off_st.state;
		break;
	case JM_STATE_MOVE_TO_TARGET_POS: // 移动到任意位置
		if (jacking_motor_move_to_target_pos())
		{
			jm_init_st.state = JM_STATE_INIT;
		}
		jm_init_st.ctrl_sub_status = jm_init_st.move_to_target_pos_st.state;
		break;
	case JM_STATE_HANDLE_UP: // 手动上升
		if (jm_init_st.sub_ctrl_type == JM_SUB_CTRL_STOP)
		{
			jacking_motor_stop(0);
			jm_init_st.state = JM_STATE_INIT;
		}
		else if (jm_init_st.sub_ctrl_type == JM_SUB_CTRL_HANDLE_DOWN)
		{
			jm_init_st.state = JM_STATE_HANDLE_DOWN;
			jm_init_st.handle_ctrl_dir = HD_DIR_DOWN;
			jacking_motor_move_speed(-jm_init_st.handle_speed, jm_init_st.move_speed_torque);
		}
		break;
	case JM_STATE_HANDLE_DOWN: // 手动下降
		if (jm_init_st.sub_ctrl_type == JM_SUB_CTRL_STOP)
		{
			jacking_motor_stop(0);
			jm_init_st.state = JM_STATE_INIT;
		}
		if (jm_init_st.sub_ctrl_type == JM_SUB_CTRL_HANDLE_UP)
		{
			jm_init_st.state = JM_STATE_HANDLE_UP;
			jm_init_st.handle_ctrl_dir = HD_DIR_UP;
			jacking_motor_move_speed(jm_init_st.handle_speed, jm_init_st.move_speed_torque);
		}
		break;
	}
#ifdef DEBUG_CMD_PRINT
	if ((main_state != jm_init_st.ctrl_main_status) || (sub_state != jm_init_st.ctrl_sub_status))
	{
		reportDebug(1, jm_init_st.ctrl_main_status, jm_init_st.ctrl_sub_status);
		main_state = jm_init_st.ctrl_main_status;
		sub_state = jm_init_st.ctrl_sub_status;
	}
#endif
	jm_init_st.pre_safe_signal = jm_init_st.pCkSafeSignal();
	jm_init_st.ctrl_type = JM_CTRL_NULL;
	jm_init_st.exit_flag = 0;
}

void jacking_motor_ctrl(JM_CTRL_TYPE type, JM_SUB_CTRL_TYPE sub_type, int16_t data) // 控制命令
{
	/*if(type == JM_CTRL_PAUSE && jm_init_st.state >0)
	{
		jm_init_st.ctrl_type = type ;
	}
	else*/
	if (JM_STATE_INIT == jm_init_st.state && type == JM_CTRL_EXIT)
	{
		return;
	}
	jm_init_st.ctrl_type = type;
	jm_init_st.sub_ctrl_type = sub_type;

	if (type == JM_CTRL_MOVE_TO_TARGET_POS)
	{
		jm_init_st.move_target_pos = data;
		jm_init_st.move_to_target_pos_st.jm_mt_pos_stae_move_flag = sub_type; // 回弹标志位
	}
	else if (type == JM_CTRL_PULL_DOWN_CLOTH)
	{
		jm_init_st.pdc_st.pdc_torque = torqOut2In(data) * 1000;
		jm_init_st.pdc_st.pdc_main_ctrl_flag = 1;
	}

	if (sub_type == JM_SUB_CTRL_HANDLE_UP || sub_type == JM_SUB_CTRL_HANDLE_DOWN)
	{
		if (data == 0)
		{
			jm_init_st.handle_speed = jm_init_st.debug_speed;
		}
		else
			jm_init_st.handle_speed = data;
	}
	/*    if(jm_init_st.sub_ctrl_type ==JM_SUB_CTRL_HANDLE_UP || jm_init_st.sub_ctrl_type ==JM_SUB_CTRL_HANDLE_DOWN ) //手动上升下降时 自动发退出流程命令
		{
		   if(jm_init_st.state !=JM_STATE_PARA_SET )
		   jm_init_st.ctrl_type = JM_CTRL_EXIT ;
		}*/
}
uint8_t jacking_motor_get_main_stat(void) // 获取升降电机的主状态
{
	return jm_init_st.state;
}
#endif