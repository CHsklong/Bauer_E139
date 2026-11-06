
#include "wire_feed_motor.h"
#include "motor.h"
#include "alert.h"
#include "work_io.h"
#include "math.h"
#include "av_data.h"
#include "tick.h"
#include "param.h"
#include "hal_for_upg.h"
#include "user_app_fun.h"

#ifdef WIRE_MOTOR_FUN
WF_SETTING_ST wf_setting_st = {

};
WF_INIT_ST wf_init_st = {
	.move_speed = 20,		   // 钢丝移动速度 RPM
	.feed_speed = 50,		   // 送丝移动速度 RPM
	.collection_speed = 50,	   // 收丝移动速度 PRM
	.signal_enable_speed = 10, // 检测传感器的速度 RPM
	.move_speed_torque = 1000, // 速度模式下的力矩
	.zero_pos = 0,			   // 送丝零位位置
	.ctrl_type = WF_CTRL_NULL,
	.sub_ctrl_type = 0,
	.state = WF_STATE_INIT,
	.feed_dec_speed = 10, // 20RPM
	.zero_signal_allow_pos = 200,
	.wire_broken_check_pos = 1000,		 // 检测断丝传感器的容差值
	.wire_broken_check_type = 2,		 // 断丝检测类型
	.wire_motor_signal_set_type = 2,	 // 为杰泰类型的设置零位
	.wire_broken_retry_factory_type = 2, // 为断丝需要回退的厂家类型
};
WF_RESET_ST wf_reset_st = {
	.reset_state = WF_RESET_INIT,
	.reset_signal_max_pos = 1000,
	.reset_limit_pos = 0x5124,
};

int32_t wire_feed_motor_abs(int32_t value) // 绝对值
{
	int32_t ret;
	if (value >= 0)
		ret = value;
	else
		ret = -value;
	return ret;
}
int32_t wire_feed_motor_get_cur_pos(void) // 获取送丝电机当前位置
{
	return posIn2Out(getCurPos());
}
int16_t wire_feed_motor_get_current(void) // 获取送丝电机当前电流
{
	return torqIn2Out(fabs(getCurTorq(1)));
}
uint8_t wire_feed_check_zero_signal(void) // 零位传感器检测 1有效 0无效
{

	return work_input_get_state(IN_INPUT2);
}
uint8_t wire_feed_check_end_signal(void) // 到位传感器检测 1有效 0 无效
{
	if (wf_init_st.end_signal_stat == SIGNAL_NO_USE) // 不使用
	{
		return 0;
	}
	else
		return work_input_get_state(IN_INPUT1);
}
uint8_t wire_feed_broken_wire_signal(void) // 断丝传感器检测 1有效 0 无效
{
	if (wf_init_st.broken_signal_stat == SIGNAL_NO_USE) // 不使用
	{
		return 0;
	}
	else
		return work_input_get_state(IN_INPUT3);
}
void wire_feed_signal_set_def_state() // 设置传感器默认状态
{

	wf_init_st.end_signal_stat = wf_init_st.wire_signal_def_status & 0x02;
	wf_init_st.zero_signal_stat = (wf_init_st.wire_signal_def_status >> 2) & 0x02;
	wf_init_st.broken_signal_stat = (wf_init_st.wire_signal_def_status >> 4) & 0x02;

	work_input_set_def_state(IN_INPUT1, (~wf_init_st.end_signal_stat) & 0x01);
	work_input_set_def_state(IN_INPUT2, (~wf_init_st.zero_signal_stat) & 0x01);
	work_input_set_def_state(IN_INPUT3, (~wf_init_st.broken_signal_stat) & 0x01);
}
uint8_t wrie_feed_broken_wire_signal_of_hall(void) // 断丝传感器检测 指定位置时 传感器状态
{
	static uint8_t pre_signal = 255;
	// static int32_t edge_change_pos = 0 ;
	uint8_t cur_signal = work_input_get_state(IN_INPUT3);
	if (wf_init_st.broken_signal_stat == SIGNAL_NO_USE) // 不使用
	{
		return 0;
	}
	if (cur_signal != pre_signal)
	{
		wf_init_st.edge_change_pos = wire_feed_motor_get_cur_pos();
		pre_signal = cur_signal;
	}
	else
	{
		if (wire_feed_motor_abs(wire_feed_motor_get_cur_pos() - wf_init_st.edge_change_pos) > wf_init_st.wire_broken_check_pos) // 断丝或堵丝
		{
                        wf_init_st.edge_change_pos = wire_feed_motor_get_cur_pos();
			return 1;
		}
	}
	return 0;
}
void wire_feed_motor_init_pos(void) // 初始化单圈位置
{
	initMotorPos();
	wf_init_st.edge_change_pos = wire_feed_motor_get_cur_pos();
}
void wire_feed_motor_set_cur_pos(int32_t pos) // 设置送丝电机当前位置
{
	setCurPos(pos);
	wf_init_st.edge_change_pos = wire_feed_motor_get_cur_pos();
}
uint16_t wire_feed_motor_get_cur_current(void) // 获取送丝电机电流
{
	return wire_feed_motor_abs((int16_t)(getCurTorq(1) * 1000));
}
void wire_feed_motor_set_broken_check_type(void) // 设置感应方式
{
	if (wf_init_st.wire_broken_check_type == 2) // 脉冲感应方式
		wf_init_st.pCkBrokenSignal = wrie_feed_broken_wire_signal_of_hall;
	else
		wf_init_st.pCkBrokenSignal = wire_feed_broken_wire_signal;
}
void wire_feed_motor_init(void) // 送丝电机初始化
{
	wf_init_st.pCkZeroSignal = wire_feed_check_zero_signal;
	wf_init_st.pCkEndSignal = wire_feed_check_end_signal;
	//wf_init_st.wire_broken_check_pos = 5000;

	//wf_init_st.wf_work_st.work_cur_back_pos = 1000;
	if (wf_init_st.wire_broken_check_type == 2) // 脉冲感应方式
		wf_init_st.pCkBrokenSignal = wrie_feed_broken_wire_signal_of_hall;
	else
		wf_init_st.pCkBrokenSignal = wire_feed_broken_wire_signal;

	/*wf_init_st.wf_work_st.work_target_pos = 10000;
	wf_init_st.wf_work_st.work_feed_block_torque = 700;	   // ma
	wf_init_st.wf_work_st.work_collect_block_torque = 700; // ma
	wf_init_st.wf_work_st.work_back_pos = 1000;
	wf_init_st.wf_work_st.work_feed_block_retry_num = 3;
	wf_init_st.feed_dec_pos = 100;		   // 5针之前作为减速位置  wf_init_st.end_pos- 58 *5
        wf_init_st.feed_zero_dec_pos = 100 ;
	wf_init_st.end_allowance_pos = 58 * 2; // 2针
	wf_init_st.wf_check_st.check_speed = 40;
	wf_init_st.wire_end_back_pos = 58;	   // 默认回退一针
	wf_init_st.wire_signal_def_status = 0; */// 默认常开
	wf_init_st.report_dis_pos = 0;
	wf_init_st.report_cur_ptr = 0;
	wf_init_st.zero_to_half_flag = 1;
	// wire_feed_motor_set_cur_pos(jm_reset_st.reset_zero_pos) ;
	motSetGearRatio(800, 16384);
	// enableMotor(true);
}
uint8_t wire_feed_end_write_flag(void) // 末端设置标记
{
	if (wf_init_st.end_pos > 0)
	{
		return 1;
	}
	else
		return 0;
}
void wire_feed_motor_save_zero(void) // 送丝电机保存零位
{
	saveZeroPos();
}
void wire_feed_motor_clear_alert(void) // 送丝电机清除报警
{
	alertClear();
}
void wire_feed_motor_enable(uint8_t enb) // 送丝电机使能 1有效 0无效
{
	enableMotor(enb);
}
void wire_feed_motor_save_end_pos(void)
{
	Param_Data paraData;
	if (mot_run_state.motRun == true)
	{
		enableMotor(false);
		hal_delayms(50);
	}

	wf_init_st.end_pos = wire_feed_motor_get_cur_pos();
	paraData.i4_data = wf_init_st.end_pos;
	writeWfMotorParameter(4, paraData);
}

void wire_feed_motor_save_signal_zero_pos(void) // 保存第一次设置零位传感器亮的位置
{
	Param_Data paraData;
	if (mot_run_state.motRun == true)
	{
		enableMotor(false);
		hal_delayms(50);
	}

	wf_init_st.wire_signal_save_zero_jt = wire_feed_motor_get_cur_pos();
	paraData.i4_data = wf_init_st.wire_signal_save_zero_jt;
	writeWfMotorParameter(23, paraData);
}

void wire_feed_motor_move_speed(int16_t speed, uint16_t torque) // 送丝电机速度模式 speed = rpm  torque = ma
{
	float fTorque = torque;
	wire_feed_motor_clear_alert();
	wire_feed_motor_enable(1);

	if (speed > 0)
	{
		wf_init_st.run_dir = WF_RUN_FEED_DIR;
	}
	else if (speed < 0)
		wf_init_st.run_dir = WF_RUN_COLLECT_DIR;
	else
		wf_init_st.run_dir = WF_STOP;
	fTorque = fTorque / 1000.0f;
	moveSpd(spdOut2In(MOT_MSG_MOVE_SPD, speed * motor.gear_ratio * 10.0f), fTorque);
}
void wire_feed_motor_move_pos(int32_t pos, uint16_t speed) // 送丝电机位置模式 pos =encoder  speed = rpm
{
	wire_feed_motor_clear_alert();
	wire_feed_motor_enable(1);
	mot_run_state.target_pos_offset = 3;
	mot_run_state.posReq = pos;
	if (pos > wf_init_st.wf_work_st.work_cur_pos) // 如果目标位置大于当前位置 则送丝
	{
		wf_init_st.run_dir = WF_RUN_FEED_DIR;
	}
	else if (pos < wf_init_st.wf_work_st.work_cur_pos) // 如果目标位置小于当前位置 则收丝
	{
		wf_init_st.run_dir = WF_RUN_COLLECT_DIR;
	}
	else
		wf_init_st.run_dir = WF_STOP;
	movePos(posOut2In(pos), spdOut2In(MOT_MSG_MOVE_POS, speed * motor.gear_ratio), -1);
}
void wire_feed_motor_move_relative_pos(int32_t pos, uint16_t speed) // 送丝电机相对位置模式
{
	wire_feed_motor_clear_alert();
	wire_feed_motor_enable(1);
	mot_run_state.target_pos_offset = 3;
	mot_run_state.posReq = pos + posIn2Out(getCurPos());
	if (pos > 0) //
	{
		wf_init_st.run_dir = WF_RUN_FEED_DIR;
	}
	else if (pos < 0) //
	{
		wf_init_st.run_dir = WF_RUN_COLLECT_DIR;
	}
	else
		wf_init_st.run_dir = WF_STOP;
	movePos(posOut2In(mot_run_state.posReq), spdOut2In(MOT_MSG_MOVE_POS, speed * motor.gear_ratio), -1);
}
void wire_feed_motor_stop(uint8_t stop_or_disable) //  送丝电机停止 1 放使能 0 位置模式锁定
{
	wire_feed_motor_clear_alert();
	wire_feed_motor_enable(1);
	wf_init_st.run_dir = WF_STOP;
	moveStop(stop_or_disable);
}
void wire_feed_motor_move_torque(int16_t torque) // 送丝电机力矩模式 torque = ma
{
	float fTorque = torque;
	wire_feed_motor_clear_alert();
	wire_feed_motor_enable(1);
	if (torque > 0) //
	{
		wf_init_st.run_dir = WF_RUN_FEED_DIR;
	}
	else if (torque < 0) //
	{
		wf_init_st.run_dir = WF_RUN_COLLECT_DIR;
	}
	else
		wf_init_st.run_dir = WF_STOP;
	fTorque = fTorque / 1000.0f;
	moveTorq(fTorque);
}

void wire_feed_motor_report_alert(uint8_t alert_id, uint8_t rank) // 送丝电机报警
{
	alertPush(alert_id, rank);
}
void wire_feed_motor_report_status(void) // 送丝电机上报状态
{
	uint32_t status = 0;
	status = wf_init_st.ctrl_main_status | (wf_init_st.ctrl_sub_status << 8);
	reportJkMotorStatus(status);
}
void wire_feed_motor_motor_move_is_stop_resh() // 重新刷新停止时间
{
	moveIsStop1(1000, 3, 1); // 2000
}
uint8_t wire_feed_motor_move_is_stop() // 送丝电机是否停止 1停止 0未停止
{
	if (moveIsStop1(1000, 3, 0) == 1)
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
uint8_t wire_feed_motor_ck_block(void) // 送丝电机是否堵转
{
	uint8_t ret = 0;
	if (wf_init_st.cur_current > wf_init_st.wf_work_st.work_feed_block_torque) // 如果当前电流大于送丝堵转电流
	{
		ret = 1;
	}
	return ret;
}
uint8_t wire_feed_motor_torque_over_speed(void) // 送丝电机 力矩模式下是否过速 1过速 0未过速
{
	return motTorqueOverSpd();
}
uint8_t wire_feed_motor_ck_rst_overtravel(void) // 送丝电机 检测复位最大容差 1过容差 0未过
{
	if (wire_feed_motor_abs(wire_feed_motor_get_cur_pos() - wf_reset_st.reset_begin_pos) > wf_reset_st.reset_limit_pos)
	{
		return 1;
	}
	else
		return 0;
}
uint8_t wire_feed_motor_ck_zero_signal_enable_overtravel(void) // 送丝电机 检测复位传感器最大容差 1过容差 0未过
{
	if (wire_feed_motor_abs(wire_feed_motor_get_cur_pos() - wf_reset_st.reset_signal_enable_begin_pos) > wf_reset_st.reset_signal_max_pos)
	{
		return 1;
	}
	else
		return 0;
}

uint8_t wire_feed_motor_move_pos_arr_flag(void) // 送丝电机 移动是否到位 1到位 0未到位
{
	return motGetMovePosArrFlag();
}
void wire_feed_motor_update_speed(uint16_t spd) // 更新速度
{
	motor_update_speed(spd);
}
void wire_feed_motor_report_current_from_pos(uint8_t dis_pos) // 根据位置上报电流
{
	wf_init_st.report_dis_pos = dis_pos;
	if (dis_pos == 0)
	{
		reportMotorCurrent(wire_feed_motor_get_cur_pos(), wire_feed_motor_get_current());
	}
	else
	{

		wf_init_st.report_cur_ptr = (wire_feed_motor_get_cur_pos() - 0) / dis_pos;
	}
}
void wire_feed_motor_run_report_current(void) // 实时上报位置和电流信息
{
	if (wf_init_st.report_dis_pos > 0)
	{
		uint16_t cur_ptr = (wire_feed_motor_get_cur_pos() - 0) / wf_init_st.report_dis_pos;
		if (cur_ptr != wf_init_st.report_cur_ptr)
		{
			reportMotorCurrent(cur_ptr * wf_init_st.report_dis_pos, wire_feed_motor_get_current());
			wf_init_st.report_cur_ptr = cur_ptr;
		}
	}
}
uint8_t wire_feed_motor_reset(uint8_t ck_pos_flag) // 复位流程 ck_pos_flag =1 检测流程标记
{
	uint8_t ret = 0;
	if (wf_init_st.exit_flag)
	{
		wf_reset_st.reset_state = WF_RESET_EXIT;
		wf_init_st.exit_flag = 0;
		wf_init_st.pause_flag = 0;
	}
	switch (wf_reset_st.reset_state)
	{
	case WF_RESET_INIT:					// 复位初始化
		if (wf_init_st.pCkZeroSignal()) // 检测零位传感器
		{
			wf_reset_st.reset_state = WF_RESET_TRANS_FEED_WIRE;
			wf_reset_st.reset_signal_enable_begin_pos = wire_feed_motor_get_cur_pos();
		}
		else
		{
			wf_reset_st.reset_state = WF_RESET_TRANS_COLLECT_WIRE;
			wf_reset_st.reset_begin_pos = wire_feed_motor_get_cur_pos();
		}
		break;
	case WF_RESET_TRANS_FEED_WIRE: // 复位送丝动作
		// wire_feed_motor_move_speed(wf_init_st.signal_enable_speed,wf_init_st.move_speed_torque) ;
		if (ck_pos_flag) // 检测流程用检测的速度
		{
			wire_feed_motor_move_speed(wf_init_st.wf_check_st.check_speed, wf_init_st.move_speed_torque);
		}
		else
		{
			wire_feed_motor_move_speed(wf_init_st.feed_speed, wf_init_st.move_speed_torque);
		}
		wf_reset_st.reset_state = WF_RESET_FEED_WIRE;
		break;
	case WF_RESET_TRANS_COLLECT_WIRE: // 复位收丝动作
		if (ck_pos_flag)			  // 检测流程用检测的速度
		{
			wire_feed_motor_move_speed(-wf_init_st.wf_check_st.check_speed, wf_init_st.move_speed_torque);
		}
		else
		{
			wire_feed_motor_move_speed(-wf_init_st.collection_speed, wf_init_st.move_speed_torque);
		}

		wf_reset_st.reset_state = WF_RESET_COLLECT_WIRE;
		wire_feed_motor_motor_move_is_stop_resh();
		break;
	case WF_RESET_FEED_WIRE:				 // 送丝
		if (wf_init_st.pCkZeroSignal() == 0) // 检测零位无效 开始收丝
		{
			wf_reset_st.reset_state = WF_RESET_TRANS_COLLECT_WIRE;
			wf_reset_st.reset_begin_pos = wire_feed_motor_get_cur_pos();
		}
		else // 检测零位有效
		{

			if (wf_init_st.pause_flag) // 暂停标记
			{
				wf_reset_st.reset_pause_state = WF_RESET_TRANS_FEED_WIRE;
				wf_reset_st.reset_state = WF_RESET_PAUSE;
			}
			if (wire_feed_motor_ck_zero_signal_enable_overtravel()) // 检测复位是否超行程
			{
				wire_feed_motor_stop(0);
				ret = 2;
				wf_reset_st.reset_state = WF_RESET_INIT;
				wire_feed_motor_report_alert(JM_MSG_ZERO_ERROR, 0); //
			}
		}
		//			wire_feed_motor_run_report_current() ;
		break;
	case WF_RESET_COLLECT_WIRE: // 收丝
								// 断电重启后钢丝再外面，这时读到的钢丝位置为负数，满足减速点速度，所以上电初始化加个标志，第一次按照收丝速度收丝，后面根据减速点进行收丝
		if (wf_init_st.zero_to_half_flag == 1)
		{
			wire_feed_motor_move_speed(-wf_init_st.collection_speed, wf_init_st.move_speed_torque); // 更新速度收丝速度速度
		}
		else
		{
			if (wf_init_st.wf_work_st.work_cur_pos < wf_init_st.wire_signal_save_zero_jt + wf_init_st.feed_zero_dec_pos) // 以第一次传感器亮的位置作为减速点
			{
				wire_feed_motor_update_speed(wf_init_st.feed_dec_speed); // 更新速度为减速速度
			}
			else
			{
				wire_feed_motor_move_speed(-wf_init_st.collection_speed, wf_init_st.move_speed_torque);
			}
		}

		if (wf_init_st.pCkZeroSignal()) // 检测零位信号 有效
		{

			/*                              wf_init_st.wire_signal_save_zero_no_jt =wire_feed_motor_get_cur_pos(); //非杰泰，传感器零位传感器亮的位置

										 if(wf_init_st.wire_motor_signal_set_type==2)  //为杰泰
										 {
											 wf_init_st.wire_signal_dec_save_zero = wf_init_st.wire_signal_save_zero_jt;
										 }
										 else  //为其它类型厂
										 {
											 wf_init_st.wire_signal_dec_save_zero = wf_init_st.wire_signal_save_zero_no_jt;
										 }
		   */
			wire_feed_motor_stop(0);
			wire_feed_motor_init_pos(); // 20250630零位传感器结构要改，这里不需要，直接设置为0
			wf_init_st.zero_to_half_flag = 0;

			if (wire_feed_motor_abs(wf_init_st.wire_signal_save_zero_jt - wire_feed_motor_get_cur_pos()) > wf_init_st.zero_signal_allow_pos)
			{

				wire_feed_motor_report_alert(JM_MSG_SET_ZERO_ERROR, 0); //
				wire_feed_motor_stop(0);
				ret = 2;
				wf_reset_st.reset_state = WF_RESET_INIT;
			}
			else
			{

				wire_feed_motor_init_pos();
				if (wf_init_st.wire_motor_signal_set_type == 2) // 为杰泰，只有杰泰才会把每次零位传感器亮的位置设置成第一次设置传感器亮的值
				{
					wire_feed_motor_set_cur_pos(wf_init_st.wire_signal_save_zero_jt); // 没次抽丝都把传感器亮的位置设置成设置零位时第一次传感器亮的位置
				}
				wf_reset_st.reset_state = WF_RESET_EXIT;
				wf_init_st.ctrl_main_status = WF_MAIN_STATUS_FINISH;
				wire_feed_motor_report_status();
			}
		}
		//			wire_feed_motor_run_report_current() ;
		else // 检测零位信号 无效
		{
			if (wf_init_st.pause_flag) // 暂停标记
			{
				wf_reset_st.reset_pause_state = WF_RESET_TRANS_COLLECT_WIRE;
				wf_reset_st.reset_state = WF_RESET_PAUSE;
			}
			if (wf_init_st.broken_signal) // 检测被堵传感器
			{
				wire_feed_motor_stop(0);
				wf_reset_st.reset_state = WF_RESET_EXIT;
				wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN, 0);
			}
			else
			{
				if (wire_feed_motor_move_is_stop()) // 检测电机是否堵转
				{
					wire_feed_motor_stop(0);
					wf_reset_st.reset_state = WF_RESET_EXIT;
					ret = 2;
					wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_OR_NO_CLAMP, 0);
				}
			}
			if (wire_feed_motor_ck_rst_overtravel()) // 检测复位总行程是否超容差
			{
				wire_feed_motor_stop(0);
				ret = 2;
				wf_reset_st.reset_state = WF_RESET_INIT;
				wire_feed_motor_report_alert(JM_MSG_ZERO_NO_ERROR, 0); // JM_MSG_ZERO_NO_ERROR JM_MSG_ZERO_ERROR
			}
		}
		break;

	case WF_RESET_PAUSE:		   // 暂停状态
		if (wf_init_st.pause_flag) // 暂停标记
		{
			wire_feed_motor_stop(0);
			if (wf_init_st.exit_flag) // 退出标记
			{
				wf_init_st.exit_flag = 0;
				wf_init_st.pause_flag = 0;
				wf_reset_st.reset_state = WF_RESET_EXIT;
			}
		}
		else // 恢复
		{
			wf_reset_st.reset_state = wf_reset_st.reset_pause_state; // 把暂停前的状态 赋值给当前状态
		}
		break;
	case WF_RESET_EXIT: // 退出状态

		wire_feed_motor_stop(0);
		wf_reset_st.reset_state = WF_RESET_INIT;
		ret = 1;
		break;
	}
	return ret;
}

uint8_t wire_feed_motor_para_setting(void) // 送丝电机设置流程
{
	uint8_t ret = 0;
	if (wf_init_st.exit_flag)
	{
		wf_init_st.setting_st.setting_state = WF_SETTING_EXIT;
		wf_init_st.exit_flag = 0;
		wf_init_st.pause_flag = 0;
	}
	switch (wf_init_st.setting_st.setting_state)
	{
	case WF_SETTING_INIT:
		switch (wf_init_st.sub_ctrl_type) // 控制子命令指令
		{
		case WF_SUB_CTRL_HANDLE_UP: // 送丝
			wire_feed_motor_move_speed(wf_init_st.handle_speed, wf_init_st.move_speed_torque);
			wf_init_st.setting_st.setting_state = WF_SETTING_MOVE_UP;
			break;
		case WF_SUB_CTRL_HANDLE_DOWN: // 收丝
			wire_feed_motor_move_speed(-wf_init_st.handle_speed, wf_init_st.move_speed_torque);
			wf_init_st.setting_st.setting_state = WF_SETTING_MOVE_DOWN;
			break;
		case WF_SUB_SAVE_ZERO: // 设置和保存零位
			wf_init_st.setting_st.setting_state = WF_SETTING_TRANS_SAVE_ZERO;
			break;
		case WF_SUB_MOVE_HALF_CIRCLE: // 移动半圈
			wf_init_st.setting_st.setting_state = WF_SETTING_TRANS_MOVE_HALF_CIRCLE;
			break;
		case WF_SUB_MOVE_ZERO_POS: // 移动到零位
			wf_init_st.setting_st.setting_state = WF_SETTING_TRANS_MOVE_ZERO_POS;
			break;
		case WF_SUB_SAVE_END_POS: // 记录末端位置
			wf_init_st.setting_st.setting_state = WF_SETTING_SAVE_END_POS;
			break;
		case WF_SUB_MOVE_ZERO_SIGNAL: // 移动到零位传感器
			wf_init_st.setting_st.setting_state = WF_SETTING_MOVE_ZERO_SIGNAL;
			break;
		case WF_SUB_IMPACK_ZERO: // 撞零
			wf_init_st.setting_st.setting_state = WF_SETTING_TRANS_IMPACK_ZERO;
			break;
		case WF_SUB_CTRL_STOP: // 停止
			wire_feed_motor_stop(0);
			wf_init_st.setting_st.setting_state = WF_SETTING_STOP;
			break;
		}
		if (wf_init_st.run_dir == WF_RUN_FEED_DIR && wf_init_st.end_signal) // 如果是送丝且末端传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_FEED_LIMIT, 1);
			wire_feed_motor_stop(0);
		}
		/*if(wf_init_st.run_dir==WF_RUN_COLLECT_DIR && wf_init_st.broken_signal) //如果是收丝且断线传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_COLLECT_LIMIT,1) ;
			wire_feed_motor_stop(0) ;
		}*/
		break;
	case WF_SETTING_MOVE_UP:
	case WF_SETTING_MOVE_DOWN:
	case WF_SETTING_STOP:
		wf_init_st.setting_st.setting_state = WF_SETTING_INIT;
		break;
	case WF_SETTING_TRANS_MOVE_HALF_CIRCLE: // 往回走半圈
		if (mot_run_state.motor_dir_rev == 1)
		{
			wire_feed_motor_move_relative_pos(posRatioIn2Out(-270.0f), wf_init_st.move_speed);
		}
		else
		{
			wire_feed_motor_move_relative_pos(posRatioIn2Out(270.0f), wf_init_st.move_speed);
		}
		wf_init_st.setting_st.setting_state = WF_SETTING_MOVE_HALF_CIRCLE;

		break;
	case WF_SETTING_MOVE_HALF_CIRCLE:

		if (wf_init_st.pCkZeroSignal())
		{

			wire_feed_motor_stop(0);
			wire_feed_motor_save_signal_zero_pos(); // 杰泰才会用这个,保存第一次传感器亮的位置
			wf_init_st.setting_st.zero_to_half_circle_pos = wire_feed_motor_get_cur_pos();
			//   wf_init_st.wire_signal_save_zero_no_jt =wire_feed_motor_get_cur_pos(); //非杰泰，传感器零位传感器亮的位置
			wf_init_st.setting_st.setting_state = WF_SETTING_INIT;
			// 完成流程
			ret = 2;
			wf_init_st.ctrl_main_status = WF_MAIN_STATUS_FINISH;
			wire_feed_motor_report_status();
		}
		else
		{
			if (wire_feed_motor_move_pos_arr_flag()) // 电机目标位置已到
			{

				wire_feed_motor_save_signal_zero_pos();
				wf_init_st.setting_st.zero_to_half_circle_pos = wire_feed_motor_get_cur_pos();
				//   wf_init_st.wire_signal_save_zero_no_jt =wire_feed_motor_get_cur_pos(); //非杰泰，传感器零位传感器亮的位置
				wf_init_st.setting_st.setting_state = WF_SETTING_INIT;
				// 完成流程
				ret = 2;
				wf_init_st.ctrl_main_status = WF_MAIN_STATUS_FINISH;
				wire_feed_motor_report_status();
			}
		}
		/*
			if(wire_feed_motor_move_pos_arr_flag())//电机目标位置已到
{
		   // wire_feed_motor_stop(0) ;
				wf_init_st.setting_st.zero_to_half_circle_pos = wire_feed_motor_get_cur_pos();
	wf_init_st.setting_st.setting_state = WF_SETTING_INIT ;
   //完成流程
	ret =2 ;
 wf_init_st.ctrl_main_status = WF_MAIN_STATUS_FINISH ;
 wire_feed_motor_report_status() ;
			}
			*/
		break;
	case WF_SETTING_TRANS_SAVE_ZERO: // 保存零位
		wire_feed_motor_set_cur_pos(0);
		wire_feed_motor_save_zero();
		wf_init_st.setting_st.setting_state = WF_SETTING_TRANS_MOVE_HALF_CIRCLE;
		break;
	case WF_SETTING_TRANS_IMPACK_ZERO: // 撞零
		wire_feed_motor_move_relative_pos(posRatioIn2Out(-360.0f), 5);
		wf_init_st.setting_st.setting_state = WF_SETTING_SAVE_ZERO;
		break;
	case WF_SETTING_IMPACK_ZERO:				 // 撞零中
		if (wire_feed_motor_move_pos_arr_flag()) // 电机目标位置已到
		{

			wf_init_st.setting_st.setting_state = WF_SETTING_INIT;
			wire_feed_motor_report_alert(JM_MSG_SET_ZERO_TIME_OUT, 0);
		}
		else
		{
			if (wire_feed_motor_ck_block())
			{
				wire_feed_motor_stop(0);
				wire_feed_motor_set_cur_pos(0);
				wire_feed_motor_save_zero();
				wf_init_st.setting_st.setting_state = WF_SETTING_TRANS_MOVE_HALF_CIRCLE;

				//	wf_init_st.setting_st.setting_state = WF_SETTING_INIT ;
			}
		}
		break;
	case WF_SETTING_TRANS_MOVE_ZERO_POS: // 提交移动到零位指令
		wire_feed_motor_move_pos(0, wf_init_st.move_speed);
		wf_init_st.setting_st.setting_state = WF_SETTING_MOVE_ZERO_POS;
		break;
	case WF_SETTING_MOVE_ZERO_POS:
		if (wf_init_st.pause_flag) // 暂停标记
		{
			wf_init_st.setting_st.setting_pause_state = WF_SETTING_TRANS_MOVE_ZERO_POS;
			wf_init_st.setting_st.setting_state = WF_SETTING_PAUSE;
		}
		else
		{
			if (wire_feed_motor_move_pos_arr_flag()) // 电机目标位置已到
			{
				wf_init_st.setting_st.setting_state = WF_SETTING_INIT;
			}
		}
		break;
	case WF_SETTING_MOVE_ZERO_SIGNAL: // 移动到零位传感器
		if (wire_feed_motor_reset(0))
		{
			wf_init_st.setting_st.setting_state = WF_SETTING_INIT;
		}
		break;
	case WF_SETTING_SAVE_END_POS: // 保存末端位置
	{
		wire_feed_motor_save_end_pos();
		wf_init_st.setting_st.setting_state = WF_SETTING_INIT;
	}

	break;
	case WF_SETTING_PAUSE:
		if (wf_init_st.pause_flag) // 暂停标记
		{
			wire_feed_motor_stop(0);
			if (wf_init_st.exit_flag) // 退出标记
			{
				wf_init_st.exit_flag = 0;
				wf_init_st.pause_flag = 0;
				wf_init_st.setting_st.setting_state = WF_SETTING_EXIT;
			}
		}
		else // 恢复
		{
			wf_init_st.setting_st.setting_state = wf_init_st.setting_st.setting_pause_state; // 把暂停前的状态 赋值给当前状态
		}
		break;
	case WF_SETTING_EXIT: // 退出状态
		wire_feed_motor_stop(0);
		wf_init_st.setting_st.setting_state = WF_SETTING_INIT;
		ret = 1;
		break;
	}

	return ret;
}

uint8_t wire_feed_motor_ck_move_end_pos(void)
{
	if (wf_init_st.wf_work_st.work_target_pos >= wf_init_st.end_pos)
	{
		if (wf_init_st.end_signal_stat == SIGNAL_NO_USE) // 不使用
		{
			wf_init_st.move_to_end_pos_flag = 0;
		}
		else
			wf_init_st.move_to_end_pos_flag = 1;
	}
	else
	{
		wf_init_st.move_to_end_pos_flag = 0;
	}
	return wf_init_st.move_to_end_pos_flag;
}

// 检测流程
uint8_t wire_feed_motor_check(void)
{
	uint8_t ret = 0;
	if (wf_init_st.exit_flag)
	{
		wf_init_st.wf_check_st.check_state = WF_CHECK_EXIT;
		wf_init_st.exit_flag = 0;
		wf_init_st.pause_flag = 0;
	}
	if (wire_feed_end_write_flag() == 0)
	{
		wire_feed_motor_stop(0);
		wf_init_st.wf_check_st.check_state = WF_CHECK_INIT;
		wire_feed_motor_report_alert(JM_MSG_NO_SET_END_POS, 0);
		ret = 1;
		return ret;
	}
	switch (wf_init_st.wf_check_st.check_state)
	{
	case WF_CHECK_INIT: // 检测初始化
		wf_init_st.wf_check_st.check_state = WF_CHECK_RESET;
		break;
	case WF_CHECK_TRANS_FEED_WIRE:						 // 提交送丝动作
		if (wf_init_st.end_signal_stat == SIGNAL_NO_USE) // 不使用
			wf_init_st.wf_work_st.work_target_pos = wf_init_st.end_pos;
		else
			wf_init_st.wf_work_st.work_target_pos = wf_init_st.end_pos + wf_init_st.end_allowance_pos;
		wire_feed_motor_move_pos(wf_init_st.wf_work_st.work_target_pos, wf_init_st.wf_check_st.check_speed);
		wf_init_st.wf_check_st.check_state = WF_CHECK_FEED_WIRE;
		break;
	case WF_CHECK_FEED_WIRE: // 送丝
		//			wire_feed_motor_run_report_current();

		if (wf_init_st.pause_flag) // 暂停标记
		{
			wf_init_st.wf_check_st.check_pause_state = WF_CHECK_TRANS_FEED_WIRE;
			wf_init_st.wf_check_st.check_state = WF_CHECK_PAUSE;
		}
		// 把末端减速点wf_init_st.feed_dec_pos变量换成，输入多少数字，到时候在末端位置wf_init_st.end_pos减去这个变量就可以
		if (wf_init_st.wf_work_st.work_cur_pos > wf_init_st.end_pos - wf_init_st.feed_dec_pos) // 如果当前送丝位置大于减速位置
		{
			wire_feed_motor_update_speed(wf_init_st.feed_dec_speed); // 更新速度为减速速度
		}

		if (wf_init_st.pCkEndSignal()) // 检测末端传感器有效
		{
			wire_feed_motor_stop(0);
			if (wf_init_st.wf_work_st.work_cur_pos >= wf_init_st.end_pos - wf_init_st.end_allowance_pos) // 如果已经到达末端
			{
				wf_init_st.wf_check_st.check_state = WF_CHECK_RESET;
			}
			else
			{
				wf_init_st.wf_check_st.check_state = WF_CHECK_EXIT;
				wire_feed_motor_report_alert(JM_MSG_ZERO_SET_ERR_OR_SIGNAL_ERR, 0);
			}
		}
		else // 末端传感器无效
		{
			if (wf_init_st.broken_signal) // 检测被堵传感器
			{
				wire_feed_motor_stop(0);
				wf_init_st.wf_check_st.check_state = WF_CHECK_EXIT;
				wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_OR_NO_CLAMP, 0);
			}
			else
			{
				if (wire_feed_motor_ck_block()) // 检测钢丝是否被堵
				{

					wire_feed_motor_stop(0);
					wf_init_st.wf_check_st.check_state = WF_CHECK_EXIT;
					wire_feed_motor_report_alert(JM_MSG_FEED_BLOCK, 0);
				}
				else
				{

					if (wire_feed_motor_move_pos_arr_flag()) // 电机目标位置已到说明传感器无效或位置设置错误
					{
						if (wf_init_st.end_signal_stat == SIGNAL_NO_USE) // 不使用
						{
							wf_init_st.wf_check_st.check_state = WF_CHECK_RESET;
						}
						else
						{
							ret = 1;
							wf_init_st.wf_check_st.check_state = WF_CHECK_INIT;
							wire_feed_motor_report_alert(JM_MSG_ZERO_SET_ERR_OR_SIGNAL_ERR, 0);
						}
					}
				}
			}
		}
		break;
	case WF_CHECK_RESET: // 复位
	{
		uint8_t ret_reset = wire_feed_motor_reset(1);
		if (ret_reset == 1)
		{
			wf_init_st.wf_check_st.check_state = WF_CHECK_TRANS_FEED_WIRE;
		}
		else if (ret_reset == 2)
		{
			wire_feed_motor_stop(0);
			wf_init_st.wf_check_st.check_state = WF_CHECK_EXIT;
		}
	}
	break;
	case WF_CHECK_PAUSE:		   // 暂停
		if (wf_init_st.pause_flag) // 暂停标记
		{
			wire_feed_motor_stop(0);
			if (wf_init_st.exit_flag) // 退出标记
			{
				wf_init_st.exit_flag = 0;
				wf_init_st.wf_check_st.check_state = WF_CHECK_EXIT;
			}
		}
		else // 恢复
		{
			wf_init_st.wf_check_st.check_state = wf_init_st.wf_check_st.check_pause_state; // 把暂停前的状态 赋值给当前状态
		}
		break;
	case WF_CHECK_EXIT: // 退出
		wire_feed_motor_stop(0);
		wf_init_st.wf_check_st.check_state = WF_CHECK_INIT;
		ret = 1;
		break;
	}
	return ret;
}

// 工作流程
uint8_t wire_feed_motor_work(void)
{
	uint8_t ret = 0;
	static uint8_t retry_cnt = 0;
	static uint8_t break_retry_cnt = 0;
	if (wf_init_st.exit_flag)
	{
		wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
		wf_init_st.exit_flag = 0;
		wf_init_st.pause_flag = 0;
	}

	wire_feed_motor_ck_move_end_pos();

	switch (wf_init_st.wf_work_st.work_state)
	{

	case WF_WORK_INIT: // 复位初始化
		retry_cnt = 0;
		break_retry_cnt = 0;
		if (wf_init_st.wf_work_st.work_target_pos > wf_init_st.wf_work_st.work_cur_pos) // 如果目标位置大于当前位置 则送丝
		{
			wf_init_st.wf_work_st.work_state = WF_WORK_TRANS_FEED_WIRE;
		}
		else if (wf_init_st.wf_work_st.work_target_pos < wf_init_st.wf_work_st.work_cur_pos) // 如果目标位置小于当前位置 则收丝
		{
			wf_init_st.wf_work_st.work_state = WF_WORK_TRANS_COLLECT_WIRE;
		}
		else // 不用动 完成
		{

			ret = 1;
		}
		break;
	case WF_WORK_TRANS_FEED_WIRE: // 送丝动作
		if (wf_init_st.move_to_end_pos_flag == 1)
		{
			wire_feed_motor_move_pos(wf_init_st.end_pos + wf_init_st.end_allowance_pos, wf_init_st.feed_speed);
		}
		else
			wire_feed_motor_move_pos(wf_init_st.wf_work_st.work_target_pos, wf_init_st.feed_speed);
		wf_init_st.wf_work_st.work_state = WF_WORK_FEED_WIRE;
		break;
	case WF_WORK_TRANS_COLLECT_WIRE: // 收丝动作
		wire_feed_motor_move_pos(wf_init_st.wf_work_st.work_target_pos, wf_init_st.collection_speed);
		wire_feed_motor_motor_move_is_stop_resh();
		wf_init_st.wf_work_st.work_state = WF_WORK_COLLECT_WIRE;
		break;
	case WF_WORK_FEED_WIRE: // 送丝
		//			wire_feed_motor_run_report_current();
		if (wf_init_st.pause_flag) // 暂停标记
		{
			wf_init_st.wf_work_st.work_pause_state = WF_WORK_TRANS_FEED_WIRE;
			wf_init_st.wf_work_st.work_state = WF_WORK_PAUSE;
		}
		// 把末端减速点wf_init_st.feed_dec_pos变量换成，输入多少数字，到时候在末端位置wf_init_st.end_pos减去这个变量就可以
		if (wf_init_st.wf_work_st.work_cur_pos > wf_init_st.end_pos - wf_init_st.feed_dec_pos) // 如果当前送丝位置大于减速位置
		{
			wire_feed_motor_update_speed(wf_init_st.feed_dec_speed); // 更新速度为减速速度
		}
		if (wf_init_st.pCkEndSignal()) // 检测末端传感器 有效
		{
			wire_feed_motor_stop(0);
			wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
			if (wf_init_st.move_to_end_pos_flag) // 移动到最末端位置 = 第一针
			{
				if (wf_init_st.wf_work_st.work_cur_pos >= wf_init_st.end_pos - wf_init_st.end_allowance_pos) // 如果已经到达末端
				{
					// 完成流程
					/*wf_init_st.ctrl_main_status = WF_MAIN_STATUS_FINISH ;
					wire_feed_motor_report_status() ;*/
					/****最后一针到位回退****/

					// 20250703,测试当钢丝松了，没有到设置的末端值，在加上容差值，这时末端传感器亮，那么就是亮时刻的值减去回退值
					// wf_init_st.wf_work_st.work_target_pos = wf_init_st.wf_work_st.work_target_pos - wf_init_st.wire_end_back_pos ;
					wf_init_st.wf_work_st.work_target_pos = wf_init_st.wf_work_st.work_cur_pos - wf_init_st.wire_end_back_pos;
					wf_init_st.wf_work_st.work_state = WF_WORK_TRANS_COLLECT_WIRE;
					/****最后一针到位回退****/
				}
				else
				{

					wire_feed_motor_report_alert(JM_MSG_ZERO_SET_ERR_OR_SIGNAL_ERR, 0);
				}
			}
			else
			{
				wire_feed_motor_report_alert(JM_MSG_ZERO_SET_ERR_OR_SIGNAL_ERR, 0); // 末端传感器错误或者零位设置错误
			}
		}
		else // 检测末端传感器 无效
		{
			if (wf_init_st.broken_signal) // 检测被堵传感器
			{

				if (wf_init_st.wire_broken_retry_factory_type == 2)
				{
					if (break_retry_cnt > wf_init_st.wf_work_st.work_feed_block_retry_num) // 如果重试次数大于
					{
						wire_feed_motor_stop(0);
						wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
						wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_NO_ERROR, 0); // JM_MSG_WIRE_BROKEN_OR_NO_CLAMP
					}
					else
					{
						wire_feed_motor_stop(0);
						wf_init_st.wf_work_st.work_state = WF_WORK_TRANS_BACK_WIRE;
						break_retry_cnt++;
					}
				}
				else
				{
					//  wire_feed_motor_stop(0) ;
					wire_feed_motor_enable(0);
					wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
					wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_NO_ERROR, 0); // JM_MSG_WIRE_BROKEN_OR_NO_CLAMP
				}
			}
			else
			{
				if (wire_feed_motor_ck_block()) // 检测钢丝是否被堵
				{
					if (retry_cnt > wf_init_st.wf_work_st.work_feed_block_retry_num) // 如果重试次数大于
					{
						wire_feed_motor_stop(0);
						wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
						wire_feed_motor_report_alert(JM_MSG_FEED_BLOCK, 0);
					}
					else
					{
						wire_feed_motor_stop(0);
						wf_init_st.wf_work_st.work_state = WF_WORK_TRANS_BACK_WIRE;
						retry_cnt++;
					}
				}
				else
				{

					if (wire_feed_motor_move_pos_arr_flag()) // 电机目标位置已到
					{
						if (wf_init_st.move_to_end_pos_flag == 1) // 末端标记
						{
							ret = 1;
							wire_feed_motor_report_alert(JM_MSG_ZERO_SET_ERR_OR_SIGNAL_ERR, 0);
							wf_init_st.wf_work_st.work_state = WF_WORK_INIT;
						}
						else
						{
							// 完成流程
							wf_init_st.ctrl_main_status = WF_MAIN_STATUS_FINISH;
							wire_feed_motor_report_status();
							ret = 1;
							wf_init_st.wf_work_st.work_state = WF_WORK_INIT;
						}
					}
				}
			}
		}
		break;
	case WF_WORK_COLLECT_WIRE: // 收丝
		//			wire_feed_motor_run_report_current();
		if (wf_init_st.pause_flag) // 暂停标记
		{
			wf_init_st.wf_work_st.work_pause_state = WF_WORK_TRANS_COLLECT_WIRE;
			wf_init_st.wf_work_st.work_state = WF_WORK_PAUSE;
		}
		if (wf_init_st.wf_work_st.work_cur_pos < wf_init_st.wire_signal_dec_save_zero + wf_init_st.feed_dec_pos) // 如果当前送丝位置大于减速位置
		{
			wire_feed_motor_update_speed(wf_init_st.feed_dec_speed); // 更新速度为减速速度
		}

		if (wf_init_st.broken_signal) // 检测被堵传感器
		{
			wire_feed_motor_stop(0);
			wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
			wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN, 0);
		}
		else
		{
			if (wire_feed_motor_move_is_stop()) // 检测电机是否堵转
			{
				wire_feed_motor_stop(0);
				wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
				wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_OR_NO_CLAMP, 0);
			}
			else
			{
				if (wire_feed_motor_move_pos_arr_flag()) // 电机目标位置已到
				{
					// 完成流程
					wf_init_st.ctrl_main_status = WF_MAIN_STATUS_FINISH;
					wire_feed_motor_report_status();
					ret = 1;
					wf_init_st.wf_work_st.work_state = WF_WORK_INIT;
				}
			}
		}
		break;
	case WF_WORK_BACK_WIRE: // 收丝回退
		if (wf_init_st.wire_broken_retry_factory_type == 2)
		{
			if (wire_feed_motor_move_is_stop()) // 检测电机是否堵转
			{
				wire_feed_motor_stop(0);
				wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
				wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_OR_NO_CLAMP, 0);
			}
			else
			{
				if (wire_feed_motor_move_pos_arr_flag()) // 已经到位
				{
					wf_init_st.wf_work_st.work_state = WF_WORK_TRANS_FEED_WIRE;
				}
			}
		}
		else
		{
			if (wf_init_st.broken_signal) // 检测被堵传感器
			{
				// if(wf_init_st.wf_work_st.work_cur_back_pos-100 >wf_init_st.wf_work_st.work_cur_pos) //在回退100个编码值后再检测,因为在这20个编码值内还是满足断丝和大电流的报警
				//  {
				wire_feed_motor_stop(0);
				wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
				wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN, 0);
				//  }
			}
			else
			{
				if (wire_feed_motor_move_is_stop()) // 检测电机是否堵转
				{
					wire_feed_motor_stop(0);
					wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
					wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_OR_NO_CLAMP, 0);
				}
				else
				{
					if (wire_feed_motor_move_pos_arr_flag()) // 已经到位
					{
						wf_init_st.wf_work_st.work_state = WF_WORK_TRANS_FEED_WIRE;
					}
				}
			}
		}
		break;
	case WF_WORK_TRANS_BACK_WIRE: // 送丝堵转回退
		wf_init_st.wf_work_st.work_cur_back_pos = wf_init_st.wf_work_st.work_cur_pos;
		wire_feed_motor_move_pos(wf_init_st.wf_work_st.work_cur_pos - wf_init_st.wf_work_st.work_back_pos, wf_init_st.collection_speed);
		wf_init_st.wf_work_st.work_state = WF_WORK_BACK_WIRE;
		wire_feed_motor_motor_move_is_stop_resh();
		break;
	case WF_WORK_PAUSE:			   // 暂停状态
		if (wf_init_st.pause_flag) // 暂停标记
		{
			wire_feed_motor_stop(0);
			if (wf_init_st.exit_flag) // 退出标记
			{
				wf_init_st.exit_flag = 0;
				wf_init_st.wf_work_st.work_state = WF_WORK_EXIT;
			}
		}
		else // 恢复
		{
			wf_init_st.wf_work_st.work_state = wf_init_st.wf_work_st.work_pause_state; // 把暂停前的状态 赋值给当前状态
		}
		break;
	case WF_WORK_EXIT: // 退出状态

		wire_feed_motor_stop(0);
		wf_init_st.wf_work_st.work_state = WF_RESET_INIT;
		ret = 1;
		break;
	}
	return ret;
}

void wire_feed_motor_ctrl(WF_CTRL_TYPE type, WF_SUB_CTRL_TYPE sub_type, int16_t pos) // 控制命令
{
	if (type == WF_CTRL_PAUSE) // 暂停
	{
		if (wf_init_st.state > 0)
		{
			wf_init_st.ctrl_type = type;
			wf_init_st.sub_ctrl_type = sub_type;
		}
	}
	else
	{
		wf_init_st.ctrl_type = type;
		wf_init_st.sub_ctrl_type = sub_type;
		if (type == WF_CTRL_WORK || WF_CTRL_CHECK == type)
			wf_init_st.wf_work_st.work_target_pos = pos;
	}
	if (sub_type == WF_SUB_CTRL_HANDLE_UP || WF_SUB_CTRL_HANDLE_DOWN == sub_type)
	{
		if (pos == 0)
			wf_init_st.handle_speed = wf_init_st.move_speed;
		else
			wf_init_st.handle_speed = pos;
	}
}
void wire_feed_motor_task_loop(void) // 送丝电机主流程
{
	static uint32_t cur_tick = 0;
	static uint8_t main_state =0 ;
	static uint8_t sub_state =0 ;
	wf_init_st.zero_signal = wf_init_st.pCkZeroSignal();
	wf_init_st.end_signal = wf_init_st.pCkEndSignal();
	wf_init_st.broken_signal = wf_init_st.pCkBrokenSignal();
	wf_init_st.wf_work_st.work_cur_pos = wire_feed_motor_get_cur_pos();
	wire_feed_signal_set_def_state();
	wire_feed_motor_run_report_current();
	if (expireTicks(cur_tick, 2))
	{
		av_set_data(1, wire_feed_motor_get_cur_current());
		wf_init_st.cur_current = av_get_data(1);
		cur_tick = getTicks();
	}

	if (wf_init_st.ctrl_type == WF_CTRL_PAUSE) // 暂停
	{
		wf_init_st.pause_flag = 1;
		wf_init_st.ctrl_type = WF_CTRL_NULL;
	}
	else if (wf_init_st.ctrl_type == WF_CTRL_RECOVER) // 恢复
	{
		wf_init_st.pause_flag = 0;
		wf_init_st.ctrl_type = WF_CTRL_NULL;
	}
	else if (wf_init_st.ctrl_type == WF_CTRL_EXIT) // 退出
	{
		wf_init_st.exit_flag = 1;
		wf_init_st.ctrl_type = WF_CTRL_NULL;
	}
	/*if(wf_init_st.state >0)
	{
		if(wf_init_st.run_dir==WF_RUN_FEED_DIR && wf_init_st.end_signal) //如果是送丝且末端传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_FEED_LIMIT) ;
			wire_feed_motor_stop(0) ;
			wf_init_st.state = WF_STATE_INIT ;

		}
		if(wf_init_st.run_dir==WF_RUN_COLLECT_DIR && wf_init_st.broken_signal) //如果是收丝且断线传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_COLLECT_LIMIT) ;
			wire_feed_motor_stop(0) ;
			wf_init_st.state = WF_STATE_INIT ;
		}
	}*/
	wf_init_st.ctrl_main_status = wf_init_st.state;
	switch (wf_init_st.state)
	{
	case WF_STATE_INIT:				  //  初始状态
		switch (wf_init_st.ctrl_type) // 控制指令
		{

		case WF_CTRL_RESET: // 进入复位
			wf_init_st.state = WF_STATE_RESET;
			break;
		case WF_CTRL_CHECK: // 进入检测
			wf_init_st.state = WF_STATE_CHECK;
			break;
		case WF_CTRL_WORK: // 进入工作
			wf_init_st.state = WF_STATE_WORK;
			break;
		case WF_CTRL_SET_PARA: // 设置参数
			wf_init_st.state = WF_STATE_PARA_SET;
			break;
		default:
			if (wf_init_st.sub_ctrl_type == WF_SUB_CTRL_HANDLE_UP) // 手动上升
			{
				wf_init_st.state = WF_STATE_HANDLE_UP;

				wire_feed_motor_move_speed(wf_init_st.move_speed, wf_init_st.move_speed_torque);
			}
			else if (wf_init_st.sub_ctrl_type == WF_SUB_CTRL_HANDLE_DOWN) // 手动下降
			{
				wf_init_st.state = WF_STATE_HANDLE_DOWN;

				wire_feed_motor_move_speed(-wf_init_st.move_speed, wf_init_st.move_speed_torque);
				wire_feed_motor_motor_move_is_stop_resh();
			}
			break;
		}

		break;
	case WF_STATE_RESET: // 复位中
		if (wire_feed_motor_reset(0))
		{
			wf_init_st.state = WF_STATE_INIT;
		}
		wf_init_st.ctrl_sub_status = wf_reset_st.reset_state;
		wf_init_st.sub_ctrl_type = WF_SUB_CTRL_NULL;
		break;
	case WF_STATE_PARA_SET: // 设置参数中
		if (wire_feed_motor_para_setting())
		{
			wf_init_st.state = WF_STATE_INIT;
		}
		wf_init_st.ctrl_sub_status = 0;
		wf_init_st.sub_ctrl_type = WF_SUB_CTRL_NULL;
		break;
	case WF_STATE_CHECK: // 检测流程中
		if (wire_feed_motor_check())
		{
			wf_init_st.state = WF_STATE_INIT;
		}
		wf_init_st.ctrl_sub_status = wf_init_st.wf_check_st.check_state;
		wf_init_st.sub_ctrl_type = WF_SUB_CTRL_NULL;
		break;
	case WF_STATE_WORK: // 工作流程中
		if (wire_feed_motor_work())
		{
			wf_init_st.state = WF_STATE_INIT;
		}
		wf_init_st.ctrl_sub_status = wf_init_st.wf_work_st.work_state;
		wf_init_st.sub_ctrl_type = WF_SUB_CTRL_NULL;
		break;
	case WF_STATE_HANDLE_UP: // 手动送丝
		if (wf_init_st.sub_ctrl_type == WF_SUB_CTRL_STOP)
		{
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		else if (wf_init_st.sub_ctrl_type == WF_SUB_CTRL_HANDLE_DOWN)
		{
			wf_init_st.state = WF_STATE_HANDLE_DOWN;

			wire_feed_motor_move_speed(-wf_init_st.move_speed, wf_init_st.move_speed_torque);
		}
		if (wf_init_st.run_dir == WF_RUN_FEED_DIR && wf_init_st.end_signal) // 如果是送丝且末端传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_FEED_LIMIT, 1);
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		if (wf_init_st.run_dir == WF_RUN_FEED_DIR && wf_init_st.broken_signal) // 如果是送丝且断线传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_NO_ERROR, 1);
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		if (wf_init_st.exit_flag)
		{
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		wf_init_st.sub_ctrl_type = WF_SUB_CTRL_NULL;
		break;
	case WF_STATE_HANDLE_DOWN: // 手动收丝
		if (wf_init_st.sub_ctrl_type == WF_SUB_CTRL_STOP)
		{
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		if (wf_init_st.sub_ctrl_type == WF_SUB_CTRL_HANDLE_UP)
		{
			wf_init_st.state = WF_STATE_HANDLE_UP;

			wire_feed_motor_move_speed(wf_init_st.move_speed, wf_init_st.move_speed_torque);
		}
		if (wf_init_st.run_dir == WF_RUN_FEED_DIR && wf_init_st.end_signal) // 如果是送丝且末端传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_FEED_LIMIT, 1);
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		if (wf_init_st.run_dir == WF_RUN_COLLECT_DIR && wf_init_st.broken_signal) // 如果是收丝且断线传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN, 1); // JM_MSG_WIRE_COLLECT_LIMIT
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		if (wf_init_st.run_dir == WF_RUN_COLLECT_DIR && wf_init_st.zero_signal) // 如果是收丝且零位传感器有效
		{
			wire_feed_motor_report_alert(JM_MSG_WIRE_COLLECT_LIMIT, 1);
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		if (wf_init_st.exit_flag)
		{
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
		}
		if (wire_feed_motor_move_is_stop()) // 检测电机是否堵转
		{
			wire_feed_motor_stop(0);
			wf_init_st.state = WF_STATE_INIT;
			wire_feed_motor_report_alert(JM_MSG_WIRE_BROKEN_OR_NO_CLAMP, 0);
		}
		wf_init_st.sub_ctrl_type = WF_SUB_CTRL_NULL;
		break;
	}
	#ifdef DEBUG_CMD_PRINT
	if((main_state!=wf_init_st.ctrl_main_status) || (sub_state!=wf_init_st.ctrl_sub_status))	
	{
		reportDebug(1,wf_init_st.ctrl_main_status,wf_init_st.ctrl_sub_status) ;
		main_state = wf_init_st.ctrl_main_status ;
		sub_state = wf_init_st.ctrl_sub_status ;
		
	}		
	#endif
	wf_init_st.exit_flag = 0;
	wf_init_st.ctrl_type = WF_CTRL_NULL;
}
void wire_feed_motor_set_target_pos(uint32_t pos) // 设置升降电机目标位置
{
	wf_init_st.wf_work_st.work_target_pos = pos;
}
#endif