
#include "stdint.h"
#include "motor_round_run_ctrl.h"
#include "motor.h"
#include "servo_can.h"
ROUND_RUN_CTRL_ST round_run_ctrl_st;
ROUND_RUN_SET_ST round_run_set_st;
ROUND_RUN_REPORT_ST round_run_report_st;
ROUND_RUN_CHANGE_DIR_ST round_run_change_dir_st;
void round_run_ctrl_init(void) // 初始化
{
	round_run_ctrl_st.run_dir = RIGHT_RUN_DIR;
	round_run_ctrl_st.ctrl_flag = 0;
	round_run_ctrl_st.state = RR_INIT;
	round_run_ctrl_st.max_v = 2000;
	round_run_ctrl_st.left_spd = 2000;
	round_run_ctrl_st.right_spd = 2000;
	round_run_ctrl_st.left_pos = 0;
	round_run_ctrl_st.right_pos = 50000;
	round_run_ctrl_st.stop_mode = ROUND_RUN_EMERGENCY_STOP;
	round_run_ctrl_st.arr_pos = 100;
	round_run_ctrl_st.left_to_right_flag = 1;
	round_run_ctrl_st.right_to_left_flag = 1;
	round_run_ctrl_st.report_offpos = 0;
	round_run_ctrl_st.report_stop_req_flag = 0;
	round_run_ctrl_st.right_to_left_pause_flag = 0;
	round_run_ctrl_st.left_to_right_pause_flag = 0;
	round_run_ctrl_st.slow_spd = 100 ;
}
/*fun:到目标为止采用s型曲线
target_pos 目标为止
v0 初始速度
vMax 最大速度
*/
void round_run_ctrl_sramp_pos(int32_t target_pos, uint16_t v0, uint16_t vMax)
{
	moveSRampPos(posOut2In(target_pos), spdOut2In(MOT_MSG_MOVE_POS, vMax), -1);
}
/*fun:设置停止的三种模式
stop_mode 设置停止的三种模式
*/
void round_run_ctrl_pause(uint8_t stop_mode)
{
	round_run_ctrl_st.stop_mode = stop_mode;
	round_run_ctrl_st.pause_flag = 1;
}
/*fun:执行停止动作
 */
void round_run_ctrl_stop(void)
{
	if (round_run_ctrl_st.stop_mode == ROUND_RUN_EMERGENCY_STOP) // 急停
	{
		moveStop(0);
	}
	else if (round_run_ctrl_st.stop_mode == ROUND_RUN_POWER_STOP) // 发电停
	{
		moveStop(0);
	}
	else // 减速停
	{
		moveStop(0);
	}
}
/*fun:设置运行时的参数
target_pos: 左右运行的目标为止
speed : 运行时的最大速度
dir : 运行方向 右行RIGHT_RUN_DIR 左行 LEFT_RUN_DIR
*/
void round_run_ctrl_set_run_target(int32_t target_pos, int16_t ctrl_data) // 设置控制数据
{
	round_run_set_st.set_pos = target_pos;
	round_run_set_st.set_ctrl_st.set_ctrl_data = ctrl_data;
	if (round_run_set_st.set_ctrl_st.set_ctrl_bit_st.set_dir == RIGHT_RUN_DIR)
	{
		round_run_ctrl_st.right_pos = round_run_set_st.set_pos;
		round_run_ctrl_st.right_spd = round_run_set_st.set_ctrl_st.set_ctrl_bit_st.set_speed;
		round_run_ctrl_st.left_to_right_flag = 1;
	}
	else
	{
		round_run_ctrl_st.left_pos = round_run_set_st.set_pos;
		round_run_ctrl_st.left_spd = round_run_set_st.set_ctrl_st.set_ctrl_bit_st.set_speed;
		round_run_ctrl_st.right_to_left_flag = 1;
	}
}
int32_t round_run_ctrl_get_cur_pos(void)
{
	return posIn2Out(getCurPos());
}
int16_t round_run_ctrl_get_cur_speed(void)
{
	return (int16_t)(getCurSpd(1));
}
uint8_t round_run_ctrl_is_stop(void) // 判断电机是否停止
{
	return motorIsStop(100, 3, 0);
}
uint8_t round_run_ctrl_get_arr(void) // 判断电机是否到位
{
	return 1;
}
void round_run_ctrl_set_report_offpos(int32_t offpos) // 设置上报的偏移位置
{
	round_run_ctrl_st.report_offpos = offpos;
	round_run_ctrl_st.report_cur_pos = round_run_ctrl_get_cur_pos();
	if (offpos != 0)
	{
		round_run_report_st.report_info_st.report_bit_st.cur_speed = round_run_ctrl_get_cur_speed();
		round_run_report_st.cur_pos = round_run_ctrl_get_cur_pos();
		round_run_report_st.report_info_st.report_bit_st.report_ack = RR_REPOT_OFFSET_POS_ACK;
		round_run_ctrl_report(round_run_report_st.cur_pos, round_run_report_st.report_info_st.report_data, 0);
	}
}

void round_run_ctrl_report(int32_t cur_pos, int16_t ctrl_ack_data16, uint8_t change_dir_event)
{
	uint8_t data[8] = {0};
	if (change_dir_event == 1) // 如果是
	{
		data[0] = CMD_ROUND_RUN_CHANGE_DIR_REPORT;
	}
	else
	{
		data[0] = CMD_ROUND_RUN_REPORT;
	}
	data[1] = mot_id;
	SET_WORD(ctrl_ack_data16, data[2], data[3]);
	SET_DWORD(cur_pos, data[4], data[5], data[6], data[7]);
	BOARD_CanSend(CAN_SEND_ID, data, 8);
}
void round_run_ctrl_main_set(uint8_t main_ctrl, uint8_t sub_ctrl) // 主控设置
{
	switch (main_ctrl)
	{
	case RR_CTRL_BEGIN:
		round_run_ctrl_st.ctrl_flag = 1;
		break;
	case RR_CTRL_PAUSE:
		round_run_ctrl_st.pause_flag = 1;
		round_run_ctrl_st.stop_mode = sub_ctrl;
		break;
	case RR_CTRL_RECOVER:
		round_run_ctrl_st.recover_flag = 1;
		round_run_ctrl_st.recover_mode = sub_ctrl;
		break;
	case RR_CTRL_EXIT:
		round_run_ctrl_st.exit_flag = 1;
		break;
	default:
		break;
	}
}
void round_run_ctrl_set_slow_speed(uint16_t speed) 
{
	round_run_ctrl_st.slow_spd = speed ;
}
/*fun:周期性调用
 */
void round_run_ctrl_loop(void)
{
	round_run_ctrl_st.cur_pos = round_run_ctrl_get_cur_pos();
	round_run_report_st.report_info_st.report_bit_st.cur_speed = round_run_ctrl_get_cur_speed();
	round_run_report_st.cur_pos = round_run_ctrl_get_cur_pos();
	round_run_change_dir_st.cur_pos = round_run_ctrl_get_cur_pos();

	if (abs(round_run_ctrl_st.cur_pos - round_run_ctrl_st.report_cur_pos) >= round_run_ctrl_st.report_offpos && round_run_ctrl_st.report_offpos > 0)
	{
		round_run_report_st.report_info_st.report_bit_st.report_ack = RR_REPOT_OFFSET_POS_ACK;
		round_run_ctrl_st.report_cur_pos = round_run_ctrl_st.cur_pos;
		round_run_ctrl_report(round_run_ctrl_st.report_cur_pos, round_run_report_st.report_info_st.report_data, 0);
	}
	if (round_run_ctrl_st.exit_flag == 1)
	{
		round_run_ctrl_st.state = RR_EXIT;
		round_run_ctrl_st.exit_flag = 0;
		round_run_ctrl_st.report_stop_req_flag = 1;
	}
	switch (round_run_ctrl_st.state)
	{
	case RR_INIT:							  // 初始化
		if (round_run_ctrl_st.ctrl_flag == 1) // 往返控制打开
		{
			if (round_run_ctrl_st.run_dir == RIGHT_RUN_DIR && round_run_ctrl_st.left_to_right_flag) // 往右运行
			{
				round_run_ctrl_st.state = RR_TRANS_RIGHT_RUN;
			}
			else if (round_run_ctrl_st.run_dir == LEFT_RUN_DIR && round_run_ctrl_st.right_to_left_flag) // 往左运行
			{
				round_run_ctrl_st.state = RR_TRANS_LEFT_RUN;
			}
			round_run_ctrl_st.report_stop_req_flag = 0;
		}
		else
		{
			if (round_run_ctrl_st.report_stop_req_flag)
			{
				if (round_run_ctrl_is_stop() == 1) // 停止动作
				{
					round_run_ctrl_st.report_stop_req_flag = 0;
					round_run_report_st.report_info_st.report_bit_st.report_ack = RR_REPORT_STOP_ACK;
					round_run_ctrl_report(round_run_report_st.cur_pos, round_run_report_st.report_info_st.report_data, 0);
				}
			}
		}
		break;
	case RR_TRANS_LEFT_RUN: // 提交向左运行动作
		round_run_ctrl_sramp_pos(round_run_ctrl_st.left_pos, 0, round_run_ctrl_st.left_spd);

		round_run_ctrl_st.state = RR_LEFT_RUNNING;
		break;
	case RR_TRANS_RIGHT_RUN: // 提交向右运行动作
		round_run_ctrl_sramp_pos(round_run_ctrl_st.right_pos, 0, round_run_ctrl_st.right_spd);
		round_run_ctrl_st.state = RR_RIGHT_RUNNING;
		break;
	case RR_RIGHT_RUNNING: // 右运行
		if (round_run_ctrl_st.pause_flag == 1)
		{
			round_run_ctrl_stop();
			round_run_ctrl_st.pause_state = RR_TRANS_RIGHT_RUN;
			round_run_ctrl_st.report_stop_req_flag = 1;
			round_run_ctrl_st.pause_flag = 0;
			round_run_ctrl_st.state = RR_PAUSE;
		}
		else
		{
			if (round_run_ctrl_st.cur_pos >= (round_run_ctrl_st.right_pos - round_run_ctrl_st.arr_pos))
			{
				if (round_run_ctrl_st.right_to_left_pause_flag == 1) // 如果主控有暂停标记 则进入暂停命令
				{
					round_run_ctrl_stop();
					round_run_ctrl_st.pause_state = RR_TRANS_LEFT_RUN;
					round_run_ctrl_st.state = RR_PAUSE;
					round_run_ctrl_st.report_stop_req_flag = 1;
				}
				else
				{
					if (round_run_ctrl_st.right_to_left_flag) // 如果有换向标记  
					{
						round_run_ctrl_st.state = RR_TRANS_LEFT_RUN;
						round_run_change_dir_st.change_dir_st.change_dir_bit_st.change_dir_flag = LEFT_RUN_DIR;
						round_run_ctrl_report(round_run_change_dir_st.cur_pos, round_run_change_dir_st.change_dir_st.change_dir_data, 1);
					}
				}
			}
		}
		break;
	case RR_LEFT_RUNNING: // 左运行
		if (round_run_ctrl_st.pause_flag == 1)
		{
			round_run_ctrl_st.pause_state = RR_TRANS_LEFT_RUN;
			round_run_ctrl_st.report_stop_req_flag = 1;
			round_run_ctrl_st.pause_flag = 0;
			round_run_ctrl_st.state = RR_PAUSE;
		}
		else
		{
			if (round_run_ctrl_st.cur_pos <= (round_run_ctrl_st.left_pos + round_run_ctrl_st.arr_pos))
			{
				if (round_run_ctrl_st.left_to_right_pause_flag == 1) // 如果主控有暂停标记 则进入暂停命令
				{
					round_run_ctrl_stop();
					round_run_ctrl_st.pause_state = RR_TRANS_RIGHT_RUN;
					round_run_ctrl_st.state = RR_PAUSE;
					round_run_ctrl_st.report_stop_req_flag = 1;
				}
				else
				{
					if (round_run_ctrl_st.left_to_right_flag ) // 如果有换向标记 
					{
						round_run_ctrl_st.state = RR_TRANS_RIGHT_RUN;
						round_run_change_dir_st.change_dir_st.change_dir_bit_st.change_dir_flag = RIGHT_RUN_DIR;
						round_run_ctrl_report(round_run_change_dir_st.cur_pos, round_run_change_dir_st.change_dir_st.change_dir_data, 1);
					}
				}
			}
		}
		break;
	case RR_PAUSE:								 // 暂停
		if (round_run_ctrl_st.recover_flag == 1) // 恢复
		{
			if (round_run_ctrl_st.recover_mode == RR_STARTUP_SLOW_ONE) // 慢速一行
			{
				round_run_ctrl_st.state = round_run_ctrl_st.pause_state ;
				if(round_run_ctrl_st.state == RR_TRANS_LEFT_RUN)
				{
					round_run_ctrl_st.left_spd = round_run_ctrl_st.slow_spd ;
				}
				else if(round_run_ctrl_st.state == RR_TRANS_RIGHT_RUN)
				{
					round_run_ctrl_st.right_spd = round_run_ctrl_st.slow_spd ;
				}
			}
			else // 匀加速
			{

			}
			round_run_ctrl_st.recover_flag = 0;
			round_run_ctrl_st.report_stop_req_flag = 0;
		}
		else // 上报停止动作
		{
			if (round_run_ctrl_st.report_stop_req_flag)
			{
				if (round_run_ctrl_is_stop() == 1) // 停止动作
				{
					round_run_ctrl_st.report_stop_req_flag = 0;
					round_run_report_st.report_info_st.report_bit_st.report_ack = RR_REPORT_STOP_ACK;
					round_run_ctrl_report(round_run_report_st.cur_pos, round_run_report_st.report_info_st.report_data, 0);
				}
			}
		}
		break;
	case RR_EXIT: // 退出
		round_run_ctrl_stop();
		round_run_ctrl_st.ctrl_flag = 0;
		round_run_ctrl_st.state = RR_INIT;
		break;
	default:
		break;
	}
}
