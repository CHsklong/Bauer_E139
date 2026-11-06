#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include "board.h"
#include "servo_can.h"
#include "motor.h"
#include "param.h"
#include "alert.h"
#include "hal_for_upg.h"
#include "mcdrv.h"
#include "tick.h"
#include "mcdrv_ftm_enc.h"
#include "motor_sync.h"
#include "machine.h"
#include "work_io.h"
#include "jacking_motor.h"
#include "upgrade.h"
#include "hal_can_encrypt.h"
#include "wire_feed_motor.h"
#include "motor_round_run_ctrl.h"
#include "user_app_fun.h"
// #include "take_debug.h"
//////////////////CAN command process ////////////////////////////////////
#define CMD_BUF_LEN (100)

// static uint8_t cmd_buffer[CMD_BUF_LEN][8];
static int32_t cmd_rd = 0;
static int32_t cmd_wr = 0;
CAN_FRAME_t can_frame_buffer[CMD_BUF_LEN];

static int32_t cmd_rd1 = 0;
static int32_t cmd_wr1 = 0;
CAN_FRAME_t can_frame_buffer1[CMD_BUF_LEN];

int easyCanPush1(uint8_t data[8], uint32_t canid, uint8_t dlc)
{
	int32_t len;

	len = cmd_wr1 - cmd_rd1;
	if (len < 0)
		len += CMD_BUF_LEN;
	if (len < CMD_BUF_LEN - 1) // not full
	{
		memcpy(can_frame_buffer1[cmd_wr1].data, data, 8);
		can_frame_buffer1[cmd_wr1].id = canid;
		can_frame_buffer1[cmd_wr1].dlc = dlc;
		cmd_wr1++;
		cmd_wr1 %= CMD_BUF_LEN;

		return 0;
	}
	return -1;
}

int easyCanPop1(uint8_t data[8], uint32_t *pID, uint8_t *pDlc)
{
	if (cmd_wr1 != cmd_rd1) // not empty
	{
		memcpy(data, can_frame_buffer1[cmd_rd1].data, 8);
		*pID = can_frame_buffer1[cmd_rd1].id;
		*pDlc = can_frame_buffer1[cmd_rd1].dlc;
		cmd_rd1++;
		cmd_rd1 %= CMD_BUF_LEN;

		return 0;
	}

	return -1;
}

int easyCanPush(uint8_t data[8], uint32_t canid, uint8_t dlc)
{
	int32_t len;

	len = cmd_wr - cmd_rd;
	if (len < 0)
		len += CMD_BUF_LEN;
	if (len < CMD_BUF_LEN - 1) // not full
	{
		memcpy(can_frame_buffer[cmd_wr].data, data, 8);
		can_frame_buffer[cmd_wr].id = canid;
		can_frame_buffer[cmd_wr].dlc = dlc;
		cmd_wr++;
		cmd_wr %= CMD_BUF_LEN;

		return 0;
	}
	return -1;
}

int easyCanPop(uint8_t data[8], uint32_t *pID, uint8_t *pDlc)
{
	if (cmd_wr != cmd_rd) // not empty
	{
		memcpy(data, can_frame_buffer[cmd_rd].data, 8);
		*pID = can_frame_buffer[cmd_rd].id;
		*pDlc = can_frame_buffer[cmd_rd].dlc;
		cmd_rd++;
		cmd_rd %= CMD_BUF_LEN;

		return 0;
	}

	return -1;
}

void easyCanDebugPrint(void)
{
#ifdef TAKE_DEEBUG_OPEN
	int i;
	uint8_t data[8] = {0};
	float debug1[take_max_num] = {0};
	float debug2[take_max_num] = {0};
	memset(debug1, 0, 4 * take_max_num);
	memset(debug2, 0, 4 * take_max_num);
	take_debug_get_data(debug1, debug2);
	data[0] = CMD_TAKE_DEBUG_PRINT;
	data[1] = mot_id;
	for (i = 0; i < take_max_num; i++)
	{
		data[2] = 0;
		memcpy(&data[4], &debug1[i], 4);
		BOARD_CanSend(CAN_SEND_ID, data, 8);
		hal_delayms(1);
		data[2] = 1;
		memcpy(&data[4], &debug2[i], 4);
		BOARD_CanSend(CAN_SEND_ID, data, 8);
		hal_delayms(1);
	}
	data[0] = CMD_TAKE_DEBUG_END;
	data[1] = mot_id;
	BOARD_CanSend(CAN_SEND_ID, data, 2);
#endif
}
uint8_t g_test_speed = 0;
#define CAN_SEC_FLAG 0
static uint8_t idProc(uint8_t *data)
{

	static uint8_t uid[12], rec_uid[12];
	static uint16_t board_type;
	static uint8_t board_id = 0x06;
	static uint8_t allow_run = 1;
	static uint8_t config_id = 0;
	static uint8_t set_board_id = 0x06;
	uint32_t rand_val;

	static uint8_t id_mode = 0;

#if (CAN_SEC_FLAG == 1)
	if (data[0] == SERVO_DEV_ID) // d4命令 进行解密
	{
		CANEncrypt(&data[1], 7, 0);
	}
#endif
	if (config_id == 0) // id未设置
	{
	}
	else // id 已设置
	{
		if ((data[1] & 0x7f) == board_id || data[1] == 0xff)
		{
		}
		else
		{
			return allow_run;
		}
	}
	//   if(((data[1] &0x7f) == board_id || data[1] == 0xff ) || data[0] == SERVO_DEV_ID || data[0] == SERVO_DEV_ASK)
	{
		switch (data[0])
		{
#if 0
        case SERVO_DEV_INFO:
            dataU16 = GET_WORD(data[2], data[3]);
            if (dataU16 & 0x8000) //读
            {
                switch (dataU16 & 0x1fff)
                {
                case 0x0: // param count   alert count
                    SET_WORD(0, data[4], data[5]);
                    SET_WORD(ALT_MAX, data[6], data[7]);
                    break;
                case 0x1: // board type
                    SET_DWORD(0x139, data[4], data[5], data[6], data[7]);
                    break;
                case 0x2: // uid 1
                    BOARD_GetUID(uid);
                    data[4] = uid[0];
                    data[5] = uid[1];
                    data[6] = uid[2];
                    data[7] = uid[3];
                    break;
                case 0x3: // uid 2
                    data[4] = uid[4];
                    data[5] = uid[5];
                    data[6] = uid[6];
                    data[7] = uid[7];
                    break;
                case 0x4: // uid 3
                    data[4] = uid[8];
                    data[5] = uid[9];
                    data[6] = uid[10];
                    data[7] = uid[11];
                    break;
                case 0x5:   // version of hw
                    SET_DWORD(1001, data[4], data[5], data[6], data[7]);
                     
                    break;
                case 0x6:   // vervsion of boot
                    SET_DWORD(1001, data[4], data[5], data[6], data[7]);
                    break;
                case 0x7:   //version of main
                    SET_DWORD(APP_VER, data[4], data[5], data[6], data[7]);
                    break;
                }
               // BOARD_CanSend(CAN_SEND_ID, data, 8); 
              BOARD_CanSend(CAN_BANDING_SEND_ID + board_id,  data, 8) ;
                
            }
            break;
#endif
		case SERVO_DEV_ID:

			switch (data[2])
			{
			case 0x03:
				id_mode = data[3];
				if (id_mode && config_id == 0) // is config mode
					allow_run = 0;
				break;
			case 0x04:
				board_type = GET_WORD(data[3], data[4]);
				rec_uid[0] = data[5];
				rec_uid[1] = data[6];
				rec_uid[2] = data[7];
				break;
			case 0x05:
				rec_uid[3] = data[3];
				rec_uid[4] = data[4];
				rec_uid[5] = data[5];
				rec_uid[6] = data[6];
				rec_uid[7] = data[7];
				break;
			case 0x06:
				BOARD_GetUID(uid);
				rec_uid[8] = data[3];
				rec_uid[9] = data[4];
				rec_uid[10] = data[5];
				rec_uid[11] = data[6];
				if (board_type == 0x139 && memcmp(uid, rec_uid, 12) == 0)
				{
					board_id = data[1] & 0x7f;
					allow_run = 1;
					data[3] = 1;
					config_id = 1;
					set_board_id = board_id;
					SET_DWORD(0, data[4], data[5], data[6], data[7]);
#if (CAN_SEC_FLAG == 1)
					CANEncrypt(&data[1], 7, 1);
#endif
					BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
					//  BOARD_CanSend(CAN_SEND_ID, data, 8);
				}
				break;
			case 0x0a:
				// srand(getTicks()+getCurPos());

				// board_id = 0x01 ;
				if (data[3] == 0)
				{
					// board_id = 1 ;
					board_id = set_board_id;
				}
				else
				{
					rand_val = rand();
					board_id = rand_val & 0x7f;
					data[1] = board_id;
					data[2] = 0x0a;
					data[3] = 1;
					SET_WORD(0x139, data[4], data[5]);
					SET_WORD(rand_val, data[6], data[7]);
#if (CAN_SEC_FLAG == 1)
					CANEncrypt(&data[1], 7, 1);
#endif
					BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				}

				break;
			/*case 0x0f:
				CANEncrypt_setkey(data[3], GET_DWORD(data[4], data[5], data[6], data[7]));
				break;*/
			case 0x11: // board type
				SET_DWORD(0x139, data[4], data[5], data[6], data[7]);
#if (CAN_SEC_FLAG == 1)
				CANEncrypt(&data[1], 7, 1);
#endif
				BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				break;
			case 0x12: // uid 1
				BOARD_GetUID(uid);
				data[4] = uid[0];
				data[5] = uid[1];
				data[6] = uid[2];
				data[7] = uid[3];
#if (CAN_SEC_FLAG == 1)
				CANEncrypt(&data[1], 7, 1);
#endif
				BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				break;
			case 0x13: // uid 2
				data[4] = uid[4];
				data[5] = uid[5];
				data[6] = uid[6];
				data[7] = uid[7];
#if (CAN_SEC_FLAG == 1)
				CANEncrypt(&data[1], 7, 1);
#endif
				BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				break;
			case 0x14: // uid 3
				data[4] = uid[8];
				data[5] = uid[9];
				data[6] = uid[10];
				data[7] = uid[11];
#if (CAN_SEC_FLAG == 1)
				CANEncrypt(&data[1], 7, 1);
#endif
				BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				break;
			case 0x15: // version of hw
				SET_DWORD(1001, data[4], data[5], data[6], data[7]);
#if (CAN_SEC_FLAG == 1)
				CANEncrypt(&data[1], 7, 1);
#endif
				BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				break;
			case 0x16: // vervsion of boot
				SET_DWORD(1001, data[4], data[5], data[6], data[7]);
#if (CAN_SEC_FLAG == 1)
				CANEncrypt(&data[1], 7, 1);
#endif
				BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				break;
			case 0x17: // version of main
#if (CAN_SEC_FLAG == 1)
				CANEncrypt(&data[1], 7, 1);
#endif
				SET_DWORD(APP_VER, data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				break;
			case 0x18: // 内部通讯错误
			case 0x19: // 子版本类型
			case 0x1a:
			case 0x1b:
			case 0x1c:
			case 0x1d:
			case 0x1e:
			case 0x1f:
				SET_DWORD(0, data[4], data[5], data[6], data[7]);
#if (CAN_SEC_FLAG == 1)
				CANEncrypt(&data[1], 7, 1);
#endif
				BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
				break;
			}
			break;
		case SERVO_DEV_ASK:
			data[1] = board_id;
			BOARD_CanSend(CAN_BANDING_SEND_ID + board_id, data, 8);
			break;

			/*    case SERVO_GET_ALERT_INFO:
							 //BOARD_CanSend(CAN_SEND_ID, data, 8);
					sendAlertInfo(CAN_BANDING_SEND_ID + board_id, data[2], data[3]);
					break;  */
		};
	}
	return allow_run;
}
u8 check_cmd(uint8_t cmd, uint8_t state, uint16_t resv_rand)
{
	u8 ret = 0;
	uint8_t data[8];
	static uint16_t rand_data = 0x4581;
	if (state == 1) // 发送随机数
	{
		srand(getTicks());
		rand_data = rand();
		data[0] = cmd;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0;
		data[5] = state;
		memcpy(&data[6], &rand_data, 2);
		BOARD_CanSend(CAN_SEND_ID, data, 8);
	}
	else if (state == 2) // 校验随机数
	{
		data[0] = cmd;
		data[5] = state;
		if (resv_rand == rand_data)
		{
			ret = 1;
			data[4] = 1;
		}

		else
		{
			ret = 0;
			data[4] = 0;
		}
		memcpy(&data[6], &rand_data, 2);
		BOARD_CanSend(CAN_SEND_ID, data, 8);
	}
	return ret;
}
void can_packet_parse(uint8_t data[8],uint32_t canid,uint8_t len,uint8_t can_type)
{
	int16_t parI16[3];
	uint16_t parU16;
	int32_t parI32;
	Param_Data paraData, *pParaData;
	Mot_Msg_Param motMsgParam;
	
	uint8_t allow_run = 1;
	//if (easyCanPop(data, &canid, &len) == 0)
	{
		CANEncrypt(data);
		allow_run = idProc(data);
		if (data[1] != mot_id && data[1] != 0xff)
			return;

		switch (data[0])
		{
		case CMD_ZERO: // find zero position
			if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
			sync_lock();
#ifdef SYNC_MOTOR_FUN
			Servo_PID_Change(PID_NOMAL);
#endif
			//  motMsgParam.motZero.speed = spdOut2In(MOT_MSG_ZERO,GET_WORD(data[2], data[3]));
			parI16[0] = spdOut2In(MOT_MSG_ZERO, GET_WORD(data[2], data[3]));
			motMsgParam.motZero.speed = (float)parI16[0];
			motMsgParam.motZero.torque = MLIB_Abs_FLT(torqOut2In(GET_WORD(data[4], data[5])));
			mot_run_state.zero_mode = data[6];
			mot_run_state.mot_zero_spd = motMsgParam.motZero.speed;
			// motMsgParam.motZero.torque= 1000;
			sync_follow_init();
			motRunSM(MOT_MSG_ZERO, &motMsgParam);
			sync_un_lock();
#ifdef DEBUG_CMD
			if (mot_run_state.ack_cmd)
				reportCmd(data[0]);
#endif
			break;

		case CMD_SYNC_CW: // sync by cw
			motMsgParam.motSync.dir = CW;
			motMsgParam.motSync.offset = GET_WORD(data[2], data[3]);
			motMsgParam.motSync.syncPos = GET_DWORD(data[4], data[5], data[6], data[7]);
			// motRunSM(MOT_SYNC, &motMsgParam);
			sync_start_follow(motMsgParam.motSync.offset, motMsgParam.motSync.syncPos, motMsgParam.motSync.dir);
			break;

		case CMD_SYNC_CCW: // sync by ccw
			motMsgParam.motSync.dir = CCW;
			motMsgParam.motSync.offset = GET_WORD(data[2], data[3]);
			motMsgParam.motSync.syncPos = GET_DWORD(data[4], data[5], data[6], data[7]);
			// motRunSM(MOT_SYNC, &motMsgParam);
			sync_start_follow(motMsgParam.motSync.offset, motMsgParam.motSync.syncPos, motMsgParam.motSync.dir);
			break;

		case CMD_SYNC_STOP: // sync stop
							// motMsgParam.motSync.syncPos = posOut2In(GET_DWORD(data[4], data[5], data[6], data[7]));
							// motRunSM(MOT_SYNC_STOP, &motMsgParam);
			motMsgParam.motSync.syncPos = GET_DWORD(data[4], data[5], data[6], data[7]);
			sync_end_pos(motMsgParam.motSync.syncPos);
			break;
		case CMD_SERVO_MOVE_SYNC_CW:
		case CMD_SERVO_MOVE_SYNC_CCW:
			sync_lock();
			mot_run_state.target_pos_offset = 5 * 8;
			parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
			mot_run_state.posReq = parI32;
			mot_run_state.move_pos_begin = getTicks();
			sync_set_stop_pos(parI32);
			movePos(posOut2In(parI32), MLIB_Abs_FLT(spdOut2In(MOT_MSG_MOVE_POS, GET_WORD(data[2], data[3]))), -1);
			sync_un_lock();
		case CDM_MOVE_CIRCLE:
			mot_run_state.target_pos_ack_flag = 1;
			// mot_run_state.punching_speed = GET_WORD(data[2], data[3]) ;
			// parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
			mot_run_circle();
			break;

		case CMD_EXT_POS_CTRL:
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
                        if(mot_run_state.ext_ctrl_pos_enable ==1) //外部控制时不能操作
			{
                            servo_ext_pos_ctrl_set(GET_WORD(data[2], data[3]),GET_DWORD(data[4], data[5], data[6], data[7])) ;
                        }
			break;
                case CMD_MOVE_POS_ACK:
#ifdef DEBUG_CMD
			if (mot_run_state.ack_cmd)
				reportCmd(data[0]);
#endif
			mot_run_state.target_pos_ack_flag = 1;
		case CMD_MOVE_POS_TO:
		case CMD_MOVE_POS:
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
			sync_lock();
#ifdef SYNC_MOTOR_FUN
			Servo_PID_Change(PID_NOMAL);
#endif
			mot_run_state.target_pos_offset = 5 * 8;
			parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
			mot_run_state.pos_target = parI32;
			motMsgParam.motMove.pos = posOut2In(parI32);
			mot_run_state.posReq = parI32;
			mot_run_state.move_pos_begin = getTicks();
			motMsgParam.motMove.speed = MLIB_Abs_FLT(spdOut2In(MOT_MSG_MOVE_POS, GET_WORD(data[2], data[3])));
			motMsgParam.motMove.torque = MLIB_Abs_FLT(torqOut2In(1000));
			sync_end_follow(parI32);

			motRunSM(MOT_MSG_MOVE_POS_TO, &motMsgParam);
			sync_un_lock();
			break;

		case CMD_CLIB_ZERO: // calibrate by encoder
#if (BOARD_TYPE == BOARD_E914)
			if (check_cmd(data[0], data[5], GET_WORD(data[6], data[7])))
			{
				motEncCalibrate();
			}
#else
			motEncCalibrate();
#endif

			break;

		case CMD_ENABLE: // enable/disable motor
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
#ifdef SYNC_MOTOR_FUN
			Servo_PID_Change(PID_NOMAL);
#endif
			alertClear();
			motMsgParam.motEnable = GET_WORD(data[2], data[3]);
			mot_run_state.target_pos_offset = 5 * 8;
			sync_set_enable(motMsgParam.motEnable);
			mot_run_state.run_mode = RUN_MODE_RUN;
			/**********lingshi *********/
			motRunSM(MOT_MSG_STOP, NULL);
			/**********lingshi *********/
			motRunSM(MOT_MSG_ENABLE, &motMsgParam);
#ifdef DEBUG_CMD
			if (mot_run_state.ack_cmd)
				reportCmd(data[0]);
#endif
			break;

		case CMD_CLR_ALERT: // clear alert
			alertClear();
			break;
#if 0
		case CMD_MOVE_POS: //move in position mode
			parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
			motMsgParam.motMove.pos = posOut2In(parI32);
			motMsgParam.motMove.speed = spdOut2In(GET_WORD(data[2], data[3]));
			motMsgParam.motMove.torque = -1;
			motRunSM(MOT_MSG_MOVE_POS, &motMsgParam);
			break;
#endif
		case CMD_MOVE_SPD: // move in speed mode
#if 0					   // 工装程序时直接返回
#else
						   // BOARD_CanSend(CAN_SEND_ID, data, 2);
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			parI16[0] = GET_WORD(data[2], data[3]);
			// parI16[0] = 1000;

#ifdef SYNC_MOTOR_FUN
			Servo_PID_Change(PID_NOMAL);
#endif
			parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
			motMsgParam.motMove.speed = spdOut2In(MOT_MSG_MOVE_SPD, parI32);
                        mot_run_state.speedReq = motMsgParam.motMove.speed ;
			motMsgParam.motMove.torque = MLIB_Abs_FLT(torqOut2In(parI16[0]));
			motRunSM(MOT_MSG_MOVE_SPD, &motMsgParam);
#endif
			break;
		case CMD_SCALAR_CTRL:
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			parI16[0] = GET_WORD(data[2], data[3]);
			parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
			motMsgParam.motMove.speed = spdOut2In(MOT_MSG_MOVE_SPD, parI32);
			motMsgParam.motMove.torque = MLIB_Abs_FLT(torqOut2In(parI16[0]));
			motRunSM(MOT_MSG_SCALAR_CTRL, &motMsgParam);
			break;
		case CMD_MOVE_TORQ: // move in torque mode
		case CMD_MOVE_TORQ_CHANGE:
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			parI16[0] = GET_WORD(data[2], data[3]);

			motMsgParam.motMove.torque = torqOut2In(parI16[0]);
			motRunSM(MOT_MSG_MOVE_TORQ, &motMsgParam);
			break;
		case CMD_MOVE_TO_POS_TO_TQRD: // 移动位置到位后切换力矩模式
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			mot_run_state.target_pos_ack_flag = 1;
			mot_run_state.target_pos_offset = 5 * 8;
			motMsgParam.motMove.pos = posOut2In(GET_WORD(data[4], data[5]));
			motMsgParam.motMove.speed = MLIB_Abs_FLT(spdOut2In(MOT_MSG_MOVE_POS, (float)GET_WORD(data[2], data[3])));
			motMsgParam.motMove.torque = MLIB_Abs_FLT(torqOut2In((float)GET_WORD(data[6], data[7])));
			mot_run_state.mot_pos_to_torque_st.pos = motMsgParam.motMove.pos;
			mot_run_state.mot_pos_to_torque_st.speed = motMsgParam.motMove.speed;
			mot_run_state.mot_pos_to_torque_st.torque = motMsgParam.motMove.torque;
			mot_run_state.run_mode = RUN_MODE_POS_TO_TORQUE;
			motRunSM(MOT_MSG_MOVE_POS, &motMsgParam);

			break;
		case CMD_MOVE_POS_SPD_TORQ: // move in position with limited speed and torque
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			motMsgParam.motMove.pos = posOut2In(GET_WORD(data[4], data[5]));
			motMsgParam.motMove.speed = MLIB_Abs_FLT(spdOut2In(MOT_MSG_MOVE_POS, (float)GET_WORD(data[2], data[3])));
			motMsgParam.motMove.torque = MLIB_Abs_FLT(torqOut2In((float)GET_WORD(data[6], data[7])));

			motRunSM(MOT_MSG_MOVE_POS, &motMsgParam);

			break;

		case CMD_MOVE_POS_SPD: // move in position mode with limited speed
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
#ifdef SYNC_MOTOR_FUN
			Servo_PID_Change(PID_NOMAL);
#endif
			parI16[0] = GET_WORD(data[6], data[7]);
			parI32 = GET_DWORD(data[2], data[3], data[4], data[5]);
			motMsgParam.motMove.pos = posOut2In(parI32);
			motMsgParam.motMove.speed = MLIB_Abs_FLT(spdOut2In(MOT_MSG_MOVE_POS, parI16[0]));
			motMsgParam.motMove.torque = -1;
			motRunSM(MOT_MSG_MOVE_POS, &motMsgParam);
			break;
		case CMD_MOVE_POS_TIME: // 带时间的位置模式
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
			mot_run_state.mot_move_time_param.move_flag = 1;
			mot_run_state.mot_move_time_param.target_pos = posOut2In(parI32);
#ifdef SYNC_MOTOR_FUN
			Servo_PID_Change(PID_NOMAL);
#endif
			if (mot_run_state.mot_move_time_param.time == 0)
			{
				mot_run_state.mot_move_time_param.cur_pos = getCurPos();
			}
			mot_run_state.mot_move_time_param.time = GET_WORD(data[2], data[3]);
			if (mot_run_state.mot_move_time_param.time > 0)
			{
				mot_run_state.mot_move_time_param.time = mot_run_state.mot_move_time_param.time / mot_run_state.mot_move_time_param.dis_time;
				if (mot_run_state.mot_move_time_param.time == 0)
					mot_run_state.mot_move_time_param.time = 1;
			}
			mot_run_state.mot_move_time_param.begin_pos = mot_run_state.mot_move_time_param.cur_pos;
			mot_run_state.mot_move_time_param.dis_pos = (mot_run_state.mot_move_time_param.target_pos - mot_run_state.mot_move_time_param.begin_pos) / mot_run_state.mot_move_time_param.time;
			break;
		case CMD_STOP: // stop moving
			// BOARD_CanSend(CAN_SEND_ID, data, 2);
                        if(servo_get_ctrl_enable(can_type) ==0) //外部控制时不能操作
                          break;
			if (allow_run == 0)
			{
				alertPush(ALT_MOT_NO_BIND, 0);
				break;
			}
#ifdef SYNC_MOTOR_FUN
			Servo_PID_Change(PID_NOMAL);
#endif
			mot_run_state.target_pos_offset = 5 * 8;
			sync_follow_init();
			motRunSM(MOT_MSG_STOP, NULL);
#ifdef DEBUG_CMD
			if (mot_run_state.ack_cmd)
				reportCmd(data[0]);
#endif
			break;

		case CMD_FIND_DIST: // find side.
			motMsgParam.motMove.speed = MLIB_Abs_FLT(spdOut2In(MOT_MSG_FIND_SIDE, GET_WORD(data[2], data[3])));
			motMsgParam.motMove.torque = MLIB_Abs_FLT(torqOut2In((float)GET_WORD(data[4], data[5])));
			motRunSM(MOT_MSG_FIND_SIDE, &motMsgParam);
			break;

		case CMD_ID_MODE: // enter/exit ID setting
			motMsgParam.devIdMode = GET_WORD(data[2], data[3]);
			motRunSM(MOT_MSG_DEVID_MODE, &motMsgParam);
			break;

		case CMD_ID_RESP: // read rand device id
			motRunSM(MOT_MSG_SEND_RANDID, 0);
			break;

		case CMD_SEL_DEVICE: // select device
			motMsgParam.devSelect.rand = GET_DWORD(data[4], data[5], data[6], data[7]);
			motMsgParam.devSelect.sel = GET_WORD(data[2], data[3]);
			motRunSM(MOT_MSG_SELECT_DEV, &motMsgParam);
			break;
		case CMD_ENCRYPT_KEY:
			if (GET_WORD(data[2], data[3]) == 0x4581)
			{
				CANEncrypt_setkey(GET_DWORD(data[4], data[5], data[6], data[7]));
			}
			break;
		case CMD_WR_ID: // set device ID
			motMsgParam.devIdSetting.rand = GET_DWORD(data[4], data[5], data[6], data[7]);
			motMsgParam.devIdSetting.motId = GET_WORD(data[2], data[3]);
			motRunSM(MOT_MSG_SET_DEVID, &motMsgParam);
			break;
		case CMD_ROUND_RUN_SET_CHANGE_DIR_DATA:
			round_run_ctrl_set_run_target(GET_DWORD(data[4], data[5], data[6], data[7]), GET_WORD(data[2], data[3]));
			break;
		case CMD_WR_PARAM: // write parameter

			parU16 = GET_WORD(data[2], data[3]);
#ifdef DEBUG_CMD
			if (mot_run_state.ack_cmd || parU16 == 0x802f)
				reportCmd(data[0]);
#endif

			if (parU16 == 0x8001) // current position
			{
				parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
				setCurPos(parI32);
				if (parI32 == 0)
				{
					saveZeroPos();
				}
			}
			else if (parU16 == 0x0000)
			{
				/*int16_t data1,data2 ;
				data1 = GET_WORD(data[4], data[5]) ;
				data2 = GET_WORD(data[6], data[7]) ;
				if(check_cmd(0x02,(data1>>8)&0xff,data2))*/
				{
					paraData.u2_data = GET_WORD(data[4], data[5]);
					writeParameter(PARNO_MOT_ID, paraData);
				}
			}
			else if (parU16 == 0x8002) // run direct and zero direct
			{
				short dw1, dw2;
				if (mot_run_state.dir_protocol == DIR_PROTOCOL_ZHISHAN)
				{
					dw1 = GET_WORD(data[6], data[7]);
					dw2 = GET_WORD(data[4], data[5]);
				}
				else
				{
					dw1 = (~GET_WORD(data[4], data[5])) & 0x01;
					dw2 = (~GET_WORD(data[6], data[7])) & 0x01;
				}
				/*dw1 = GET_WORD(data[6], data[7]);
				dw2 = GET_WORD(data[4], data[5]);*/
				paraData.u2_data = dw1;
				writeParameter(PARNO_RUN_DIR, paraData);
				paraData.u2_data = dw2;
				writeParameter(PARNO_ZERO_DIR, paraData);
			}
			else if (parU16 == 0x8003) // stop position after zero
			{
				parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
				paraData.f4_data = ((float)parI32);
				mot_run_state.zeroStPos = paraData.f4_data;
				writeParameter(PARNO_ZERO_STOP, paraData);
			}
			else if (parU16 == 0x8004) // zero position
			{
#if 1
				parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
				paraData.f4_data = ((float)parI32);
				mot_run_state.zeroPos = parI32;
				// writeParameter(PARNO_ZERO_POS, paraData);
#endif
			}
			else if (parU16 == 0x8005) // gear ratio
			{
#if 1
				paraData.i2_data = GET_WORD(data[6], data[7]);
#if (BOARD_TYPE == BOARD_E145)
#ifdef JACKING_MOTOR_FUN // 升降功能打开
				mot_run_state.nume = paraData.i2_data * 3;
#else
				mot_run_state.nume = paraData.i2_data;
#endif
#else
				mot_run_state.nume = paraData.i2_data;
#endif
				//  writeParameter(PARNO_NUME, paraData );
				paraData.i2_data = GET_WORD(data[4], data[5]);
				mot_run_state.deno = paraData.i2_data;
				//  writeParameter(PARNO_DENO, paraData);
				initMotorPos();
#endif
			}
			else if (parU16 == 0x8006) // max offset in sync
			{
				parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
				/*  paraData.f4_data = posOut2In((float)parI32);
				 writeParameter(PARNO_MAX_SYNC_OFF, paraData);*/
				sync_set_follow_dwviation(parI32);
			}
			else if (parU16 == 0x8007) // offset ratio in sync
			{
				paraData.i2_data = GET_WORD(data[6], data[7]); //
				if (!paraData.i2_data)
					paraData.i2_data = 1;
				mot_run_state.nume2 = paraData.i2_data;
				paraData.i2_data = GET_WORD(data[4], data[5]);
				mot_run_state.deno2 = paraData.i2_data; //
			}
			else if (parU16 == 0x8008) //
			{
				parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
				if (mot_run_state.sensor_flag == 1) //
				{
					mot_run_state.zero_set_pos = parI32;
				}
				else
				{
					setCurPos(parI32);
					update_target_pos(posOut2In(mot_run_state.pos_target));
				}
			}
			else if (parU16 == 0x8009) //
			{
				mot_run_state.sensor_flag = data[4];
			}
			else if (parU16 == 0x800C) //
			{
				paraData.i1_data = data[4];
				writeParameter(PARNO_CAN_BAUND, paraData);
			}
			else if (parU16 == 0x800e)
			{
				mot_run_state.dir_protocol = data[4];
			}
			else if (parU16 == 0x8028)
			{
				mot_run_state.punching_speed = GET_WORD(data[4], data[5]);
				paraData.f4_data = mot_run_state.punching_speed;
				writeParameter(57, paraData);
			}
			else if (parU16 == 0x8040)
			{
#if (BOARD_TYPE == BOARD_E139)
				if (g_test_speed == 0)
				{
					if (data[4] == 3)
					{
						g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = motor_run.spdPidP1;
						g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = motor_run.posPip1;
						g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = motor_run.spdPidI1 / 1000.0f;
					}
					else
					{
						mot_run_state.change_pid_begin_tick = getTicks();
						mot_run_state.change_pid_flag = 1;
						// g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = motor_run.spdPidP ;
						//  g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = motor_run.posPip ;
					}
				}
#endif
			}
			else if (parU16 == 0x8045)
			{
				work_output(OUTPUT_IDX1, data[5]);
			}
			else if (parU16 == 0x8047)
			{
				mot_run_state.io_fun_set = GET_DWORD(data[4], data[5], data[6], data[7]);
				work_io_set_fun((int32_t)mot_run_state.io_fun_set);
			}
			else if (parU16 == 0x8048)
			{
				mot_run_state.punching_cnt = GET_DWORD(data[4], data[5], data[6], data[7]);
			}
			else if (parU16 == 0x8049)
			{
			}
			else if (parU16 == 0x8050)
			{
			}
			else if (parU16 >= 0x8100 && parU16 <= 0x8200)
			{
				// uint16_t para_no = 0;

				// para_no = parU16 - 0x8100 + PARANO_MOTOR_BEGIN;
#if 0
				//	  paraData.f4_data = GET_DWORD(data[4],data[5],data[6],data[7]);
				//	 	memcpy(&paraData.f4_data,&data[4],4) ;
#else
				int32_t para_value = 0;
				memcpy(&para_value, &data[4], 4);

				paraData.f4_data = (float)(para_value / 10000.0f);
				// paraData.i4_data = para_value / 10000 ;
#endif
				writeParameter(parU16 - 0x8100 + PARANO_MOTOR_BEGIN, paraData);

				motSysParamApply();
			}
#ifdef JACKING_MOTOR_FUN // 升降功能打开
			else if (parU16 >= 0x8610 && parU16 <= 0x8700)
			{
				uint16_t para_no = 0;
				para_no = parU16 - 0x8610;

				paraData.i4_data = GET_DWORD(data[4], data[5], data[6], data[7]);
				if (parU16 == 0x8621) // 大扭矩
				{
					paraData.i4_data = torqOut2In(paraData.i4_data) * 1000;
				}
				writeJackingMotorParameter(para_no, paraData);
			}
			else if (parU16 == 0x8601)
			{
				jacking_motor_ctrl(data[4], data[5], GET_WORD(data[6], data[7]));
			}
#endif
#ifdef ROUND_RUN_FUN // 往返运行功能
			else if (parU16 >= 0x8810 && parU16 <= 0x8900)
			{

				paraData.i4_data = GET_DWORD(data[4], data[5], data[6], data[7]);
				if (parU16 == 0x8811) // 设置位移上报偏移量
				{
					round_run_ctrl_set_report_offpos(paraData.i4_data);
				}
				else if (parU16 == 0x8816) // 设置恢复时的慢速速度
				{
					round_run_ctrl_set_slow_speed(GET_WORD(data[4], data[5]));
				}
			}
			else if (parU16 == 0x8801)
			{
				round_run_ctrl_main_set(data[4], data[5]);
			}
#endif
#ifdef WIRE_MOTOR_FUN // 穿丝功能打开
			else if (parU16 >= 0x8710 && parU16 <= 0x8800)
			{
				uint16_t para_no = 0;
				para_no = parU16 - 0x8710;
				paraData.i4_data = GET_DWORD(data[4], data[5], data[6], data[7]);

				if ((parU16 == 0x8714) || (parU16 == 0x8727)) // 末端位置和零位保存值不写入
				{
				}
				else
				{
					writeWfMotorParameter(para_no, paraData);
				}
				if (parI16 == 0x8723) // 断丝检测
				{
					wire_feed_motor_set_broken_check_type();
				}
			}
			else if (parU16 == 0x8701)
			{
				wire_feed_motor_ctrl(data[4], data[5], GET_WORD(data[6], data[7]));
			}
			else if (parU16 == 0x8705) // 保存当前的参数值到某一组
			{
				SaveWfMotorGroupParamToEpp(data[4]);
			}
			else if (parU16 == 0x8706) // 设置该组号为当前使用的组号
			{
				SaveWfMotorCurGroup(data[4]);
				ReadWfMotorGroupParamFromEpp(data[4]);
				SaveWfMotorGroupParamToEpp(0);
				wire_feed_motor_set_broken_check_type();
			}
			else if (parU16 == 0x8707)
			{
				if (data[4] == 0)
				{
					DeleteAllWfMotorGroup();
				}
				else
				{
					AddWfMotorGroup(data[4], data[5]);
				}
			}
#endif
			else if (parU16 == 0x8025)
			{
				mot_run_state.signal_report = GET_WORD(data[4], data[5]);
			}
			else if (parU16 == 0x802c)
			{
				servo_Auto_SR_Set_Type(data[4]);
				servo_Auto_SR_Set_Sensor_Port(data[5]);
				servo_Auto_SR_Set_CanID(GET_WORD(data[6], data[7]));
			}
			else if (parU16 == 0x802d)
			{
				servo_Auto_SR_Enable((GET_WORD(data[4], data[5]) >> 15) & 0x01);
				servo_Auto_SR_Set_Period(GET_WORD(data[4], data[5]) & 0x7fff);
			}
			else if (parU16 == 0x802e)
			{
				servo_Auto_SR_Set_offset(GET_DWORD(data[4], data[5], data[6], data[7]));
			}
			else if (parU16 == 0x802f)
			{
				mot_run_state.ack_cmd = data[4];
			}
                        else if(parU16 == 0x8056)
			{
                                servo_sync_buf_zero() ;
                        }
                        else if(parU16 == 0x8060)
			{
				servo_sync_clib_time( GET_DWORD(data[4], data[5], data[6], data[7]));
			}
                        else if(parU16 == 0x8061)
                        {
                                servo_set_ext_ctrl_enable(data[4],data[5], can_type);
                        }
			break;

		case CMD_RD_PARAM: // read parameter
			parU16 = GET_WORD(data[2], data[3]);
			if (parU16 == 0x8001)
			{
				parI32 = posIn2Out(getCurPos());
				SET_DWORD(parI32, data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8002)
			{
				pParaData = readParameter(PARNO_RUN_DIR);
				SET_WORD(pParaData->i1_data, data[4], data[5]);
				pParaData = readParameter(PARNO_ZERO_DIR);
				SET_WORD(pParaData->i1_data, data[6], data[7]);

				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8003)
			{
				pParaData = readParameter(PARNO_ZERO_STOP);
				parI32 = (pParaData->f4_data);
				SET_DWORD(parI32, data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8004)
			{
				pParaData = readParameter(PARNO_ZERO_POS);
				parI32 = (pParaData->f4_data);
				SET_DWORD(parI32, data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8005)
			{
				pParaData = readParameter(PARNO_NUME);
				SET_WORD(pParaData->i2_data, data[4], data[5]);
				pParaData = readParameter(PARNO_DENO);
				SET_WORD(pParaData->i2_data, data[6], data[7]);

				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8006) // max offset in sync
			{
				pParaData = readParameter(PARNO_MAX_SYNC_OFF);
				parI32 = posIn2Out(pParaData->f4_data);
				SET_DWORD(parI32, data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8007)
			{
				pParaData = readParameter(PARNO_SYNC_OFF_RATIO);
				SET_WORD(pParaData->i2_data, data[4], data[5]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8028)
			{
				uint16_t speed = (uint16_t)mot_run_state.punching_speed;

				SET_WORD(speed, data[4], data[5]);
				BOARD_CanSend(CAN_SEND_ID, data, 6);
			}
			else if (parU16 == 0x8048)
			{

				SET_DWORD(mot_run_state.punching_cnt, data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8049)
			{
				SET_WORD(mot_run_state.punching_max_current, data[4], data[5]);
				BOARD_CanSend(CAN_SEND_ID, data, 6);
			}
			else if(parU16 == 0x8059)
			{
				
				SET_WORD(servo_get_sync_pos_ptr(), data[4], data[5]); 
				if(can_type ==0)
				BOARD_CanSend(CAN_SEND_ID, data, 6);
				else
 				BOARD_Can1Send(CAN_SEND_ID, data, 6);	
			}
 			else if(parU16 == 0x8060)
			{
				 
				SET_DWORD(servo_sync_get_clib_time(), data[4], data[5], data[6], data[7]);
				if(can_type ==0)
				BOARD_CanSend(CAN_SEND_ID, data, 8);
				else
				BOARD_Can1Send(CAN_SEND_ID, data, 8); 
			}	
                        else if(parU16 == 0x8061)
			{
                                data[4] = mot_run_state.ext_ctrl_pos_enable ;
				if(can_type ==0)
				BOARD_CanSend(CAN_SEND_ID, data, 5);
				else
				BOARD_Can1Send(CAN_SEND_ID, data, 5); 
			}
			else if (parU16 >= 0x8100 && parU16 <= 0x8200)
			{
				pParaData = readParameter(parU16 - 0x8100 + PARANO_MOTOR_BEGIN);
				if (pParaData)
				{
					//  int32_t para_value = (int32_t) ;
					//  SET_DWORD((int32_t)(pParaData->f4_data), data[4], data[5], data[6], data[7]);
					/*	if( (parU16-0x8100) ==PARANO_MOTOR_LS)
						{
							SET_DWORD((int32_t)(pParaData->f4_data*10000000), data[4], data[5], data[6], data[7]);
						}
						else*/
					{
						SET_DWORD((int32_t)(pParaData->f4_data * 10000), data[4], data[5], data[6], data[7]);
					}
					//	memcpy(&data[4],&pParaData->f4_data,4) ;
					BOARD_CanSend(CAN_SEND_ID, data, 8);
				}
			}
#ifdef JACKING_MOTOR_FUN // 升降功能打开
			else if (parU16 == 0x8602)
			{
				jacking_motor_report_status();
			}
			else if (parU16 >= 0x8610 && parU16 <= 0x8700)
			{
				uint16_t para_no = 0;
				para_no = parU16 - 0x8610;
				SET_DWORD(readJackingMotorParameter(para_no), data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
#endif
#ifdef WIRE_MOTOR_FUN // 穿丝功能打开
			else if (parU16 >= 0x8710 && parU16 <= 0x8800)
			{
				uint16_t para_no = 0;
				para_no = parU16 - 0x8710;
				SET_DWORD(readWfMotorParameter(para_no), data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (parU16 == 0x8702)
			{
				wire_feed_motor_report_status();
			}
			else if (parU16 == 0x8703)
			{
				uint8_t dis_pos = data[4];
				wire_feed_motor_report_current_from_pos(dis_pos);
			}
			else if (parU16 == 0x8704)
			{

				if (data[4] == 0)
				{
					data[4] = GetWfMotorGroup(data[4]);
					data[5] = 0;
					data[6] = 0;
				}
				else
				{
					data[5] = GetWfMotorGroup(data[4]);
					data[6] = GetWfMotorFactoryGroup(data[4]);
				}
				BOARD_CanSend(CAN_SEND_ID, data, 7);
			}
			else if (parU16 == 0x8706)
			{
				data[4] = GetWfMotorCurGroup();
				data[5] = 0;
				data[6] = 0;

				BOARD_CanSend(CAN_SEND_ID, data, 7);
			}
#endif
			break;
		case CMD_WR_PARAM_LIM: // set left/right limit
			parI16[0] = GET_WORD(data[2], data[3]);
			if (parI16[0] == 1)
			{
				paraData.f4_data = GET_DWORD(data[4], data[5], data[6], data[7]);
				writeParameter(PARNO_RIGHT_LMT, paraData);
				// initMotorPos();
			}
			else if (parI16[0] == 2)
			{
				paraData.f4_data = GET_DWORD(data[4], data[5], data[6], data[7]);
				writeParameter(PARNO_LEFT_LMT, paraData);
				// initMotorPos();
			}
			else if (parI16[0] == 3)
			{
				paraData.f4_data = FLT_MAX;
                                mot_run_state.speed_right_pos = FLT_MAX ;
				writeParameter(PARNO_RIGHT_LMT, paraData);
			}
			else if (parI16[0] == 4)
			{
				paraData.f4_data = -FLT_MAX;
                                mot_run_state.speed_left_pos = -FLT_MAX;
				writeParameter(PARNO_LEFT_LMT, paraData);
			}
                        else if(parI16[0] == 5)
                        {
                                mot_run_state.speed_right_pos = GET_DWORD(data[4], data[5], data[6], data[7]);
				

                        }
                        else if(parI16[0] == 6)
                        {
                               mot_run_state.speed_left_pos = GET_DWORD(data[4], data[5], data[6], data[7]);
				

                        }
			break;

		case CMD_AUTO_ANSWER: // auto report motor state
			parU16 = GET_WORD(data[2], data[3]);
			if (parU16 & 0x0080)
				parU16 = GET_WORD(data[4], data[5]);
			else
				parU16 = 0;
			reportStart(parU16);
			break;

		case CMD_RD_STATUS: // query motor state
			if (canid == servo_Auto_SR_Get_CanID() && servo_Auto_SR_Get_Type() == MOTOR_SLAVE_TYPE)
			{
				servo_Auto_SR_Set_Resv_Pos(GET_DWORD(data[4], data[5], data[6], data[7]));
			}
			else
			{
				if (canid == 0x01 || canid == 0x02)
				{
					reportState(CAN_SEND_ID, mot_id,can_type);
                                      
				}
			}
			break;
		case CMD_RD_SPD: //
			SET_WORD((int16_t)getCurSpd(0), data[2], data[3]);
			SET_WORD((int16_t)getCurSpd(1), data[4], data[5]);
			SET_WORD(0, data[6], data[7]);
			if(can_type ==0)
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			else
				BOARD_Can1Send(CAN_SEND_ID, data, 8); 
		
			break;
		case CMD_RD_TORQ: // query torque
			parU16 = (uint16_t)torqIn2Out(fabs(getCurTorq(0)));
			SET_WORD(parU16, data[2], data[3]);
			parU16 = (uint16_t)torqIn2Out(fabs(getCurTorq(1)));
			SET_WORD(parU16, data[4], data[5]);
			SET_WORD(0, data[6], data[7]);
                        if(can_type ==0)
                        BOARD_CanSend(CAN_SEND_ID, data, 8);
                        else
                        BOARD_Can1Send(CAN_SEND_ID, data, 8); 
			break;
		case CMD_COND_RESP: // report arrived
#ifdef DEBUG_CMD
			if (mot_run_state.ack_cmd)
				reportCmd(data[0]);
#endif
			parU16 = GET_WORD(data[2], data[3]);
			parI32 = GET_DWORD(data[4], data[5], data[6], data[7]);
			if (parU16 == 1)
			{
				mot_run_state.repZero = true;
			}
			else if (parU16 == 2)
			{
				// mot_run_state.repUpPos = posOut2In(parI32) -10;
				mot_run_state.repUpPos = parI32 - 3;
			}
			else if (parU16 == 3)
			{
				// mot_run_state.repDnPos = posOut2In(parI32) + 10;
				mot_run_state.repDnPos = parI32 + 3;
			}
			mot_run_state.report_type = parU16;
			break;

		case CMD_PROG_START: // enter upgrade mode
			parU16 = GET_WORD(data[4], data[5]);
			if (parU16 == DEVICE_ID || parU16 == 0xA5A5)
			{
				enableMotor(false);
				hal_delayms(50);
				hal_set_dev_id(data[1]);
				hal_upg_begin(data[7]);
				set_bootflag_data(1);
				hpm_ota_soc_reset();
			}
			else
			{
				data[1] = mot_id;
				data[2] = DEVICE_ERROR;
				data[3] = 1;
				BOARD_CanSend(CAN_SEND_ID, data, 4);
			}

			break;
		case CMD_RESET:
			if (check_cmd(data[0], data[5], GET_WORD(data[6], data[7])))
			{
				if (data[1] == 1)
					hpm_ota_soc_reset();
			}
			break;
		case CMD_RD_DEV_ID:
			data[1] = mot_id;
			SET_WORD(0x4851, data[2], data[3]);
			SET_WORD(DEVICE_ID, data[4], data[5]);

                        if(can_type ==0)
                        BOARD_CanSend(CAN_SEND_ID, data, 8);
                        else
                        BOARD_Can1Send(CAN_SEND_ID, data, 8); 
			break;
			/*case CMD_RD_DEV_ID: //
			  data[1] = mot_id;
			  if(data[2] == 1)
			  {
				parI32 = mot_run_state.syncPos;
			  }
			  else if(data[2] == 2)
			  {
				parI32 = mot_run_state.syncOfsset;
			  }
			  else if(data[2] == 3)
			  {
				parI32 = mot_run_state.masterPos;
			  }
			  else if(data[2] == 4)
			  {
				parI32 = getCurPos();
			  }
			  else if(data[2] == 5)
			  {
				parI32 = posRatioIn2Out(mot_run_state.syncErr);
			  }
			  else if(data[2] == 6)
			  {
				parI32 = MCDRV_FtmEncGetCnt(&g_sM1Enc);
			  }
			  else if(data[2] == 7)
			  {
				parI32 = MCDRV_FtmEncGetAbsCnt(&g_sM1Enc);
			  }
			  data[2] = mot_run_state.run_mode;
			  data[3] = mot_run_state.sub_mode;
			  SET_DWORD(parI32, data[4], data[5], data[6], data[7]);
			  BOARD_CanSend(CAN_SEND_ID, data, 8);
			  break;
			 */
		case CMD_RD_DEV_VER: //

			data[1] = mot_id;
			SET_WORD(0x4851, data[2], data[3]); // 731
			SET_WORD(APP_VER, data[4], data[5]);
			SET_WORD(0, data[6], data[7]);
			//  SET_DWORD(DRV8353_Status(), data[4], data[5],data[6],data[7]);
                        if(can_type ==0)
                        BOARD_CanSend(CAN_SEND_ID, data, 8);
                        else
                        BOARD_Can1Send(CAN_SEND_ID, data, 8); 
			
			break;
		case CMD_CLIB_CTRL:
			if (data[2] == 1) //
			{
				motEncCalibrateExt();
			}
			else if (data[2] == 2) //
			{
				motEncClibrateStop();
			}
			else if (data[2] == 3) //
			{
				motEncClibrateData(GET_WORD(data[3], data[4]), GET_WORD(data[5], data[6]));
			}
			else if (data[2] == 4) //
			{
				motEncClibrateSaveData();
			}
			break;
		case CMD_CLIB_READ_ENCODE:
			if (data[2] == 5) //
			{
				SET_DWORD(get_clib_encode(0), data[3], data[4], data[5], data[6]);
				//  SET_DWORD(DRV8353_Status(), data[4], data[5],data[6],data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (data[2] == 6) //
			{
				SET_DWORD(get_clib_encode(1), data[3], data[4], data[5], data[6]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (data[2] == 7) //
			{
				SET_DWORD(g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][data[3]], data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			else if (data[2] == 8)
			{
				SET_DWORD(g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[data[3]], data[4], data[5], data[6], data[7]);
				BOARD_CanSend(CAN_SEND_ID, data, 8);
			}
			break;
		case CMD_ZHISHAN_READ_VER:
			data[1] = mot_id;
			SET_WORD(0x4851, data[2], data[3]);
			SET_WORD(APP_VER, data[4], data[5]);
			BOARD_CanSend(CAN_SEND_ID, data, 8);
			break;
		case CMD_READ_ALARM_STR:
			reportAlarmStr(data[1], data[3], data[2]);
			break;

		case CMD_TAKE_DEBUG_BEGING:
#ifdef TAKE_DEEBUG_OPEN
			take_debug_begin(data[2]);
#endif
			break;
		case CMD_TAKE_DEBUG_PRINT:
#ifdef TAKE_DEEBUG_OPEN
			easyCanDebugPrint();
#endif
			break;
		}
            }
}
void easyCanProc1(void)
{
        uint8_t data[8];
	uint32_t canid;
	uint8_t len;
	 
	if (easyCanPop1(data, &canid, &len) == 0)
	{
		can_packet_parse(data,canid,len,1) ;
	}
}
void easyCanProc(void)
{
	 
	uint8_t data[8];
	uint32_t canid;
	uint8_t len;
	 
	if (easyCanPop(data, &canid, &len) == 0)
	{
		can_packet_parse(data,canid,len,0) ;
	}
}
