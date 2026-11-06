#include <stdint.h>
#include <float.h>
#include "board.h"
#include "alert.h"
#include "motor.h"
#include "tick.h"
#include "m1_sm_ref_sol.h"
#include "servo_can.h"
#include "machine.h"
#include "mcdrv.h"
#include "pwm.h"
//--------------auto report manage-----------------------------

#ifdef WIRE_MOTOR_FUN //��˿���ܴ���   
ALT_STR alt_cn[ALT_MAX_STR_NUM]={
	{7,"��������λ"},
	{8,"��������λ"},
	{9,"�������㳬ʱ"},
	{10,"����δ�ҵ���"},
	{17,"��������"},
	{18,"������ת"},
	{21,"��������"},
	{22,"������ѹ"},
	{32,"��������"},
	{36,"����Ƿѹ"},
	{34,"��������"},
	{51,"��λ����������"},
	{52,"��е������ת��������С"},
	{53,"��Ȧ̫����������"},
	{54,"�������Ƿ���˿"},
	{55,"��˿̫��"},
	{56,"��λ������ĩ�˴�������"},
	{57,"δ������λ��ĩ��λ��"},
	{58,"��˿��λ"},
	{59,"��˿��λ"},
	{60,"������λ����"},
	{61,"��λƫ����������λ���ô�"},
	{62,"ĩ�˴�������Ч"},
	{63,"��λ��������Ч��"},
    {64,"�������Ƿ���˿�͸�˿ѹ��û��ѹ��"},
	
};
ALT_STR alt_en[ALT_MAX_STR_NUM]={
	{7,"ALT_LEFT_LIMIT"},
	{8,"ALT_RIGHT_LIMIT"},
	{9,"ALT_ZERO_TIMEOUT"},
	{10,"ALT_NOT_FIND_SIZE"},
	{17,"ALT_MOT_OVER_CURRENT"},
	{18,"ALT_MOT_ROTOR_BLOCKED"},
	{21,"ALT_MOT_OVER_LOAD"},
	{22,"ALT_MOT_OVER_DCBUS"},
	{32,"ALT_MOT_OVER_SPEED"},
	{36,"ALT_MOT_UNDER_DCBUS"},
	{34,"ALT_SYNC_ERROR"},
	{51,"NO_FIND_ZERO_SIGNAL"},
	{52,"BELT_BROKEN"},
	{53,"NO_FIND_SAFE_SIGNAL"},
	{54,"SAFE_SIGNAL_OFFSET_ERROR"},
	{55,"SAFE_SIGNAL_OR_POS_ERROR"},
	{56,"MOVE_SAFE_TORQUE_SMALL"},
	{57,"BALANCE_TOO_BIG"},
	{58,"TORQUE_OVER_SPEED"},
	{59,"ZERO_SIGANL_ERROR"},
	{60,"DOSSAL SET ERROR"},
	{61,"BLOCK OR DOSSAL SET ERROR"},
 	{62,"END SERSON ERROR"},
	{63,"NO_FIND_ZERO_SIGNAL"},
    {64,"BROKEN WIRE"},
};
#else
ALT_STR alt_cn[ALT_MAX_STR_NUM]={
	{7,"��������λ"},
	{8,"��������λ"},
	{9,"�������㳬ʱ"},
	{10,"����δ�ҵ���"},
	{17,"��������"},
	{18,"������ת"},
	{21,"��������"},
	{22,"������ѹ"},
	{32,"��������"},
	{36,"����Ƿѹ"},
	{34,"��������"},
	{51,"��е����δ�ҵ���λ"},
	{52,"ͬ�����ϻ�ͬ������"},
	{53,"û���ҵ���ȫλ�ź�"},
	{54,"������ȫλ�ݲ�"},
	{55,"ȫλ�źŻ�λ�ô���"},
	{56,"�ƶ�����ȫλ��ع�С"},
	{57,"ƽ��������ù���"},
	{58,"δ�����"},
	{59,"��λ��������һֱ��"},
	{60,"�Ҳ�λ���ô���"},
	{61,"��е�����Ҳ�λ���ô�"},
	{62,"��ȫλ��������һֱ��"},
	{63,"�����ȫλ���Ʊ���"},
	
};
ALT_STR alt_en[ALT_MAX_STR_NUM]={
	{7,"ALT_LEFT_LIMIT"},
	{8,"ALT_RIGHT_LIMIT"},
	{9,"ALT_ZERO_TIMEOUT"},
	{10,"ALT_NOT_FIND_SIZE"},
	{15,"ALT_NOT_BIND"},
	{17,"ALT_MOT_OVER_CURRENT"},
	{18,"ALT_MOT_ROTOR_BLOCKED"},
	{21,"ALT_MOT_OVER_LOAD"},
	{22,"ALT_MOT_OVER_DCBUS"},
	{32," ALT_MOT_OVER_SPEED"},
	{36,"ALT_MOT_UNDER_DCBUS"},
	{34,"ALT_SYNC_ERROR"},
	{51,"NO_FIND_ZERO_SIGNAL"},
	{52,"BELT_BROKEN"},
	{53,"NO_FIND_SAFE_SIGNAL"},
	{54,"SAFE_SIGNAL_OFFSET_ERROR"},
	{55,"SAFE_SIGNAL_OR_POS_ERROR"},
	{56,"MOVE_SAFE_TORQUE_SMALL"},
	{57,"BALANCE_TOO_BIG"},
	{58,"TORQUE_OVER_SPEED"},
	{59,"ZERO_SIGANL_ERROR"},
	{60,"DOSSAL SET ERROR"},
	{61,"BLOCK OR DOSSAL SET ERROR"},
	{62,"JM_MSG_SIGANL_SAFE_ERROR"},
	{63,"JM_MSG_PULL_SAFE_ERROR"},
};
#endif 

void reportAlarmStr(uint8_t mot_id, uint8_t str_type, uint8_t alart_no)
{
	ALT_STR *pStr;
	uint8_t cmd[8];
	uint8_t i = 0;
	uint8_t pack_idx = 0, len = 5;
	if (str_type == 0) //
	{
		pStr = alt_cn;
	}
	else
		pStr = alt_en;
	cmd[0] = CMD_READ_ALARM_STR;
	cmd[1] = mot_id; // motor ID

	for (i = 0; i < ALT_MAX_STR_NUM; i++)
	{
		if (alart_no == pStr[i].alt_no)
		{

			uint8_t send_len = MAX_CHAR;
			uint8_t str_ptr = 0;
			while (send_len > 0)
			{
				cmd[2] = (pack_idx << 4) | len;
				memcpy(&cmd[3], &(pStr[i].str[str_ptr]), len);
				send_len = send_len - len;
				pack_idx++;
				str_ptr += len;

				BOARD_CanSend(CAN_SEND_ID, cmd, 8);
			}
			break;
		}
	}
}
void reportMotorCurrent(int16_t pos ,int16_t current) 
{
	uint8_t cmd[8];
 

  	cmd[0] = CMD_RD_PARAM;
  	cmd[1] = mot_id;  //motor ID
 	SET_WORD(0x8703, cmd[2], cmd[3]) ;
  	SET_WORD(pos,cmd[4],cmd[5]) ;
 	SET_WORD(current,cmd[6],cmd[7]) ;
  	BOARD_CanSend(CAN_SEND_ID, cmd,8);
}
void reportCmd(uint8_t cmd)
{
	uint8_t data[2];
	data[0] = cmd;
	data[1] = mot_id; // motor ID

	BOARD_CanSend(CAN_SEND_ID, data, 2);
}
void servo_send_over_zero(uint16_t value)
{
        uint8_t cmd[8];
	// int32_t pos = posIn2Out(getCurPos());
        
      	cmd[0] = CMD_RD_PARAM;
	cmd[1] = mot_id;					// motor ID
	cmd[2] = 0x57; //
	cmd[3] = 0x80;							//
	SET_WORD(value, cmd[4], cmd[5]);
 
	BOARD_CanSend(CAN_SEND_ID, cmd, 6);
} 
void reportState(uint16_t canid, uint8_t motor_id,uint8_t can_type)
{
	uint8_t cmd[8];
	uint8_t alertNo = 0;
	uint8_t signal;
	int32_t pos = 0 ; 
        if(can_type ==0)
        {
            pos= posIn2Out(getCurPos());
        }
        else
        {
            pos= -((getCurPos() - mot_run_state.zeroOffset) * 16384.0f * mot_run_state.encDir / 360.0f) ;
        }
	alertNo = alertGet();
	signal = mot_run_state.signal;

	cmd[0] = CMD_RD_STATUS;
	cmd[1] = motor_id;															   // motor ID
	cmd[2] = alertNo;															   // alert
	cmd[3] = signal | (mot_run_state.zero_flag << 1) | mot_run_state.arr_pos_flag; // signal
	SET_DWORD(pos, cmd[4], cmd[5], cmd[6], cmd[7]);

        if(can_type ==0)
	BOARD_CanSend(canid, cmd, 8);
        else
        BOARD_Can1Send(canid, cmd, 8);
}
void alert_send(uint8_t alert_id, int32_t pos)
{
	uint8_t cmd[8];

	cmd[0] = CMD_ALARM_REPORT;
	cmd[1] = mot_id;   // motor ID
	cmd[2] = alert_id; // alert
	cmd[3] = 0;		   // signal
	SET_DWORD(pos, cmd[4], cmd[5], cmd[6], cmd[7]);
	BOARD_CanSend(CAN_SEND_ID, cmd, 8);
}
void reportAlert(uint8_t rank)
{
	uint8_t cmd[8];
	uint8_t alertNo = 0;

	alertNo = alertGet();
	if(alertNo == 0)
		return ;
	cmd[0] = CMD_ALARM_REPORT;
	cmd[1] = mot_id;        //motor ID
	cmd[2] = alertNo;       //alert
	cmd[3] = rank;        //signal
	BOARD_CanSend(CAN_SEND_ID, cmd, 4);
}
void reportAck(void)
{
	uint8_t cmd[8];

	cmd[0] = CMD_MOVE_POS_ACK_REPORT;
	cmd[1] = mot_id; // motor ID

	SET_DWORD(mot_run_state.move_pos_dis, cmd[2], cmd[3], cmd[4], cmd[5]);

	BOARD_CanSend(CAN_SEND_ID, cmd, 6);
}
void reportJkMotorStatus(uint32_t status)
{
	uint8_t cmd[8];

	cmd[0] = CMD_RD_PARAM;
	cmd[1] = mot_id; // motor ID
	SET_WORD(0x8602, cmd[2], cmd[3]);
	SET_DWORD(status, cmd[4], cmd[5], cmd[6], cmd[7]);

	BOARD_CanSend(CAN_SEND_ID, cmd, 8);
}

// retTime (ms)
void reportStart(uint16_t repTime)
{

	// mot_run_state.repStTime = repTime ;
	mot_run_state.repStTime = repTime;
}
void reportDebug(uint16_t data1,uint16_t data2,uint16_t data3) 
{
	uint8_t cmd[8];
  	cmd[0] = CMD_DEBUG_PRINT;
  	cmd[1] = mot_id;  //motor ID
	SET_WORD(data1, cmd[2], cmd[3]);
	SET_WORD(data2, cmd[4], cmd[5]);
	SET_WORD(data3, cmd[6], cmd[7]);	

  	BOARD_CanSend(CAN_SEND_ID, cmd, 8);
}
void reportPos(int32_t pos)
{
	uint8_t cmd[8];
	// int32_t pos = posIn2Out(getCurPos());

	cmd[0] = CMD_COND_RESP;
	cmd[1] = mot_id;					// motor ID
	cmd[2] = mot_run_state.report_type; //
	cmd[3] = 0;							//
	SET_DWORD(pos, cmd[4], cmd[5], cmd[6], cmd[7]);

	BOARD_CanSend(CAN_SEND_ID, cmd, 8);
}
float g_cur_pos = 0;
void reportTask(void)
{
	static uint32_t tickstart = 0;

	g_cur_pos = posIn2Out(getCurPos());
	// auto report state
	if (mot_run_state.repStTime != 0)
	{
		if (expireTicks(tickstart, mot_run_state.repStTime))
		{
			tickstart = getTicks();
			reportState(CAN_SEND_ID, mot_id,0);
			// reportAlert() ;
		}
	}
	else
	{
		tickstart = getTicks();
	}

	if ((g_cur_pos > mot_run_state.repUpPos && mot_run_state.report_type == 2) || ((g_cur_pos < mot_run_state.repDnPos && mot_run_state.report_type == 3)))
	{
		reportPos(posIn2Out(getCurPos()));
		mot_run_state.report_type = 0;
	}

	if (mot_run_state.target_pos_offset != 0)
	{
		uint8_t flag = 0;
		uint32_t curTicks = getTicks();
		if (fabs(getCurPos() - mot_run_state.posReq) < mot_run_state.target_pos_offset)
		{

			mot_run_state.move_pos_dis = ((curTicks >= mot_run_state.move_pos_begin) ? (curTicks - mot_run_state.move_pos_begin) : (0xffffffff - mot_run_state.move_pos_begin + curTicks));
			flag = 1;
		}
		/*if(mot_run_state.posReq >= g_cur_pos)
		{
			if((mot_run_state.posReq - g_cur_pos) < mot_run_state.target_pos_offset)
			{
					mot_run_state.move_pos_dis= ((curTicks >= mot_run_state.move_pos_begin) ? (curTicks-mot_run_state.move_pos_begin) : (0xffffffff-mot_run_state.move_pos_begin+curTicks));
					flag =1 ;
			}
		}
		else
		{
			   if((g_cur_pos -mot_run_state.posReq  ) < mot_run_state.target_pos_offset)
			{
					mot_run_state.move_pos_dis= ((curTicks >= mot_run_state.move_pos_begin) ? (curTicks-mot_run_state.move_pos_begin) : (0xffffffff-mot_run_state.move_pos_begin+curTicks));
					flag =1 ;
			}
		}*/

		if (flag)
		{
			if (mot_run_state.target_pos_ack_flag)
				reportAck();
			mot_run_state.arr_pos_flag = 1;
			mot_run_state.target_pos_ack_flag = 0;
			mot_run_state.target_pos_offset = 0;
		}
	}

	// check left/right limited
	/* 	if(g_sM1Drive.sPosCtrl.fltPositionFilt < mot_run_state.leftLmt
		 && g_sM1Drive.sSpeed.fltSpeedFilt < 0)
	   {
			moveStop(false);
			alertPush(ALT_LEFT_LIMIT,0);
	   }
	   else if(g_sM1Drive.sPosCtrl.fltPositionFilt > mot_run_state.rightLmt
		 && g_sM1Drive.sSpeed.fltSpeedFilt > 0)
	   {
		   moveStop(false);
		   alertPush(ALT_RIGHT_LIMIT,0);
	   }*/
}

//------------------alert manange --------------------------
#define ALT_BUF_LEN (10)

static bool alt_buffer[ALT_MAX] = {false};

int alertProc(uint8_t alertNo)
{
	switch (alertNo)
	{
	case ALT_MOT_OVER_CURRENT:
		// moveStop(true);
		break;

	default:
		// moveStop(false);
		break;
	}
	return 0;
}

int alertPush(uint8_t alertNo,uint8_t rank)
{

	if (alertNo < ALT_MAX && alt_buffer[alertNo] == false && alertNo != 0)
	{
		alt_buffer[alertNo] = true;
		alertProc(alertNo);
		// reportState();
		reportAlert(rank);
	}

	return true;
}

uint8_t alertPop(void)
{
	for (int i = 0; i < ALT_MAX; i++)
	{
		if (alt_buffer[i])
		{
			alt_buffer[i] = false;
			return i;
		}
	}

	return ALT_NONE;
}

uint8_t alertGet(void)
{
	for (int i = 0; i < ALT_MAX; i++)
	{
		if (alt_buffer[i])
			return i;
	}

	return ALT_NONE;
}

void alertClear(void)
{
	for (int i = 0; i < ALT_MAX; i++)
	{
		alt_buffer[i] = false;
	}
	pwm_clear_all_fault();
}
