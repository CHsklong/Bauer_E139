

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "SysMain.h"

#define  USE_TOR_G    1

CAN_Data    can_rcv;
CAN_SEND_FLAG_TYPEDEF  	can_send_flag ;
CAN_CMD_RCV_TYPEDEF		can_pos_rcv;
CAN_DELT_CMD_TYPEDEF    can_delt_cmd;

FUNC_PARA_TYPEDEF 	funcPar;

// 取送纱器ID: 0 -- 5
ATTR_RAMFUNC int YC_get_ID()
{
	return can_send_flag.self_id;
}

ATTR_RAMFUNC int YC_Set_YarnTensionIn(char val)
{
}

ATTR_RAMFUNC int YC_Set_YarnTensionOut(char val)
{
}

ATTR_RAMFUNC int YC_Calibration_Sensor(int onoff)
{
}

ATTR_RAMFUNC int YC_Calibration_Swing(int onoff)
{
}

ATTR_RAMFUNC int YC_Calibration_Motor(int onoff)
{
}

ATTR_RAMFUNC int YC_SendData(void *data, int dlen)
{
}

ATTR_RAMFUNC int YC_YarnMeasure_start()
{
}

ATTR_RAMFUNC int YC_YarnMeasure_stop()
{
}

ATTR_RAMFUNC unsigned long YC_YarnMeasure_length()
{
}


// 主动轮直径
static unsigned long yc_diam = 0;
// 纱线支数
static unsigned long yc_yarnsize = 0;
// 机号 针数每10英寸
static int yc_NeedlePer10Inch = 0;
static unsigned long yc_YarnTensionIn = 0;
static unsigned long yc_YarnTensionOut = 0;
static int yc_yarnrange_phase = 0;
static int yc_yarnrange_start = 0;
static int yc_yarnrange_end = 0;

static int yc_workmode = 0;
int16_t yc_rundir = 0;
int32_t yc_runspd = 0;
int16_t yc_ndlpos ;
int16_t yc_ndlpos_old;
int16_t yc_ndlpos_inc;

int8_t  yc_start_flag = 0;
int8_t  yc_stop_flag = 0;

int8_t  yc_start_flag_old = 0;
int8_t  yc_stop_flag_old = 0;

ATTR_RAMFUNC void yc_pos_judge_edge(void)
{
	if (yc_ndlpos > yc_yarnrange_start)
	{
		yc_start_flag = 1;
	} else {
		yc_start_flag = 0;
	}

	if (yc_ndlpos > yc_yarnrange_end)
	{
		yc_stop_flag = 1;
	} else {
		yc_stop_flag = 0;
	}

}

ATTR_RAMFUNC int YC_SetKnitData(int start, int end, int mode, int yspd)
{
	yc_workmode = mode;
	switch (mode) {
	case 0x0: // 复位
		break;
	case 0x1: // 学习
		break;
	case 0x2: // 编织
		break;
	}
}

ATTR_RAMFUNC int YC_SendKnitState()
{
	char data[8];
	data[0] = 7;
	data[1] = YC_get_ID();
	data[2] = yc_workmode;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	YC_SendData(data, 8);
}

ATTR_RAMFUNC int YC_SendYarnLength()
{
	unsigned long ylen = YC_YarnMeasure_length();
	char data[8];

	data[0] = 8;
	data[1] = YC_get_ID();
	data[2] = (ylen >> 0) & 0xFF;
	data[3] = (ylen >> 8) & 0xFF;
	data[4] = (ylen >> 16) & 0xFF;
	data[5] = (ylen >> 24) & 0xFF;
	data[6] = 0;
	data[7] = 0;
	YC_SendData(data, 8);
}

ATTR_RAMFUNC int16_t YC_RunData(int16_t dir, int16_t ndlpos, int16_t rspd)
{
	if (yc_rundir != dir) {
		YC_SendKnitState();
	}
	yc_rundir = dir;
	yc_runspd = rspd;
	yc_ndlpos = ndlpos;

	//------------------------------
	yc_ndlpos_inc = yc_ndlpos - yc_ndlpos_old;

	yc_ndlpos_old = yc_ndlpos ;

	//-----------------------------

	if (yc_yarnrange_phase == 1) {
		if (ndlpos == yc_yarnrange_start) {
			YC_YarnMeasure_start();
			yc_yarnrange_phase = 2;
		}
	}
	if (yc_yarnrange_phase == 2) {
		if (ndlpos == yc_yarnrange_end) {
			YC_YarnMeasure_stop();
			YC_SendYarnLength();
			yc_yarnrange_phase = 0;
		}
	}
}

ATTR_RAMFUNC int YC_SetYarnRange(int start, int end)
{

	yc_yarnrange_start = start;
	yc_yarnrange_end = end;
	yc_yarnrange_phase = 1;
	mach_spd.needle_range = abs(yc_yarnrange_start - yc_yarnrange_end );

}
ATTR_RAMFUNC  void set_fore_id(void)
{

	switch (can_send_flag.self_id )
	{
	case 1:
		if (can_send_flag.use_id_flag & 0x01)
		{
			can_send_flag.first_id = can_send_flag.self_id;

		} else {

			can_send_flag.first_id = 0;
		}
		can_send_flag.pre_id = 0;
		break;
	case 2:
		if (can_send_flag.use_id_flag & 0x02)
		{
			can_send_flag.first_id = can_send_flag.self_id;

			if (can_send_flag.use_id_flag & 0x01)
			{
				can_send_flag.first_id = 1;
				can_send_flag.pre_id = 1;
			}

		} else {

			can_send_flag.first_id = 0;
			can_send_flag.pre_id = 0;
		}

		break;
	case 3:
		if (can_send_flag.use_id_flag & 0x03)
		{
			can_send_flag.first_id = can_send_flag.self_id;

			if (can_send_flag.use_id_flag & 0x02)
			{
				can_send_flag.first_id = 2;
				can_send_flag.pre_id = 2;

			} else if (can_send_flag.use_id_flag & 0x01)
			{
				can_send_flag.first_id = 1;
				can_send_flag.pre_id = 1;
			}

		} else {

			can_send_flag.first_id = 0;
			can_send_flag.pre_id = 0;
		}
		break;
	case 4:
		if (can_send_flag.use_id_flag & 0x04)
		{
			can_send_flag.first_id = can_send_flag.self_id;

			if (can_send_flag.use_id_flag & 0x03)
			{
				can_send_flag.first_id = 3;
				can_send_flag.pre_id = 3;
			} else if (can_send_flag.use_id_flag & 0x02)
			{
				can_send_flag.first_id = 2;
				can_send_flag.pre_id = 2;

			} else if (can_send_flag.use_id_flag & 0x01)
			{
				can_send_flag.first_id = 1;
				can_send_flag.pre_id = 1;
			}

		} else {

			can_send_flag.first_id = 0;
			can_send_flag.pre_id = 0;
		}
		break;
	//===+++++++++++++-----------------------------------
	case 5:
		if (can_send_flag.use_id_flag & 0x10)
		{
			can_send_flag.first_id = can_send_flag.self_id;

		} else {


			can_send_flag.first_id = 0;
		}
		can_send_flag.pre_id = 0;
		break;
	case 6:
		if (can_send_flag.use_id_flag & 0x20)
		{
			can_send_flag.first_id = can_send_flag.self_id;

			if (can_send_flag.use_id_flag & 0x10)
			{
				can_send_flag.first_id = 5;
				can_send_flag.pre_id = 5;
			}

		} else {

			can_send_flag.first_id = 0;
			can_send_flag.pre_id = 0;
		}
		break;
	case 7:
		if (can_send_flag.use_id_flag & 0x30)
		{
			can_send_flag.first_id = can_send_flag.self_id;

			if (can_send_flag.use_id_flag & 0x20)
			{
				can_send_flag.first_id = 6;
				can_send_flag.pre_id = 6;

			} else if (can_send_flag.use_id_flag & 0x10)
			{
				can_send_flag.first_id = 5;
				can_send_flag.pre_id = 5;
			}

		} else {

			can_send_flag.first_id = 0;
			can_send_flag.pre_id = 0;
		}
		break;
	case 8:
		if (can_send_flag.use_id_flag & 0x40)
		{
			can_send_flag.first_id = can_send_flag.self_id;

			if (can_send_flag.use_id_flag & 0x30)
			{
				can_send_flag.first_id = 7;
				can_send_flag.pre_id = 7;
			} else if (can_send_flag.use_id_flag & 0x20)
			{
				can_send_flag.first_id = 6;
				can_send_flag.pre_id = 6;

			} else if (can_send_flag.use_id_flag & 0x10)
			{
				can_send_flag.first_id = 5;
				can_send_flag.pre_id = 5;
			}

		} else {

			can_send_flag.first_id = 0;
			can_send_flag.pre_id = 0;
		}
		break;
	}

}

ATTR_RAMFUNC int YC_DoCmd(CAN_Data *pkg)
{
	int8_t temp8 ;
	int16_t temp16;
	int32_t temp32;

	switch (pkg->data[0]) {

	case 0x1: // 滚轮直径
		yc_diam = pkg->data[1] * 100L + pkg->data[2 + YC_get_ID()];
		break;
	case 0x2: // 纱嘴支数
		yc_yarnsize = (pkg->data[2] << 8) | ((unsigned char)pkg->data[1]);
		yc_NeedlePer10Inch = (pkg->data[4] << 8) | ((unsigned char)pkg->data[3]);
		break;
	case 0x3: // 校准
	{
		int mask = 1 << YC_get_ID();
		YC_Calibration_Sensor(pkg->data[1] & mask);
		YC_Calibration_Swing(pkg->data[2] & mask);
		YC_Calibration_Motor(pkg->data[3] & mask);
	}
	break;
	case 0x4: // 进纱张力
		yc_YarnTensionIn = pkg->data[1 + YC_get_ID()];
		break;
	case 0x5: // 出纱张力
		yc_YarnTensionOut = pkg->data[1 + YC_get_ID()];
		break;
	case 0x6: // 运行数据
	{
		//int ndlpos = (pkg->data[2] << 8) | ((unsigned char)pkg->data[1]);
		//int rspeed = (pkg->data[4] << 8) | ((unsigned char)pkg->data[3]);
		//int dir = pkg->data[5];
		int16_t ndlpos = (pkg->data[3] << 8) | (uint8_t) (pkg->data[2]);
		int16_t rspeed = (int16_t)((int16_t)(pkg->data[5] << 8) | (uint8_t)(pkg->data[4]));
		int16_t dir = pkg->data[6];
		YC_RunData(dir, ndlpos, rspeed);
	}
	break;
	case 0x7: // 编织数据
		if (pkg->data[1] != YC_get_ID())
			break;
		{
			int mode = pkg->data[2];
			int yspeed = pkg->data[3];
			int start = (pkg->data[5] << 8) | ((unsigned char)pkg->data[4]);
			int end = (pkg->data[7] << 8) | ((unsigned char)pkg->data[6]);
			mach_spd.line_num = end;
			YC_SetKnitData(start, end, mode, yspeed);
		}
		break;
	case 0x8: // 读取纱长数据
		//if(pkg->data[1] != YC_get_ID())
		//	break;
	{
		int start = (pkg->data[3] << 8) | ((unsigned char)pkg->data[2]);
		int end = (pkg->data[5] << 8) | ((unsigned char)pkg->data[4]);
		//int mode = pkg->data[6];
		YC_SetYarnRange(start, end);
	}
	break;
	case 0x9: // 读取纱长数据
		if (pkg->data[1] != YC_get_ID())
			break;
		{
			outVolt = (pkg->data[3] << 8) | ((unsigned char)pkg->data[2]);


		}
		break;
	case 0x10: // 读取纱长数据
		can_send_flag.cmd = pkg->data[0] ;
		can_send_flag.enable = 1;
		len_meas.cap_len_mm  = len_meas.send402_len_mm ; //len_meas.mt_enc_add_mm;
		break;
	case 0x11:// write纱长数据

		if ((pkg->data[1] == YC_get_ID()) || (pkg->data[1] == 0x0f))
		{

			can_send_flag.enable = 1;
			can_send_flag.cmd = pkg->data[0] ;

			temp32 = (pkg->data[5] << 24) | (uint8_t)(pkg->data[4] << 16) | (pkg->data[3] << 8) | (pkg->data[2]) ;


			len_meas.motor_len_mm = temp32;
			len_meas.send402_len_mm = temp32;
			len_meas.mt_enc_add64 = len_meas.motor_len_mm * len_meas.Kmm2p ;
			// len_meas.mt_enc_add64 = 0;
			len_meas.send402_len_mm = len_meas.motor_len_mm ;




			can_pos_rcv.yarn_cmd_mm = 0;
			can_pos_rcv.plus64_old = 0;
			can_pos_rcv.plus64 = 0;

			

		}
		break;




	default:
		break;


	}

}


