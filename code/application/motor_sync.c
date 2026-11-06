#include "motor_sync.h"
#include "motor.h"
#include "math.h"
#include "mcdrv.h"
#include "alert.h"
typedef struct
{
	int Follow_deviation;
	int Start_Position;

} MOTOR_SYNC_t;

// static int pos;
MOTOR_SYNC_t Sync_Set_Inif;
float old_pos, pos, gap_position, old_gap_position, set_pos;

char in_follow_flage = 0, in_first_sync_flage = 0;
static char follow_dir;
int speed, set_speed;
int sync_stop_pos ;

int debug_ptr = 0;
int debug_buff[5];
int start_acc_scale = 1;
int speed_gap_scale = 1;
int16_t trp;
static float my_pos;
static float Start_Position, sync_offset; //
float ratio;
uint8_t start_lock = 0;
int follow_error_limit = 220;
SYNC_SET sync_set;
#define ARRY_MAX_NUM 10
int gap_position_arry[ARRY_MAX_NUM];
int sub;
#define AD_BUF_LEN 10 // 10
/*static uint32_t val[AD_BUF_LEN];
static uint16_t val_ptr = 0;
static uint16_t val_num = 0;*/
int32_t buf[AD_BUF_LEN];
int16_t m_speed = 2500;
void sync_set_follow_dwviation(int32_t val)
{
	Sync_Set_Inif.Follow_deviation = val ; //
}
int32_t sync_get_follow_dwviation(void)
{
	return Sync_Set_Inif.Follow_deviation;
}
void sync_set_follow_error_limit(int value)
{
	follow_error_limit = value;
}

void sync_follow_power(void)
{
	sync_set.Kp = 5.5; // 5.5
	sync_set.Ki = 0.5;
	sync_set.sync_start_speed = 0;
	sync_set.sync_stop_pos = SYNC_POS_INIT;
	sync_set.alarm_enable = 0;
	sync_set.sync_end_follow_spd_scale = 1.5;
	sync_set.sync_lead = 5.5;
	sync_set.sync_spd_ratio = 0.5;
	sync_set.adv_speed_scale = 1.1;
	sync_set.sync_adv_dis_scale = 0.1;
	sync_set.set_stop_pos_first_flag = 1;
	/*Sync_Set_Inif.Follow_deviation = 240 ;
	pos = 238565 ;
	sync_set.sync_cur_ext_pos = 232504 ;
	Start_Position = 229680 ;
	sync_set.alarm_enable  =1 ;
	if(Sync_Set_Inif.Follow_deviation !=0 && (fabs(pos -sync_set.sync_cur_ext_pos)) > Sync_Set_Inif.Follow_deviation && pos <(Start_Position ) && sync_set.alarm_enable ==1)
	{
		sync_set.alarm_enable =0 ;
	}*/
}

void sync_follow_init(void)
{
	in_follow_flage = 0;
	in_first_sync_flage = 0;
	old_pos = pos;
	g_sM1Drive.sPosCtrl.fltAdvSpeed = 0;
	sync_set.sync_max_acc_speed = m_speed;
	sync_set.follow_cnt = 0;
	// sync_set.sync_adv_max_pos = 500 * Virtual_Ratio ;
	sync_set.sync_adv_max_pos = 500;
}
void sync_set_pid(uint8_t type, float value)
{
	switch (type)
	{
	case PID_P:
		sync_set.Kp = value;
		break;
	case PID_I:
		sync_set.Ki = value;
		break;
	case PID_D:
		sync_set.Kd = value;
		break;
	case END_FOLLOW_SPD_SCALE:
		sync_set.sync_end_follow_spd_scale = value;
	default:
		break;
	}
}
void sync_set_debug_idx(uint8_t type, uint8_t value)
{
	if (type == 0)
	{
		sync_set.sync_debug_idx[0] = value;
	}
	else
	{
		sync_set.sync_debug_idx[1] = value;
	}
}
int pid_acc_speed_interp(void)
{
	sync_set.err = sync_set.sync_set_pos - sync_set.sync_pos;
	sync_set.sync_acc_speed = sync_set.Kp * sync_set.err + sync_set.Ki * sync_set.integral + sync_set.Kd * (sync_set.err - sync_set.err_last);
	sync_set.err_last = sync_set.err;

	if (sync_set.sync_acc_speed > sync_set.sync_max_acc_speed)
		sync_set.sync_acc_speed = sync_set.sync_max_acc_speed;
	else if (sync_set.sync_acc_speed < -sync_set.sync_max_acc_speed)
		sync_set.sync_acc_speed = -sync_set.sync_max_acc_speed;
	else
		sync_set.integral += sync_set.err;

	return sync_set.sync_acc_speed;
}
/*static void pid_acc_speed_clear(void)
{
	sync_set.err = 0;
	sync_set.err_last = 0;
	sync_set.integral = 0;
}*/
void sync_in_follow_type(void)
{
	if (in_first_sync_flage)
		return;
	in_first_sync_flage = 1;
	/*Servo_Set_RunMode(MP6570_POSITION_MODE);
	SetPositionMode(0);
	SetSpeedIQLimit((float)MAX_Limit/1000);
	SetPositionSlope(3000, 18,18);*/
}

void sync_lock(void)
{
	start_lock = 1;
}
void sync_un_lock(void)
{
	start_lock = 0;
}
void sync_end_follow(int pos)
{
	in_follow_flage = 0;
	g_sM1Drive.sPosCtrl.fltAdvSpeed = 0;
	// position_adv_speed = 0 ;
	// sync_set.sync_stop_pos = pos *Virtual_Ratio ;
	if (sync_set.set_stop_pos_first_flag == 1)
		sync_set.set_stop_pos_first_flag = 0;
	sync_set.sync_stop_pos = pos;
}
void sync_set_stop_pos(int pos)
{
	// sync_set.sync_stop_pos = pos *Virtual_Ratio ;
	sync_set.sync_stop_pos = pos;
}
void sync_start_follow(int16_t offset, int start_pos, char dir)
{

	start_lock = 1;
	sync_in_follow_type();
	sync_offset = posRatio2In2Out(offset);
	//	Start_Position= posRatioOut2In(start_pos+sync_offset);
	Start_Position = start_pos + sync_offset;
	sync_set.alarm_enable = 1;
	sync_set.sync_start_pos = start_pos;
	my_pos = posIn2Out(getCurPos());
	if (sync_set.set_stop_pos_first_flag == 1)
	{
		sync_set.sync_stop_pos = my_pos;
		sync_set.set_stop_pos_first_flag = 0;
	}
	sync_set.sync_first_acc = 0;
	if (dir)
	{
		follow_dir = CCW;
	}
	else
	{
		follow_dir = CW;
	}
	in_follow_flage = 1;

	// av_set_max_num(40) ;
	// av_clear_data() ;
	Servo_PID_Change(PID_SYNC) ;
	start_lock = 0;
}

void sync_end_pos(int end_pos)
{
	//	static int16_t  av_sync_current =0 ;
	start_lock = 1;
	in_follow_flage = 2;
	my_pos = posIn2Out(getCurPos());
	sync_stop_pos = end_pos;
	sync_set.sync_stop_pos = sync_stop_pos;
	start_lock = 0;
}
float sync_get_ext_pos(void)
{
	return posIn2Out(getCurPos());
}

int sync_get_shazui_pos(void)
{
	// return pos ;
	// return Start_Position *_POS_COEFFICIENT_OUT ;
	return Start_Position;
}

void sync_set_start_acc_scale(int value)
{
	start_acc_scale = value;
}
void sync_set_gap_scale(int value)
{
	speed_gap_scale = value;
}
void sync_deal_follow(void)
{
#if 0
	static u8 follow_stop = 0;
	if (in_follow_flage == 1)
	{
		if (sync_set.follow_cnt >= 300 && follow_stop == 0)
		{
			follow_stop = 1;
			MOTOR_Set_Info.set_speed = m_speed;
			SetPositionSlope(set_speed, 30, 30);
			SetPositionCommand_2(set_pos, MOTOR_Set_Info.dir_mask);
			StartMotor();
		}
		else
			sync_set.follow_cnt++;
	}
	else
	{
		sync_set.follow_cnt = 0;
		follow_stop = 0;
	}
#endif
}

#if 1
int pid_advance_pos_interp(char dir)
{
	if (dir == CW)
		sync_set.err = sync_set.sync_set_pos - sync_set.sync_pos;
	else
		sync_set.err = sync_set.sync_pos - sync_set.sync_set_pos;
	sync_set.sync_adv_pos = sync_set.Kp * sync_set.err + sync_set.Ki * sync_set.integral + sync_set.Kd * (sync_set.err - sync_set.err_last);
	sync_set.err_last = sync_set.err;

	if (sync_set.sync_adv_pos > sync_set.sync_adv_max_pos)
		sync_set.sync_adv_pos = sync_set.sync_adv_max_pos;
	else if (sync_set.sync_adv_pos < -sync_set.sync_adv_max_pos)
		sync_set.sync_adv_pos = -sync_set.sync_adv_max_pos;
	else
		sync_set.integral += sync_set.err;

	return sync_set.sync_adv_pos;
}
int sync_av_pos(int pos)
{
	static uint8_t add_trp = 0;
	static char flag = 0;
	int total_gap_pos = 0;
	int i = 0;
	gap_position_arry[add_trp++] = pos;
	if (add_trp >= ARRY_MAX_NUM)
	{
		flag = 1;
		add_trp = 0;
	}
	for (i = 0; i < ARRY_MAX_NUM; i++)
	{
		total_gap_pos += gap_position_arry[i];
	}
	if (flag == 1)
	{
		return total_gap_pos / ARRY_MAX_NUM;
	}
	else
		return pos;
}

/*int sync_lb(int pos)
{
	val[val_ptr] = pos;
	val_ptr++;
	if (val_ptr >= AD_BUF_LEN)
		val_ptr = 0;
	if (val_num < AD_BUF_LEN)
	{
		val_num++;
	}
}*/
void array_sort(uint32_t *src, uint32_t *dst, uint8_t len, uint8_t cmp)
{
	uint8_t i, j, min, max;
	int tmp;
	for (i = 0; i < len; i++)
	{
		dst[i] = src[i];
	}
	if (cmp > (len / 2))
		cmp = len / 2;
	for (i = 0; i < cmp; i++)
	{
		min = i;
		max = i;
		for (j = i + 1; j < len - i; j++)
		{
			if (dst[j] > dst[max])
			{
				max = j;
				continue;
			}
			if (dst[j] < dst[min])
			{
				min = j;
			}
		}
		if (max == min)
			return;
		tmp = dst[i];
		dst[i] = dst[min];
		dst[min] = tmp;
		tmp = dst[len - i - 1];
		dst[len - i - 1] = dst[max];
		dst[max] = tmp;
	}
}
#if 0
int sync_get_lb_pos(int pos)
{
	u8 i;
	u8 cmp = 2;
	u32 sum = 0;
	sync_lb(pos);
	if (val_num < AD_BUF_LEN) {
		return pos;
	}
	array_sort(val, buf, AD_BUF_LEN, cmp);
	for (i = cmp; i < (AD_BUF_LEN - cmp); i++)
	{
		sum += buf[i];
	}
	return sum / (AD_BUF_LEN - 2 * cmp);
}
#endif
#if 1
int sync_get_lb_pos(int gap_pos, int old_gap_pos)
{
	static uint8_t flag = 0;
	static int total_pos = 0;
	if (fabs(gap_pos - old_gap_pos) > 2)
	{
		if (flag == 0)
		{
			total_pos = old_gap_pos;
			flag = 1;
			return old_gap_pos;
		}
		else
		{
			total_pos += gap_pos;
			flag = 0;
			return total_pos / 2;
		}
	}
	else
	{
		return gap_pos;
	}
}
#endif
#if 0
int sync_get_lb_pos(int gap_pos, int old_gap_pos, int push_pos, int old_push_pos)
{
	static u8 flag = 0;
	static int total_pos = 0;
	if (abs(gap_pos - old_gap_pos) > 2)
	{
		if (flag == 0)
		{
			total_pos = old_push_pos;
			flag = 1;
			return old_push_pos;
		}
		else
		{
			total_pos += push_pos;
			flag = 0;
			return total_pos / 2;
		}
	}
	else
	{
		return push_pos;
	}
}
#endif
int sync_debug_para_values(uint8_t idx)
{

	return 0;
}
/*****1100 ==0.25  1400== 0.3  1700== 0.35 ********/

void sync_adv_dis_scale(int cur_speed)
{
#if 1
	if (cur_speed < 1400)
	{
		sync_set.sync_adv_dis_scale = 0.35;
	}
	else if (cur_speed < 1700)
	{
		sync_set.sync_adv_dis_scale = 0.35;
	}
	else
	{
		sync_set.sync_adv_dis_scale = 0.35;
	}
#endif
}

void sync_set_enable(uint8_t enable)
{
	sync_set.sync_enable = enable;
}

void sync_deal_can(int push_pos)
{
#if 1
	//char dir;
	//uint16_t acce;
	static uint8_t end_follow_flag = 0;

	//float sz_gap_position = 0 ;
	uint16_t start_spd = 800; // 500
	uint8_t alarm_flag = 0;
	sync_set.sync_cur_ext_pos = sync_get_ext_pos(); //
#if 1
	//	gap_position =  sync_get_lb_pos(abs(push_pos - old_pos) ,old_gap_position) ;
	//	gap_position = sync_av_pos(gap_position) ;
	gap_position = fabs(push_pos - old_pos);
#else
	sync_get_lb_pos(abs(push_pos - old_pos), old_gap_position, push_pos, old_pos);
	push_pos = sync_av_pos(push_pos);
	gap_position = abs(push_pos - old_pos);
#endif
	// speed =  ((( int )(gap_position * Virtual_Ratio*2000*60))>>16) ;
	speed = ((int)((posRatioOut2In(gap_position) * 2000 * 60) / 360.0f));
	if (speed >= 0)
	{
		m_speed = 2500;
		sync_set.abs_spd = speed;
	}
	else
	{
		m_speed = -2500;
		sync_set.abs_spd = -speed;
	}
	sync_set.sync_start_speed = sync_set.abs_spd;
	if (in_follow_flage == 0 || start_lock == 1 || sync_set.sync_enable == 0) //
	{
		goto end;
	}
	//	sz_gap_position = posOut2In(gap_position)  ;

	// speed =  ((( int )(gap_position * Virtual_Ratio*2000*60))>>16) ;

	if (in_follow_flage == 2) //
	{

		Servo_PID_Change(PID_SYNC) ;

		//	position_adv_speed = 0 ;
		g_sM1Drive.sPosCtrl.fltAdvSpeed = 0;
		if (end_follow_flag == 0)
		{
			sync_set.sync_end_follow_speed = sync_set.abs_spd * 1.5;
			end_follow_flag = 1;
		}
		if (follow_dir == CW)
		{
#if 0
			if (sync_set.sync_cur_ext_pos >= sync_stop_pos)
			{
				set_speed = m_speed;
				set_pos = sync_stop_pos;
				in_follow_flage = 0;

				goto motor_start;
			}
			else
			{
				set_pos = sync_stop_pos;
				//	set_speed = sync_set.sync_end_follow_speed ;
				set_speed = sync_set.sync_end_follow_speed * (1.0 - (float)(sync_set.sync_cur_ext_pos - my_pos) / (sync_stop_pos - my_pos));
				if (set_speed > sync_set.sync_end_follow_speed)
					set_speed = sync_set.sync_end_follow_speed;
				if (set_speed < 400 || alertGet()>0)
				{
					set_speed = sync_set.sync_end_follow_speed * sync_set.sync_end_follow_spd_scale;
					if (set_speed < 20)
						set_speed = 20;
					in_follow_flage = 0;
					goto motor_start;
				}
				else
				{

					moveSpd(set_speed / g_fltM1speedAngularScale, -1);

					goto end;

				}


			}
#else
			set_speed = sync_set.abs_spd;
			set_pos = sync_stop_pos;
			in_follow_flage = 0;

			goto motor_start;
#endif
		}
		else
		{
#if 0
			if (sync_set.sync_cur_ext_pos <= sync_stop_pos)
			{
				set_speed = m_speed;
				set_pos = sync_stop_pos;
				in_follow_flage = 0;
				goto motor_start;
			}
			else
			{
				set_pos = sync_stop_pos;
				//set_speed = sync_set.sync_end_follow_speed ;

				set_speed = sync_set.sync_end_follow_speed * (1.0 - (float)(my_pos - sync_set.sync_cur_ext_pos) / (my_pos - sync_stop_pos));
				if (set_speed > sync_set.sync_end_follow_speed)
					set_speed = sync_set.sync_end_follow_speed;
				if (set_speed < 400 || alertGet()>0)
				{
					set_speed = sync_set.sync_end_follow_speed * sync_set.sync_end_follow_spd_scale;
					in_follow_flage = 0;
					if (set_speed < 20)
						set_speed = 20;
					goto motor_start;
				}
				else
				{

					if (speed >= 0)
						moveSpd(-(set_speed / g_fltM1speedAngularScale), -1);
					else
						moveSpd(set_speed / g_fltM1speedAngularScale, -1);
					goto end;
				}


			}
#else
			set_speed = sync_set.abs_spd;
			set_pos = sync_stop_pos;
			in_follow_flage = 0;

			goto motor_start;
#endif
		}
	}

	if (in_follow_flage == 1) //
	{
		//int acc = 50;	  // pp/500us 91*65536/120000
		//int acc_rpm = 91; //rpm/500us 	0-3000转需要 33个500us 
		//int track_pos = 30;
		end_follow_flag = 0;
		sync_adv_dis_scale(sync_set.abs_spd);

		pos = push_pos + sync_offset;
		sync_set.sync_set_pos = pos;

		if (follow_dir == CW)
		{
			if (sync_set.abs_spd > start_spd)
				sync_set.sync_start_dis = sync_set.sync_adv_dis_scale * sync_set.abs_spd;
			else
				sync_set.sync_start_dis = 0;
			if (Sync_Set_Inif.Follow_deviation != 0 && (fabs(pos - sync_set.sync_cur_ext_pos)) > Sync_Set_Inif.Follow_deviation && pos > (Start_Position) && sync_set.alarm_enable == 1)
			{
				sync_set.alarm_enable = 0;
				alert_send(ALT_SYNC_ERROR, (int32_t)(fabs(pos - sync_set.sync_cur_ext_pos)));
				alarm_flag = 1;
			}
			else
				alarm_flag = 0;
			if (sync_set.sync_stop_pos != SYNC_POS_INIT && pos < (sync_set.sync_stop_pos - sync_set.sync_start_dis)) // ��ͷδ��ɳ��
			{
				goto end;
			}
			else
			{

				if (sync_set.sync_cur_ext_pos < Start_Position)
				{
					if (sync_set.sync_start_dis > 0)
					{
						if (fabs(getCurSpd(1)) < sync_set.abs_spd * sync_set.sync_spd_ratio && sync_set.sync_first_acc == 0 && alarm_flag == 0)
						{

							moveSpd((m_speed / g_fltM1speedAngularScale), -1);
							goto end;
						}
						else
						{
							sync_set.sync_first_acc = 1;
							g_sM1Drive.sPosCtrl.fltAdvSpeed = speed / g_fltM1speedAngularScale * sync_set.adv_speed_scale;
						}
					}
					else
					{
						sync_set.sync_first_acc = 1;
						g_sM1Drive.sPosCtrl.fltAdvSpeed = speed / g_fltM1speedAngularScale * sync_set.adv_speed_scale;
					}
				}
				else
				{
					g_sM1Drive.sPosCtrl.fltAdvSpeed = speed / g_fltM1speedAngularScale * sync_set.adv_speed_scale;
				}
				set_speed = sync_set.abs_spd * 3;

				set_pos = pos;
			}
		}
		else // ccw
		{
			if (sync_set.abs_spd > start_spd)
				sync_set.sync_start_dis = sync_set.sync_adv_dis_scale * sync_set.abs_spd;
			else
				sync_set.sync_start_dis = 0;

			if (Sync_Set_Inif.Follow_deviation != 0 && (fabs(pos - sync_set.sync_cur_ext_pos)) > Sync_Set_Inif.Follow_deviation && pos < (Start_Position) && sync_set.alarm_enable == 1)
			{
				sync_set.alarm_enable = 0;

				alert_send(ALT_SYNC_ERROR, (int32_t)(fabs(pos - sync_set.sync_cur_ext_pos)));
				alarm_flag = 1;
			}
			else
				alarm_flag = 0;
			if (sync_set.sync_stop_pos != SYNC_POS_INIT && pos > (sync_set.sync_stop_pos + sync_set.sync_start_dis))
			{
				goto end;
			}
			else
			{
				if (sync_set.sync_cur_ext_pos > Start_Position)
				{
					if (sync_set.sync_start_dis > 0)
					{
						if (fabs(getCurSpd(1)) < sync_set.abs_spd * sync_set.sync_spd_ratio && sync_set.sync_first_acc == 0 && alarm_flag == 0)
						{

							moveSpd(-(m_speed / g_fltM1speedAngularScale), -1);
							goto end;
						}
						else
						{
							sync_set.sync_first_acc = 1;
							g_sM1Drive.sPosCtrl.fltAdvSpeed = -speed / g_fltM1speedAngularScale * sync_set.adv_speed_scale;
						}
					}
					else
					{
						sync_set.sync_first_acc = 1;
						g_sM1Drive.sPosCtrl.fltAdvSpeed = -speed / g_fltM1speedAngularScale * sync_set.adv_speed_scale;
					}
				}
				else
				{
					g_sM1Drive.sPosCtrl.fltAdvSpeed = -speed / g_fltM1speedAngularScale * sync_set.adv_speed_scale;
				}
				set_speed = sync_set.abs_spd * 3;
				set_pos = pos;
			}
		}
	}
motor_start:
	/* if(set_speed >2000)
		set_speed = 2000 ;*/

	movePos(posOut2In(set_pos), spdOut2In(MOT_MSG_MOVE_POS, set_speed), -1);
#endif
end:
	/*  buff[0][debug_ptr] =   sync_debug_para_values(sync_set.sync_debug_idx[0]) ;
	  buff[1][debug_ptr] =   sync_debug_para_values(sync_set.sync_debug_idx[1]);
	  if(debug_ptr < debug_max_num -1 )
			debug_ptr ++ ;
		else
			debug_ptr =0 ;*/
	old_pos = push_pos;
	old_gap_position = gap_position;
}
#endif
