

#include <stdint.h>
#include "machine.h"
#ifndef _WIN32
#include "motor.h"
#include "mcdrv.h"
#include "param.h"
#include "alert.h"
#include "m1_sm_ref_sol.h"
#include "motor_sync.h"
#include "board.h"
#include "av_data.h"
#endif // !WIN32
#include "math.h"
SYNC_SET sync_un_speed_set;
#ifdef _WIN32
#define ENCODER_SYNC 1

SYNC_SET sync_set;
float posIn2Out(float fPos)
{
	return fPos;
}

float posOut2In(float fPos)
{
	return fPos;
}
float sync_get_ext_pos(void)
{
	return 0.0f;
}
float getCurPos(void)
{
	return 0.0f;
}
float getCurSpd(void)
{
	return 0.0f;
}
#endif
MACHINE_SET_ST machine_set_st;
MACHINE_SET_ST machine_main_set_st;
SCURVEST SCurve_ltr_St;
SCURVEST SCurve_rtl_St;
SCURVEST SCurveSt_set_st;
SCURVEST *pScurveSt = NULL;
int m_nPush = 0;

#define VMM_TO_VRPM(X) (X * 60000.0f / MOTOR_CYCLE_POS)
#define VRPM_TO_VMM(X) (X * MOTOR_CYCLE_POS / 60000.0f)
#define MM_TO_ENCODER(X) (X * MAX_JR_MOTOR_MAX_ENCODER / MOTOR_CYCLE_POS)
#define ENCODER_TO_MM(X) (X * MOTOR_CYCLE_POS / MAX_JR_MOTOR_MAX_ENCODER)

uint16_t adv_change_pos[30] = {40, 40, 40, 60, 60, 80, 80, 80, 80, 80, 80, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165}; // 100-3000RPM 2A

#define MAX_SPEED 1000					  // rpm
#define CIRCLE_POS 137.0f				  // ????????mm
#define CIRCLE_ENCODER 16383			  // ??????????
#define MAX_ACC (CIRCLE_POS * 0.025f / 5) // mm/ms^2
#define MAX_DEC (CIRCLE_POS * 0.025f / 5) // mm/ms^2

#define SYNC_MIN_SPEED 0

uint16_t mm_to_encoder(float f)
{
	return (uint16_t)(f * CIRCLE_ENCODER / CIRCLE_POS);
}
float encoder_to_mm(uint16_t encoder)
{
	return encoder * CIRCLE_POS / CIRCLE_ENCODER;
}
float vMm_to_vRpm(float f)
{
	return f * 60000 / CIRCLE_POS;
}
float vRpm_to_vMm(float rpm)
{
	return rpm * CIRCLE_POS / 60000.0f;
}

/**
j_cur_speed= ??????
nL = ????????(mm)
acc_max = 137*0.025/7
dec_max = 137*0.025/14
jScale = ???
**/
float SimCalGearRatio(uint16_t j_cur_speed, uint16_t nL, float acc_max, float dec_max, float jScale) // ???????????????????????????????????????????
{
	float t, v0;
	float ax = 1 / acc_max + 1 / dec_max;

	t = jScale / 2 * 60000 / j_cur_speed;				   // ????
	v0 = t / ax - sqrt((t * t) / (ax * ax) - 2 * nL / ax); // t/x - sqrt(t^2/ax^2-2*l/ax) (mm/ms)
	return v0;
	// return   vMm_to_vRpm(v0) / j_max_speed;
}

/*
* j_cur_speed = ?????????PM
* jScale = ???
* v0 = ???????mm/ms

*/
float SimCalTime(uint16_t j_cur_speed, float jScale) // ????????????? ?????????????
{
	/*if(dir ==0)//???
	{
			m_Ht1 = (1-f_acc)*v0/m_Ha1 ;
			m_Ht2 = (1+f_acc)*v0/m_Ha2 ;
	}
	else //??
	{
			m_Ht1 = (1+f_acc)*v0/m_Ha1 ;
			m_Ht2 = (1-f_acc)*v0/m_Ha2 ;
	}*/
	return jScale / 2 * 60000 / j_cur_speed;
}

/**
 *j_cur_speed= ??????
 *nL = ????????(mm)
 *acc_max = 137*0.025/7
 *dec_max = 137*0.025/14
 *jScale = ???
 *j_acc = ??????-1.0f-1.0f ????????????
 **/
float SimCalunSpeed(uint16_t j_cur_speed, float nL, float acc_max, float dec_max, float jScale, float j_acc) // ????????
{
	float t, v0;
	//	float ax = 1 / acc_max + 1 / dec_max;
	float ax = 2 * (1 + j_acc) / acc_max + 2 * (1 - j_acc) / dec_max - (1 + j_acc) * (1 + j_acc) / acc_max - (1 - j_acc) * (1 - j_acc) / dec_max;
	t = jScale / 2 * 60000 / j_cur_speed;				   // ????
	v0 = t / ax - sqrt((t * t) / (ax * ax) - 2 * nL / ax); // t/x - sqrt(t^2/ax^2-2*l/ax) (mm/ms)
	return v0;
	// return   vMm_to_vRpm(v0) / j_max_speed;
}
/*
 * V3 = ???????????????mm/ms
 * T = ???????ms
 * T1 = ????????????ms
 * T5 = ????????????ms
 * S1 = ???????????mm
 * s5 = ???????????mm
 *l_start = ???????????????mm
 *l_right = ????????????????mm
 *f_acc = ???????-1to1
 */
void ScurveCalT(float v3, float T, float T1, float T5, float S1, float S5, float l_start, float l_right, float f_acc) // ??????
{
	uint8_t tx = 1; // ????????????????????? = T1+tx  tx??????????????????????????????
	uint8_t flag = 0;
	float fv3_max = 0.5 * v3; // ??????????????????
	float sync_dis = 0.0f;	  // ????
	if (l_start == 0)
	{
		SCurveSt_set_st.s_curve_T1 = T1;
		SCurveSt_set_st.s_curve_T2 = 0;
	}
	else
	{
		for (tx = 1; tx < 100; tx++)
		{
			SCurveSt_set_st.s_curve_v2 = v3 * (1 - f_acc) + SCurveSt_set_st.s_curve_a3 * tx;
			sync_dis = S1 + v3 * (1 - f_acc) * tx + 0.5f * SCurveSt_set_st.s_curve_a3 * tx * tx;
			SCurveSt_set_st.s_curve_T1 = (SCurveSt_set_st.s_curve_v2 * (T1 + tx) - 2 * (sync_dis - l_start)) / (SCurveSt_set_st.s_curve_v2 - SCurveSt_set_st.s_curve_a_max * (T1 + tx));
			SCurveSt_set_st.s_curve_v1 = SCurveSt_set_st.s_curve_a_max * SCurveSt_set_st.s_curve_T1;
			if (SCurveSt_set_st.s_curve_v1 > fv3_max)
			{
				flag = 1;
				SCurveSt_set_st.s_curve_T2 = T1 + tx - SCurveSt_set_st.s_curve_T1;
				break;
			}
		}
	}
	if (l_right == 0)
	{
		SCurveSt_set_st.s_curve_T4 = 0;
		SCurveSt_set_st.s_curve_T5 = T1;
	}
	else
	{
		/*	if (flag)
				{
						m_s_curve_V4 = v3 * (1 - f_acc) + m_s_curve_a3 * (T - T1 - T5 - tx);
						sync_dis = S5 + m_s_curve_V4 * tx + 0.5f * m_s_curve_a3 * tx * tx;
						m_s_curve_T5 = (m_s_curve_V4 * (T5 + tx) - 2 * (sync_dis - l_right)) / (m_s_curve_V4 - m_s_curve_a_max * (T5 + tx));
						m_s_curve_V5 = m_s_curve_a_max * m_s_curve_T5;
						m_s_curve_T4 = T5 + tx - m_s_curve_T5;
				}
				else*/
		{
			for (tx = 1; tx < 100; tx++)
			{
				SCurveSt_set_st.s_curve_v4 = v3 * (1 - f_acc) + SCurveSt_set_st.s_curve_a3 * (T - T1 - T5 - tx);
				sync_dis = S5 + SCurveSt_set_st.s_curve_v4 * tx + 0.5f * SCurveSt_set_st.s_curve_a3 * tx * tx;
				SCurveSt_set_st.s_curve_T5 = (SCurveSt_set_st.s_curve_v4 * (T5 + tx) - 2 * (sync_dis - l_right)) / (SCurveSt_set_st.s_curve_v4 - SCurveSt_set_st.s_curve_a_max * (T5 + tx));
				SCurveSt_set_st.s_curve_v5 = SCurveSt_set_st.s_curve_a_max * SCurveSt_set_st.s_curve_T5;
				if (SCurveSt_set_st.s_curve_v5 > fv3_max && SCurveSt_set_st.s_curve_v5 <= SCurveSt_set_st.s_curve_v4)
				{
					SCurveSt_set_st.s_curve_T4 = T5 + tx - SCurveSt_set_st.s_curve_T5;
					break;
				}
			}
		}
	}
	SCurveSt_set_st.s_curve_T3 = T - SCurveSt_set_st.s_curve_T1 - SCurveSt_set_st.s_curve_T2 - SCurveSt_set_st.s_curve_T4 - SCurveSt_set_st.s_curve_T5;
}
/*
 * s = s????? mm
 * s4 = s???s4?????????mm
 * t = s???????ms
 * return = ?????_max mm/ms
 */
float ScurveCalVmax(float s, float s4, float t) // ??????
{

	return (2 * s - s4) / t;
}
/*
 * s = s????? mm
 * a_max = s??????????mm/ms^2
 * v_max = s????????mm/ms
 * t = s???????ms
 * return = ?????????mm/ms^3
 */
float ScurveCalJJ(float s, float a_max, float v_max, float t) // ????????
{
	return (a_max * a_max * v_max) / (v_max * a_max * t - v_max * v_max - s * a_max);
}
/*
 * j = s??????????
 * a_max = s??????????mm/ms^2
 * v_max = s????????mm/ms
 * t = s???????ms
 * return = s???T4 ms
 */
float ScurveCalT4(float j, float a_max, float v_max, float t) // ?S?T4
{
	return t - 2 * v_max / a_max - 2 * a_max / j;
}

void InitScurve(void)
{
	SCurveSt_set_st.s_curve_a_max = MAX_ACC;
	SCurveSt_set_st.s_curve_s4 = 170;
}
void machine_set_cw_flag(uint8_t flag) //????
{
	machine_main_set_st.mc_cw_flag = flag;
}
void machine_cal_scurve_para(uint8_t cw_flag) // S
{
	float f_hl = machine_main_set_st.mc_right_limit_mm - machine_main_set_st.mc_left_limit_mm;

	if (cw_flag)
	{

		ScurveCalT(SCurveSt_set_st.s_curve_v3, SCurveSt_set_st.s_curve_T, SCurveSt_set_st.s_curve_v3 * (1 - machine_set_st.mc_acc_scale) / SCurveSt_set_st.s_curve_a_max,
				   SCurveSt_set_st.s_curve_v3 * (1 + machine_set_st.mc_acc_scale) / SCurveSt_set_st.s_curve_a_max, SCurveSt_set_st.s_curve_s1, SCurveSt_set_st.s_curve_s5,
				   machine_main_set_st.mc_left_limit_mm - machine_main_set_st.mc_cw_pre_left_limit_mm, machine_main_set_st.mc_cw_pre_right_limit_mm - machine_main_set_st.mc_right_limit_mm, machine_set_st.mc_acc_scale);
		if (SCurveSt_set_st.s_curve_T2 == 0)
			SCurveSt_set_st.s_curve_a2 = SCurveSt_set_st.s_curve_a_max;
		else
			SCurveSt_set_st.s_curve_a2 = (SCurveSt_set_st.s_curve_v2 - SCurveSt_set_st.s_curve_v1) / SCurveSt_set_st.s_curve_T2;
		if (SCurveSt_set_st.s_curve_T4 == 0)
			SCurveSt_set_st.s_curve_a4 = SCurveSt_set_st.s_curve_a_max;
		else
			SCurveSt_set_st.s_curve_a4 = (SCurveSt_set_st.s_curve_v4 - SCurveSt_set_st.s_curve_v5) / SCurveSt_set_st.s_curve_T4;
		// SimCalAccSA(m_Jv, m_Llimit, m_Rlimit, m_gear_ratio, m_t);
	}
	else
	{

#ifdef S_CURVE

		SCurveSt_set_st.s_curve_T = SimCalTime(machine_set_st.mc_set_speed, machine_main_set_st.mc_j_scale);
		SCurveSt_set_st.s_curve_v_max = ScurveCalVmax(f_hl, SCurveSt_set_st.s_curve_s4, SCurveSt_set_st.s_curve_T); // pre_cw_h??????????? m_s_curve_s4?????150 = (200 - 2*?????5)
		SCurveSt_set_st.s_curve_jj = ScurveCalJJ(f_hl, SCurveSt_set_st.s_curve_a_max, SCurveSt_set_st.s_curve_v_max, SCurveSt_set_st.s_curve_T);
		SCurveSt_set_st.s_curve_T4 = SCurveSt_set_st.s_curve_s4 / SCurveSt_set_st.s_curve_v_max;
		SCurveSt_set_st.s_curve_s = f_hl;
		SCurveSt_set_st.s_curve_T1 = SCurveSt_set_st.s_curve_a_max / SCurveSt_set_st.s_curve_jj;
		SCurveSt_set_st.s_curve_T2 = SCurveSt_set_st.s_curve_v_max / SCurveSt_set_st.s_curve_a_max - SCurveSt_set_st.s_curve_T1;
		SCurveSt_set_st.s_curve_T3 = SCurveSt_set_st.s_curve_T1;
		SCurveSt_set_st.s_curve_T5 = SCurveSt_set_st.s_curve_T3;
		SCurveSt_set_st.s_curve_T6 = SCurveSt_set_st.s_curve_T2;
		SCurveSt_set_st.s_curve_T7 = SCurveSt_set_st.s_curve_T1;
#else

		machine_main_set_st.mc_cw_pre_left_limit_mm = machine_main_set_st.mc_left_limit_mm;
		machine_main_set_st.mc_cw_pre_right_limit_mm = machine_main_set_st.mc_right_limit_mm;
		SCurveSt_set_st.s_curve_s = f_hl;
		SCurveSt_set_st.s_curve_v3 = SimCalunSpeed(machine_set_st.mc_set_speed, SCurveSt_set_st.s_curve_s, SCurveSt_set_st.s_curve_a_max, SCurveSt_set_st.s_curve_a_max, machine_main_set_st.mc_j_scale, machine_set_st.mc_acc_scale);
		SCurveSt_set_st.s_curve_v_max = SCurveSt_set_st.s_curve_v3;
		SCurveSt_set_st.s_curve_T = SimCalTime(machine_set_st.mc_set_speed, machine_main_set_st.mc_j_scale);
		SCurveSt_set_st.s_curve_T1 = SCurveSt_set_st.s_curve_v3 * (1 - machine_set_st.mc_acc_scale) / SCurveSt_set_st.s_curve_a_max;
		SCurveSt_set_st.s_curve_T2 = 0;
		SCurveSt_set_st.s_curve_T4 = 0;
		SCurveSt_set_st.s_curve_T5 = SCurveSt_set_st.s_curve_v3 * (1 + machine_set_st.mc_acc_scale) / SCurveSt_set_st.s_curve_a_max;
		SCurveSt_set_st.s_curve_T3 = SCurveSt_set_st.s_curve_T - SCurveSt_set_st.s_curve_T1 - SCurveSt_set_st.s_curve_T2 - SCurveSt_set_st.s_curve_T5 - SCurveSt_set_st.s_curve_T4;
		SCurveSt_set_st.s_curve_s1 = 0.5f * SCurveSt_set_st.s_curve_a_max * SCurveSt_set_st.s_curve_T1 * SCurveSt_set_st.s_curve_T1;
		SCurveSt_set_st.s_curve_s5 = 0.5f * SCurveSt_set_st.s_curve_a_max * SCurveSt_set_st.s_curve_T5 * SCurveSt_set_st.s_curve_T5;
		SCurveSt_set_st.s_curve_a3 = 2 * machine_set_st.mc_acc_scale * SCurveSt_set_st.s_curve_v3 / SCurveSt_set_st.s_curve_T3;

#endif
	}
}
void machine_main_paras_use(uint8_t type) // 0 1 2
{
	if (type == 0) //??????
	{
		machine_set_st.mc_left_limit = machine_main_set_st.mc_left_limit;
		machine_set_st.mc_f_left_limit = machine_main_set_st.mc_f_left_limit;
		machine_set_st.mc_left_r2l_acc_speed_fScale = machine_main_set_st.mc_left_acc_fScale;
		machine_set_st.mc_right_r2l_acc_speed_fScale = machine_main_set_st.mc_right_acc_fScale;
		machine_set_st.mc_left_r2l_acc_speed_pos = machine_main_set_st.mc_left_acc_speed_pos;
		machine_set_st.mc_right_r2l_acc_speed_pos = machine_main_set_st.mc_right_acc_speed_pos;
		machine_set_st.mc_left_r2l_fScale = machine_main_set_st.mc_left_scale;
		machine_set_st.mc_un_speed_pos = machine_main_set_st.mc_right_limit - machine_main_set_st.mc_left_limit - machine_main_set_st.mc_max_acc_pos - machine_main_set_st.mc_var_acc_pos - machine_main_set_st.mc_max_dec_pos - machine_main_set_st.mc_var_dec_pos;
		machine_set_st.mc_acc = machine_main_set_st.mc_acc;
		machine_set_st.mc_dec = machine_main_set_st.mc_dec;
		machine_set_st.mc_un_speed_fScale = machine_main_set_st.mc_un_speed_fScale;
		machine_set_st.mc_left_limit_mm = machine_main_set_st.mc_left_limit_mm;
		machine_cal_scurve_para(machine_main_set_st.mc_cw_flag);
#ifdef S_CURVE
		SCurve_rtl_St.s_curve_jj = SCurveSt_set_st.s_curve_jj;
		SCurve_rtl_St.s_curve_T1 = SCurveSt_set_st.s_curve_T1;
		SCurve_rtl_St.s_curve_T2 = SCurveSt_set_st.s_curve_T2;
		SCurve_rtl_St.s_curve_T3 = SCurveSt_set_st.s_curve_T3;
		SCurve_rtl_St.s_curve_T4 = SCurveSt_set_st.s_curve_T4;
		SCurve_rtl_St.s_curve_T5 = SCurveSt_set_st.s_curve_T5;
		SCurve_rtl_St.s_curve_T6 = SCurveSt_set_st.s_curve_T6;
		SCurve_rtl_St.s_curve_T7 = SCurveSt_set_st.s_curve_T7;
		SCurve_rtl_St.s_curve_T = SCurveSt_set_st.s_curve_T;
		SCurve_rtl_St.s_curve_a_max = SCurveSt_set_st.s_curve_a_max;
		SCurve_rtl_St.s_curve_v_max = SCurveSt_set_st.s_curve_v_max;

		SCurve_rtl_St.s_curve_v1 = SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T1 / 2;
		SCurve_rtl_St.s_curve_v2 = SCurve_rtl_St.s_curve_v1 + SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T2;
		SCurve_rtl_St.s_curve_v3 = SCurve_rtl_St.s_curve_v2 + SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T1 / 2;
		SCurve_rtl_St.s_curve_v4 = SCurve_rtl_St.s_curve_v3;
		SCurve_rtl_St.s_curve_v5 = SCurve_rtl_St.s_curve_v4 - SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T5 * SCurve_rtl_St.s_curve_T5 / 2;
		SCurve_rtl_St.s_curve_v6 = SCurve_rtl_St.s_curve_v5 - SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T5 * SCurve_rtl_St.s_curve_T6;
		SCurve_rtl_St.s_curve_v7 = SCurve_rtl_St.s_curve_v6 - SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T7 * SCurve_rtl_St.s_curve_T7 / 2;

		SCurve_rtl_St.s_curve_s1 = 0.5f * SCurve_rtl_St.s_curve_a_max * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T1;
		SCurve_rtl_St.s_curve_s2 = SCurve_rtl_St.s_curve_s1 + SCurve_rtl_St.s_curve_v1 * SCurve_rtl_St.s_curve_T2 + SCurve_rtl_St.s_curve_a2 * SCurve_rtl_St.s_curve_T2 * SCurve_rtl_St.s_curve_T2 / 2;
		SCurve_rtl_St.s_curve_s3 = SCurve_rtl_St.s_curve_s2 + SCurve_rtl_St.s_curve_v2 * SCurve_rtl_St.s_curve_T3 + 0.5f * SCurve_rtl_St.s_curve_a3 * SCurve_rtl_St.s_curve_T3 * SCurve_rtl_St.s_curve_T3;
		SCurve_rtl_St.s_curve_s4 = SCurve_rtl_St.s_curve_s3 + SCurve_rtl_St.s_curve_v3 * SCurve_rtl_St.s_curve_T4 - SCurve_rtl_St.s_curve_a4 * SCurve_rtl_St.s_curve_T4 * SCurve_rtl_St.s_curve_T4 / 2;
		SCurve_rtl_St.s_curve_s5 = SCurve_rtl_St.s_curve_s4 + SCurve_rtl_St.s_curve_v4 * SCurve_rtl_St.s_curve_T5 - SCurve_rtl_St.s_curve_a_max * SCurve_rtl_St.s_curve_T5 * SCurve_rtl_St.s_curve_T5 / 2;

		SCurve_rtl_St.s_curve_s1 = SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T1 / 6;
		SCurve_rtl_St.s_curve_s2 = SCurve_rtl_St.s_curve_s1 + SCurve_rtl_St.s_curve_v1 * SCurve_rtl_St.s_curve_T2 + SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T2 * SCurve_rtl_St.s_curve_T2 / 2;
		SCurve_rtl_St.s_curve_s3 = SCurve_rtl_St.s_curve_s2 + SCurve_rtl_St.s_curve_v2 * SCurve_rtl_St.s_curve_T3 + SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T3 * SCurve_rtl_St.s_curve_T3 / 2 - SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T3 * SCurve_rtl_St.s_curve_T3 * SCurve_rtl_St.s_curve_T3 / 6;
		SCurve_rtl_St.s_curve_s4 = SCurve_rtl_St.s_curve_s3 + SCurve_rtl_St.s_curve_v3 * SCurve_rtl_St.s_curve_T4;
		SCurve_rtl_St.s_curve_s5 = SCurve_rtl_St.s_curve_s4 + SCurve_rtl_St.s_curve_v4 * SCurve_rtl_St.s_curve_T5 - SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T5 * SCurve_rtl_St.s_curve_T5 * SCurve_rtl_St.s_curve_T5 / 6;
		SCurve_rtl_St.s_curve_s6 = SCurve_rtl_St.s_curve_s5 + SCurve_rtl_St.s_curve_v5 * SCurve_rtl_St.s_curve_T6 - SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T6 * SCurve_rtl_St.s_curve_T6 / 2;
		SCurve_rtl_St.s_curve_s7 = SCurve_rtl_St.s_curve_s6 + SCurve_rtl_St.s_curve_v6 * SCurve_rtl_St.s_curve_T7 - SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T7 * SCurve_rtl_St.s_curve_T7 / 2 + SCurve_rtl_St.s_curve_jj * SCurve_rtl_St.s_curve_T7 * SCurve_rtl_St.s_curve_T7 * SCurve_rtl_St.s_curve_T7 / 6;
#else
		SCurve_rtl_St.s_curve_T1 = SCurveSt_set_st.s_curve_T1;
		SCurve_rtl_St.s_curve_T2 = SCurveSt_set_st.s_curve_T2;
		SCurve_rtl_St.s_curve_T3 = SCurveSt_set_st.s_curve_T3;
		SCurve_rtl_St.s_curve_T4 = SCurveSt_set_st.s_curve_T4;
		SCurve_rtl_St.s_curve_T5 = SCurveSt_set_st.s_curve_T5;
		SCurve_rtl_St.s_curve_T6 = SCurveSt_set_st.s_curve_T6;
		SCurve_rtl_St.s_curve_T7 = SCurveSt_set_st.s_curve_T7;
		SCurve_rtl_St.s_curve_T = SCurveSt_set_st.s_curve_T;
		SCurve_rtl_St.s_curve_a_max = SCurveSt_set_st.s_curve_a_max;
		SCurve_rtl_St.s_curve_a2 = SCurveSt_set_st.s_curve_a2;
		SCurve_rtl_St.s_curve_a3 = SCurveSt_set_st.s_curve_a3;
		SCurve_rtl_St.s_curve_a4 = SCurveSt_set_st.s_curve_a4;
		SCurve_rtl_St.s_curve_v1 = SCurveSt_set_st.s_curve_a_max * SCurveSt_set_st.s_curve_T1;

		SCurve_rtl_St.s_curve_v2 = SCurve_rtl_St.s_curve_v1 + SCurve_rtl_St.s_curve_a2 * SCurve_rtl_St.s_curve_T2;
		SCurve_rtl_St.s_curve_v3 = SCurve_rtl_St.s_curve_v2 + SCurve_rtl_St.s_curve_a3 * SCurve_rtl_St.s_curve_T3;
		SCurve_rtl_St.s_curve_v4 = SCurve_rtl_St.s_curve_v3 - SCurve_rtl_St.s_curve_a4 * SCurve_rtl_St.s_curve_T4;
		SCurve_rtl_St.s_curve_v5 = 0;

		SCurve_rtl_St.s_curve_s1 = 0.5f * SCurve_rtl_St.s_curve_a_max * SCurve_rtl_St.s_curve_T1 * SCurve_rtl_St.s_curve_T1;
		SCurve_rtl_St.s_curve_s2 = SCurve_rtl_St.s_curve_s1 + SCurve_rtl_St.s_curve_v1 * SCurve_rtl_St.s_curve_T2 + SCurve_rtl_St.s_curve_a2 * SCurve_rtl_St.s_curve_T2 * SCurve_rtl_St.s_curve_T2 / 2;
		SCurve_rtl_St.s_curve_s3 = SCurve_rtl_St.s_curve_s2 + SCurve_rtl_St.s_curve_v2 * SCurve_rtl_St.s_curve_T3 + 0.5f * SCurve_rtl_St.s_curve_a3 * SCurve_rtl_St.s_curve_T3 * SCurve_rtl_St.s_curve_T3;
		SCurve_rtl_St.s_curve_s4 = SCurve_rtl_St.s_curve_s3 + SCurve_rtl_St.s_curve_v3 * SCurve_rtl_St.s_curve_T4 - SCurve_rtl_St.s_curve_a4 * SCurve_rtl_St.s_curve_T4 * SCurve_rtl_St.s_curve_T4 / 2;
		SCurve_rtl_St.s_curve_s5 = SCurve_rtl_St.s_curve_s4 + SCurve_rtl_St.s_curve_v4 * SCurve_rtl_St.s_curve_T5 - SCurve_rtl_St.s_curve_a_max * SCurve_rtl_St.s_curve_T5 * SCurve_rtl_St.s_curve_T5 / 2;
#endif // S_CURVE
	}
	else if (type == 1) //??????
	{
		machine_set_st.mc_right_limit = machine_main_set_st.mc_right_limit;
		machine_set_st.mc_f_right_limit = machine_main_set_st.mc_f_right_limit;
		machine_set_st.mc_left_l2r_acc_speed_fScale = machine_main_set_st.mc_left_acc_fScale;
		machine_set_st.mc_right_l2r_acc_speed_fScale = machine_main_set_st.mc_right_acc_fScale;
		machine_set_st.mc_left_l2r_acc_speed_pos = machine_main_set_st.mc_left_acc_speed_pos;
		machine_set_st.mc_right_l2r_acc_speed_pos = machine_main_set_st.mc_right_acc_speed_pos;
		machine_set_st.mc_left_l2r_fScale = machine_main_set_st.mc_left_scale;
		machine_set_st.mc_un_speed_pos = machine_main_set_st.mc_right_limit - machine_main_set_st.mc_left_limit - machine_main_set_st.mc_max_acc_pos - machine_main_set_st.mc_var_acc_pos - machine_main_set_st.mc_max_dec_pos - machine_main_set_st.mc_var_dec_pos;
		machine_set_st.mc_acc = machine_main_set_st.mc_acc;
		machine_set_st.mc_dec = machine_main_set_st.mc_dec;
		machine_set_st.mc_un_speed_fScale = machine_main_set_st.mc_un_speed_fScale;
		machine_set_st.mc_right_limit_mm = machine_main_set_st.mc_right_limit_mm;
		machine_cal_scurve_para(machine_main_set_st.mc_cw_flag);
#ifdef S_CURVE
		SCurve_ltr_St.s_curve_jj = SCurveSt_set_st.s_curve_jj;
		SCurve_ltr_St.s_curve_T1 = SCurveSt_set_st.s_curve_T1;
		SCurve_ltr_St.s_curve_T2 = SCurveSt_set_st.s_curve_T2;
		SCurve_ltr_St.s_curve_T3 = SCurveSt_set_st.s_curve_T3;
		SCurve_ltr_St.s_curve_T4 = SCurveSt_set_st.s_curve_T4;
		SCurve_ltr_St.s_curve_T5 = SCurveSt_set_st.s_curve_T5;
		SCurve_ltr_St.s_curve_T6 = SCurveSt_set_st.s_curve_T6;
		SCurve_ltr_St.s_curve_T7 = SCurveSt_set_st.s_curve_T7;
		SCurve_ltr_St.s_curve_T = SCurveSt_set_st.s_curve_T;
		SCurve_ltr_St.s_curve_a_max = SCurveSt_set_st.s_curve_a_max;
		SCurve_ltr_St.s_curve_v_max = SCurveSt_set_st.s_curve_v_max;

		SCurve_ltr_St.s_curve_v1 = SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T1 / 2;
		SCurve_ltr_St.s_curve_v2 = SCurve_ltr_St.s_curve_v1 + SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T2;
		SCurve_ltr_St.s_curve_v3 = SCurve_ltr_St.s_curve_v2 + SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T1 / 2;
		SCurve_ltr_St.s_curve_v4 = SCurve_ltr_St.s_curve_v3;
		SCurve_ltr_St.s_curve_v5 = SCurve_ltr_St.s_curve_v4 - SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T5 * SCurve_ltr_St.s_curve_T5 / 2;
		SCurve_ltr_St.s_curve_v6 = SCurve_ltr_St.s_curve_v5 - SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T5 * SCurve_ltr_St.s_curve_T6;
		SCurve_ltr_St.s_curve_v7 = SCurve_ltr_St.s_curve_v6 - SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T7 * SCurve_ltr_St.s_curve_T7 / 2;

		SCurve_ltr_St.s_curve_s1 = 0.5f * SCurve_ltr_St.s_curve_a_max * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T1;
		SCurve_ltr_St.s_curve_s2 = SCurve_ltr_St.s_curve_s1 + SCurve_ltr_St.s_curve_v1 * SCurve_ltr_St.s_curve_T2 + SCurve_ltr_St.s_curve_a2 * SCurve_ltr_St.s_curve_T2 * SCurve_ltr_St.s_curve_T2 / 2;
		SCurve_ltr_St.s_curve_s3 = SCurve_ltr_St.s_curve_s2 + SCurve_ltr_St.s_curve_v2 * SCurve_ltr_St.s_curve_T3 + 0.5f * SCurve_ltr_St.s_curve_a3 * SCurve_ltr_St.s_curve_T3 * SCurve_ltr_St.s_curve_T3;
		SCurve_ltr_St.s_curve_s4 = SCurve_ltr_St.s_curve_s3 + SCurve_ltr_St.s_curve_v3 * SCurve_ltr_St.s_curve_T4 - SCurve_ltr_St.s_curve_a4 * SCurve_ltr_St.s_curve_T4 * SCurve_ltr_St.s_curve_T4 / 2;
		SCurve_ltr_St.s_curve_s5 = SCurve_ltr_St.s_curve_s4 + SCurve_ltr_St.s_curve_v4 * SCurve_ltr_St.s_curve_T5 - SCurve_ltr_St.s_curve_a_max * SCurve_ltr_St.s_curve_T5 * SCurve_ltr_St.s_curve_T5 / 2;

		SCurve_ltr_St.s_curve_s1 = SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T1 / 6;
		SCurve_ltr_St.s_curve_s2 = SCurve_ltr_St.s_curve_s1 + SCurve_ltr_St.s_curve_v1 * SCurve_ltr_St.s_curve_T2 + SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T2 * SCurve_ltr_St.s_curve_T2 / 2;
		SCurve_ltr_St.s_curve_s3 = SCurve_ltr_St.s_curve_s2 + SCurve_ltr_St.s_curve_v2 * SCurve_ltr_St.s_curve_T3 + SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T3 * SCurve_ltr_St.s_curve_T3 / 2 - SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T3 * SCurve_ltr_St.s_curve_T3 * SCurve_ltr_St.s_curve_T3 / 6;
		SCurve_ltr_St.s_curve_s4 = SCurve_ltr_St.s_curve_s3 + SCurve_ltr_St.s_curve_v3 * SCurve_ltr_St.s_curve_T4;
		SCurve_ltr_St.s_curve_s5 = SCurve_ltr_St.s_curve_s4 + SCurve_ltr_St.s_curve_v4 * SCurve_ltr_St.s_curve_T5 - SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T5 * SCurve_ltr_St.s_curve_T5 * SCurve_ltr_St.s_curve_T5 / 6;
		SCurve_ltr_St.s_curve_s6 = SCurve_ltr_St.s_curve_s5 + SCurve_ltr_St.s_curve_v5 * SCurve_ltr_St.s_curve_T6 - SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T6 * SCurve_ltr_St.s_curve_T6 / 2;
		SCurve_ltr_St.s_curve_s7 = SCurve_ltr_St.s_curve_s6 + SCurve_ltr_St.s_curve_v6 * SCurve_ltr_St.s_curve_T7 - SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T7 * SCurve_ltr_St.s_curve_T7 / 2 + SCurve_ltr_St.s_curve_jj * SCurve_ltr_St.s_curve_T7 * SCurve_ltr_St.s_curve_T7 * SCurve_ltr_St.s_curve_T7 / 6;
#else
		SCurve_ltr_St.s_curve_T1 = SCurveSt_set_st.s_curve_T1;
		SCurve_ltr_St.s_curve_T2 = SCurveSt_set_st.s_curve_T2;
		SCurve_ltr_St.s_curve_T3 = SCurveSt_set_st.s_curve_T3;
		SCurve_ltr_St.s_curve_T4 = SCurveSt_set_st.s_curve_T4;
		SCurve_ltr_St.s_curve_T5 = SCurveSt_set_st.s_curve_T5;
		SCurve_ltr_St.s_curve_T6 = SCurveSt_set_st.s_curve_T6;
		SCurve_ltr_St.s_curve_T7 = SCurveSt_set_st.s_curve_T7;
		SCurve_ltr_St.s_curve_T = SCurveSt_set_st.s_curve_T;
		SCurve_ltr_St.s_curve_a_max = SCurveSt_set_st.s_curve_a_max;
		SCurve_ltr_St.s_curve_a2 = SCurveSt_set_st.s_curve_a2;
		SCurve_ltr_St.s_curve_a3 = SCurveSt_set_st.s_curve_a3;
		SCurve_ltr_St.s_curve_a4 = SCurveSt_set_st.s_curve_a4;
		SCurve_ltr_St.s_curve_v1 = SCurveSt_set_st.s_curve_a_max * SCurveSt_set_st.s_curve_T1;

		SCurve_ltr_St.s_curve_v2 = SCurve_ltr_St.s_curve_v1 + SCurve_ltr_St.s_curve_a2 * SCurve_ltr_St.s_curve_T2;
		SCurve_ltr_St.s_curve_v3 = SCurve_ltr_St.s_curve_v2 + SCurve_ltr_St.s_curve_a3 * SCurve_ltr_St.s_curve_T3;
		SCurve_ltr_St.s_curve_v4 = SCurve_ltr_St.s_curve_v3 - SCurve_ltr_St.s_curve_a4 * SCurve_ltr_St.s_curve_T4;
		SCurve_ltr_St.s_curve_v5 = 0;

		SCurve_ltr_St.s_curve_s1 = 0.5f * SCurve_ltr_St.s_curve_a_max * SCurve_ltr_St.s_curve_T1 * SCurve_ltr_St.s_curve_T1;
		SCurve_ltr_St.s_curve_s2 = SCurve_ltr_St.s_curve_s1 + SCurve_ltr_St.s_curve_v1 * SCurve_ltr_St.s_curve_T2 + SCurve_ltr_St.s_curve_a2 * SCurve_ltr_St.s_curve_T2 * SCurve_ltr_St.s_curve_T2 / 2;
		SCurve_ltr_St.s_curve_s3 = SCurve_ltr_St.s_curve_s2 + SCurve_ltr_St.s_curve_v2 * SCurve_ltr_St.s_curve_T3 + 0.5f * SCurve_ltr_St.s_curve_a3 * SCurve_ltr_St.s_curve_T3 * SCurve_ltr_St.s_curve_T3;
		SCurve_ltr_St.s_curve_s4 = SCurve_ltr_St.s_curve_s3 + SCurve_ltr_St.s_curve_v3 * SCurve_ltr_St.s_curve_T4 - SCurve_ltr_St.s_curve_a4 * SCurve_ltr_St.s_curve_T4 * SCurve_ltr_St.s_curve_T4 / 2;
		SCurve_ltr_St.s_curve_s5 = SCurve_ltr_St.s_curve_s4 + SCurve_ltr_St.s_curve_v4 * SCurve_ltr_St.s_curve_T5 - SCurve_ltr_St.s_curve_a_max * SCurve_ltr_St.s_curve_T5 * SCurve_ltr_St.s_curve_T5 / 2;
#endif
	}
	else if (type == 2)
	{
		machine_set_st.mc_left_scale = machine_main_set_st.mc_left_scale;
		machine_set_st.mc_left_set_value1 = machine_main_set_st.mc_left_set_value1;
		machine_set_st.mc_left_set_value2 = machine_main_set_st.mc_left_set_value2;
		machine_set_st.mc_right_scale = machine_main_set_st.mc_right_scale;
		machine_set_st.mc_right_set_value1 = machine_main_set_st.mc_right_set_value1;
		machine_set_st.mc_right_set_value2 = machine_main_set_st.mc_right_set_value2;
		machine_set_st.mc_un_speed_fScale = machine_main_set_st.mc_un_speed_fScale;
		//	machine_set_st.mc_acc_speed_fScale = machine_main_set_st.mc_acc_speed_fScale ;
		//	machine_set_st.mc_acc_speed_pos = machine_main_set_st.mc_acc_speed_pos ;
		machine_set_st.mc_jr_acc_encoder = machine_main_set_st.mc_acc_speed_pos / machine_main_set_st.mc_acc_speed_fScale * machine_main_set_st.mc_left_scale;
	}
	machine_set_st.mc_v0 = VRPM_TO_VMM(machine_set_st.mc_set_speed * machine_set_st.mc_un_speed_fScale);
	machine_set_st.mc_t = machine_main_set_st.mc_t;
	if (machine_set_st.mc_sync_dir == RIGHT_DIR) //
	{
		machine_set_st.mc_acc_t = (1 - machine_set_st.mc_acc_scale) * machine_set_st.mc_v0 / machine_set_st.mc_acc;
		machine_set_st.mc_dec_t = (1 + machine_set_st.mc_acc_scale) * machine_set_st.mc_v0 / machine_set_st.mc_dec;
		machine_set_st.mc_un_speed_t = machine_set_st.mc_t - machine_set_st.mc_acc_t - machine_set_st.mc_dec_t;
		machine_set_st.mc_un_speed_acc = 2 * machine_set_st.mc_acc_scale * machine_set_st.mc_v0 / machine_set_st.mc_un_speed_t;
	}
	else //
	{
		machine_set_st.mc_acc_t = (1 + machine_set_st.mc_acc_scale) * machine_set_st.mc_v0 / machine_set_st.mc_acc;
		machine_set_st.mc_dec_t = (1 - machine_set_st.mc_acc_scale) * machine_set_st.mc_v0 / machine_set_st.mc_dec;
		machine_set_st.mc_un_speed_t = machine_set_st.mc_t - machine_set_st.mc_acc_t - machine_set_st.mc_dec_t;
		machine_set_st.mc_un_speed_acc = 2 * machine_set_st.mc_acc_scale * machine_set_st.mc_v0 / machine_set_st.mc_un_speed_t;
	}

	// machine_set_st.mc_un_speed_t = machine_main_set_st.mc_un_speed_t;
}
void machine_set_total_time(float t) //
{
	machine_main_set_st.mc_t = t;
	//	machine_set_scurve_t(t);
}
void machine_reset(void)
{
	machine_set_st.mc_target_pos = machine_set_st.mc_left_limit;
	machine_set_st.mc_sync_dir = LEFT_DIR;
	machine_set_st.mc_state = 0;
	machine_set_st.mc_per = 0.1;
	// machine_set_st.mc_per = 0.125;
	machine_set_st.mc_ticks = 1;
}
void machine_adv_pos_init(void)
{
	uint8_t i = 0;
	uint8_t j = 0;
	for (i = 0; i < 30; i++)
	{
		for (j = 0; j < 9; j++)
		{
			uint16_t dis;
			if (i == 0)
			{
				dis = adv_change_pos[i * 10 + 9] / 10;
				if (j == 0)
				{
					adv_change_pos[i * 10 + j] = dis;
				}
				else
				{
					adv_change_pos[i * 10 + j] = adv_change_pos[i * 10 + j - 1] + dis;
				}
			}
			else
			{
				dis = (adv_change_pos[(i) * 10 + 9] - adv_change_pos[(i - 1) * 10 + 9]) / 10;
				if (j == 0)
				{
					adv_change_pos[i * 10 + j] = adv_change_pos[(i - 1) * 10 + 9] + dis;
				}
				else
				{
					adv_change_pos[i * 10 + j] = adv_change_pos[i * 10 + j - 1] + dis;
				}
			}
		}
	}
}
uint8_t machine_get_oc(void)
{
	return machine_set_st.mc_oc;
}
void machine_set_oc(uint8_t oc)
{
	static uint8_t first = 1;
	if (oc > 0)
	{
		if (first == 1)
		{

			sync_un_speed_set.sync_cur_ext_pos = ENCODER_TO_MM(sync_get_ext_pos()); //
			sync_un_speed_set.sync_set_pos = sync_un_speed_set.sync_cur_ext_pos;
			sync_un_speed_set.sync_cur_set_pos = sync_un_speed_set.sync_cur_ext_pos;

			sync_set.sync_cur_ext_pos = sync_get_ext_pos(); //
			sync_set.sync_set_pos = sync_set.sync_cur_ext_pos;
			sync_set.sync_cur_set_pos = sync_set.sync_cur_ext_pos;
			first = 0;
		}
		// g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain =2 ; //
#ifndef _WIN32
		g_sM1Drive.sSpeed.sSpeedPiParams.fltUpperLim = motor_run.spdILmt;
		g_sM1Drive.sSpeed.sSpeedPiParams.fltLowerLim = -motor_run.spdILmt;
#endif // !_WIN32

		if (machine_set_st.mc_oc == 0)
		{
			machine_main_paras_use(2);
			machine_main_paras_use(0);
			machine_main_paras_use(1);
		}
	}
	else
	{
		// g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = 7 ;

#ifndef _WIN32
		g_sM1Drive.sPosCtrl.fltAdvSpeed = 0;

		g_sM1Drive.sSpeed.sSpeedPiParams.fltUpperLim = 4;
		g_sM1Drive.sSpeed.sSpeedPiParams.fltLowerLim = -4;
#endif
		machine_set_st.mc_state = 0;
	}
	machine_set_st.mc_oc = oc;
}
#if 0

#else
uint16_t machine_set_auto_spd(int mc_left_limit, int mc_right_limit, float mc_left_scale, float mc_right_scale, uint8_t dir) //
{
#if 0
	uint16_t spd_idx;
	int mc_center_limit;
	float fabs_dis_scale;
	float scale;
	if (dir == 0)
	{
		mc_center_limit = (mc_right_limit - mc_left_limit) / 2 + mc_left_limit;
	}
	else
	{

		mc_center_limit = mc_right_limit;
	}
	if (mc_right_scale == mc_left_scale)
		scale = 0;
	else
	{
		if (dir == 0)
			scale = (float)(sync_set.sync_cur_set_pos - mc_center_limit) / (float)((mc_right_limit - mc_left_limit) / 2);
		else
			scale = (float)(sync_set.sync_cur_set_pos - mc_left_limit) / (float)((mc_right_limit - mc_left_limit));
	}
	if (dir == 0)
		fabs_dis_scale = fabs((mc_left_scale - mc_right_scale) / 2) * scale;
	else
		fabs_dis_scale = fabs((mc_left_scale - mc_right_scale)) * scale;
	if (fabs_dis_scale != 0)
	{
		if (mc_right_scale >= mc_left_scale)
		{
			if (dir == 0)
				machine_set_st.mc_target_scale = mc_left_scale + fabs_dis_scale;
			else
				machine_set_st.mc_target_scale = mc_left_scale + fabs_dis_scale;
			//machine_set_st.mc_target_scale = mc_left_scale-fabs_dis_scale ;
			machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;

		}
		else
		{
			if (dir == 0)
				machine_set_st.mc_target_scale = mc_left_scale - fabs_dis_scale;
			else
				machine_set_st.mc_target_scale = mc_left_scale - fabs_dis_scale;
			//machine_set_st.mc_target_scale = mc_left_scale+fabs_dis_scale ;
			machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
		}
	}
	else
	{
		machine_set_st.mc_target_scale = mc_left_scale;
		machine_set_st.mc_cal_speed = mc_left_scale * machine_set_st.mc_set_speed;
	}
	/* spd_idx = machine_set_st.mc_cal_speed/10 ;
	 if(spd_idx<1 || spd_idx >300)
		 spd_idx = 1 ;
	 machine_set_st.mc_arr_dis = adv_change_pos[spd_idx-1] ;*/
	return machine_set_st.mc_cal_speed;
#else

	if (dir == 1) //
	{
		if (machine_set_st.mc_sync_dir == LEFT_DIR) //
		{
			machine_set_st.mc_target_scale = machine_set_st.mc_left_r2l_acc_speed_fScale;
			machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
		}
		else
		{
			machine_set_st.mc_target_scale = machine_set_st.mc_left_l2r_acc_speed_fScale;
			machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
		}
	}
	else if (dir == 2) //
	{
		if (machine_set_st.mc_sync_dir == LEFT_DIR) //
		{
			machine_set_st.mc_target_scale = machine_set_st.mc_right_r2l_acc_speed_fScale;
			machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
		}
		else
		{
			machine_set_st.mc_target_scale = machine_set_st.mc_right_l2r_acc_speed_fScale;
			machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
		}
	}
	else
	{
		int mc_center_limit;
		float fabs_dis_scale;
		float scale;
		mc_center_limit = (mc_right_limit - mc_left_limit) / 2 + mc_left_limit;

		if (mc_right_scale == mc_left_scale)
			scale = 0;
		else
		{

			scale = (float)(sync_set.sync_cur_set_pos - mc_center_limit) / (float)((mc_right_limit - mc_left_limit) / 2);
		}
		fabs_dis_scale = fabs((mc_left_scale - mc_right_scale) / 2) * scale;

		if (fabs_dis_scale != 0)
		{
#if 1
			if (mc_right_scale >= mc_left_scale)
			{
				machine_set_st.mc_target_scale = mc_left_scale + fabs_dis_scale;
				machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
			}
			else
			{

				machine_set_st.mc_target_scale = mc_left_scale - fabs_dis_scale;
				machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
			}
#else
			if (machine_set_st.mc_sync_dir == RIGHT_DIR)
			{
				machine_set_st.mc_target_scale = mc_left_scale + fabs_dis_scale;
				machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
			}
			else
			{

				machine_set_st.mc_target_scale = mc_left_scale - fabs_dis_scale;
				machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale * machine_set_st.mc_set_speed;
			}
#endif
		}
		else
		{
			machine_set_st.mc_target_scale = mc_left_scale;
			machine_set_st.mc_cal_speed = mc_left_scale * machine_set_st.mc_set_speed;
		}
		// machine_set_st.mc_target_scale = machine_set_st.mc_un_speed_fScale;
		// machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale  *machine_set_st.mc_set_speed ;
	}

	return machine_set_st.mc_cal_speed;
#endif
}
#endif
void machine_auto_cal_arr_dis(int32_t spd)
{
	uint16_t spd_idx;
	spd_idx = spd / 100;
	if (spd_idx < 1 || spd_idx > 30)
		spd_idx = 1;
	machine_set_st.mc_arr_pos_offset = adv_change_pos[spd_idx - 1];
}
void machine_set_target_spd(int32_t spd) //
{
	if (spd < 0)
		machine_set_st.mc_set_speed = -spd;
	else if (spd == 0)
	{
		//		machine_set_oc(0) ;
		//		M1_SetAppSwitch(0);
	}
	else
		machine_set_st.mc_set_speed = spd;

	//	machine_set_st.target_speed = machine_set_st.mc_left_scale*machine_set_st.mc_set_speed ;
}
void machine_set_left_limit(int32_t pos)
{
	machine_main_set_st.mc_left_limit = pos;
	machine_main_set_st.mc_left_limit_mm = ENCODER_TO_MM(machine_main_set_st.mc_left_limit);
	machine_main_set_st.mc_f_left_limit = posOut2In(machine_main_set_st.mc_left_limit);
}
void machine_set_right_limit(int32_t pos)
{

	machine_main_set_st.mc_right_limit = pos;
	machine_main_set_st.mc_right_limit_mm = ENCODER_TO_MM(machine_main_set_st.mc_right_limit);
	// machine_main_set_st.mc_f_right_limit = ENCODER_TO_MM(machine_main_set_st.mc_right_limit);

	machine_main_set_st.mc_f_right_limit = posOut2In(machine_main_set_st.mc_right_limit);
}

void machine_set_scale(uint8_t idx, float mot_value) //
{
	if (idx == 0)
	{
		machine_main_set_st.mc_left_scale = mot_value;
	}
	else //
	{
		machine_set_st.mc_acc_scale = mot_value;
	}
	if (machine_main_set_st.mc_left_scale > 1)
		machine_main_set_st.mc_left_scale = 1;
	if (machine_main_set_st.mc_right_scale > 1)
		machine_main_set_st.mc_right_scale = 1;
}
uint8_t machine_get_debug_flag(void)
{
	return machine_set_st.mc_debug_send_flag;
}
void machine_set_debug_flag(uint8_t flag)
{
	machine_set_st.mc_debug_send_flag = flag;
}
uint8_t machine_get_change_flag(void)
{
	return machine_set_st.mc_change_send_flag;
}
void machine_set_change_flag(uint8_t flag)
{
	machine_set_st.mc_change_send_flag = flag;
}
int32_t machine_get_arr_pos(void)
{
	return machine_set_st.mc_arr_pos;
}
void machine_set_cur_quan(uint32_t quan)
{
	machine_set_st.mc_cur_quan = quan;
}
void machine_set_report_quan(uint32_t quan)
{

	machine_set_st.mc_set_report_quan = machine_set_st.mc_cur_quan + quan;

	machine_set_st.mc_report_quan_flag = 1;
}
uint32_t machine_get_cur_quan(void)
{
	return machine_set_st.mc_cur_quan;
}
uint16_t machine_get_debug_value(void)
{
	return machine_set_st.mc_debug_value;
}
uint16_t machine_get_debug_value1(void)
{
	return machine_set_st.mc_debug_value1;
}
uint16_t machine_get_debug_canid(void)
{
	return machine_set_st.mc_debug_canid;
}
uint16_t machine_get_jr_encode(void)
{
	return machine_set_st.mc_jr_motor_encoder;
}

void machine_auto_cal_scale(void) //
{
	/*int32_t un_speed_left_limit, un_speed_right_limit; //
	uint16_t max_acc_limit_pos, var_acc_limit_pos, un_speed_limit_pos, var_dec_limit_pos;
	float acc_t, dec_t;

	acc_t = machine_set_st.mc_set_speed * machine_set_st.mc_un_speed_scale;

	if (machine_set_st.mc_sync_dir == LEFT_DIR) //
	{
		un_speed_left_limit = machine_set_st.mc_left_limit + machine_set_st.mc_left_r2l_acc_speed_pos;
		un_speed_right_limit = machine_set_st.mc_right_limit - machine_set_st.mc_right_r2l_acc_speed_pos;

	}
	else //
	{
		max_acc_limit_pos = machine_set_st.mc_left_limit + machine_set_st.mc_max_acc_pos;
		var_acc_limit_pos = max_acc_limit_pos + machine_set_st.mc_var_acc_pos;
		un_speed_left_limit = machine_set_st.mc_left_limit + machine_set_st.mc_left_l2r_acc_speed_pos;
		un_speed_right_limit = machine_set_st.mc_right_limit - machine_set_st.mc_right_l2r_acc_speed_pos;
	}*/
	int32_t un_speed_left_limit, un_speed_right_limit; //
	uint16_t max_acc_limit_pos, var_acc_limit_pos, un_speed_limit_pos, var_dec_limit_pos;
	if (machine_set_st.mc_sync_dir == LEFT_DIR) //
	{
		un_speed_left_limit = machine_set_st.mc_left_limit + machine_set_st.mc_left_r2l_acc_speed_pos;
		un_speed_right_limit = machine_set_st.mc_right_limit - machine_set_st.mc_right_r2l_acc_speed_pos;
	}
	else //
	{
		max_acc_limit_pos = machine_set_st.mc_left_limit + machine_set_st.mc_max_acc_pos;
		var_acc_limit_pos = max_acc_limit_pos + machine_set_st.mc_var_acc_pos;
		un_speed_left_limit = machine_set_st.mc_left_limit + machine_set_st.mc_left_l2r_acc_speed_pos;
		un_speed_right_limit = machine_set_st.mc_right_limit - machine_set_st.mc_right_l2r_acc_speed_pos;
	}
	/* machine_set_st.mc_init_speed_scale = machine_set_st.mc_acc_speed_scale *2 -machine_set_st.mc_un_speed_scale; //
	 un_speed_left_limit = machine_set_st.mc_left_limit + machine_set_st.mc_jr_acc_encoder ;
	 un_speed_right_limit = machine_set_st.mc_right_limit - machine_set_st.mc_jr_acc_encoder ;

	 machine_set_st.mc_init_speed_fScale = machine_set_st.mc_acc_speed_fScale *2 - machine_set_st.mc_un_speed_fScale;
	*/
	// machine_set_st.mc_un_speed_fScale = ((float)machine_set_st.mc_left_set_value1)/((float)machine_set_st.mc_un_speed_scale);
	if (sync_set.sync_set_pos <= un_speed_left_limit) //
	{
		machine_set_auto_spd(machine_set_st.mc_left_limit, un_speed_left_limit, 0, machine_set_st.mc_un_speed_fScale, 1);
		// machine_set_st.mc_target_scale = machine_set_st.mc_acc_speed_fScale;
		// machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale  *machine_set_st.mc_set_speed ;
	}
	else if (sync_set.sync_set_pos >= un_speed_right_limit) //
	{
		machine_set_auto_spd(un_speed_right_limit, machine_set_st.mc_right_limit, machine_set_st.mc_un_speed_fScale, 0, 2);
		// machine_set_st.mc_target_scale = machine_set_st.mc_acc_speed_fScale;
		// machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale  *machine_set_st.mc_set_speed ;
	}
	else //
	{
		machine_set_auto_spd(un_speed_left_limit, un_speed_right_limit, machine_set_st.mc_un_speed_fScale, machine_set_st.mc_un_speed_fScale * 1.0f, 0);
		// machine_set_st.mc_target_scale = machine_set_st.mc_un_speed_fScale;
		// machine_set_st.mc_cal_speed = machine_set_st.mc_target_scale  *machine_set_st.mc_set_speed ;
	}
}

#ifndef _WIN32
void machine_acc_dec_sync_deal(int push_pos) //
{

	char dir;
	uint16_t acce;
	static uint8_t end_follow_flag = 0;
	static uint8_t old_oc = 0;
	//	static int32_t old_push_pos = 0;
	static int32_t old_dis_pos = 0;
	uint16_t start_spd = 0; //
	uint8_t alarm_flag = 0;
	static uint8_t first = 1;
	uint16_t un_speed = 0;
	// machine_set_st.target_pos =0 ;
	if (first)
	{
		machine_set_st.mc_pre_push_pos = push_pos;
		machine_set_st.mc_max_push_pos = 0;
		first = 0;
	}
	// machine_set_st.mc_arr_pos_offset = 200;//80 200
	machine_set_st.mc_jr_motor_encoder = push_pos;
	machine_set_st.mc_cur_pos = posIn2Out(getCurPos());
	machine_set_st.mc_push_pos = push_pos - machine_set_st.mc_pre_push_pos;
	machine_set_st.mc_cur_speed = getCurSpd(0);
	if (machine_set_st.mc_push_pos < -(16384 / 2))
		machine_set_st.mc_push_pos = machine_set_st.mc_push_pos + 16384;
	if (machine_set_st.mc_push_pos > machine_set_st.mc_max_push_pos)
		machine_set_st.mc_max_push_pos = machine_set_st.mc_push_pos;
	machine_set_st.mc_pre_push_pos = push_pos;
	old_dis_pos = machine_set_st.mc_push_pos;

	if (machine_set_st.mc_oc == 0)
	{
		g_sM1Drive.sPosCtrl.fltAdvSpeed = 0;
		machine_set_st.state = 0;
		return;
	}
	machine_set_st.mc_taget_speed_scale = 1.2f;
	if (machine_set_st.mc_set_speed == 0)
		machine_set_st.mc_set_speed = SYNC_MIN_SPEED;
	machine_auto_cal_scale();
	machine_set_st.mc_jl_pos += (machine_set_st.mc_push_pos);
	if (machine_set_st.mc_jl_pos > 16383)
		machine_set_st.mc_jl_pos = machine_set_st.mc_jl_pos - 16383;
	g_sM1Drive.sPosCtrl.fltAdvSpeed = 0;
	if (machine_set_st.mc_sync_dir == LEFT_DIR)
	{
		if (machine_set_st.state == 0)
		{
			if (machine_set_st.mc_set_speed < 1000)
				machine_set_st.mc_target_pos = machine_set_st.mc_left_limit + 100;
			else
				machine_set_st.mc_target_pos = machine_set_st.mc_left_limit;
			machine_set_st.target_pos = machine_set_st.mc_target_pos + machine_set_st.mc_arr_dis;
		}
	}
	else
	{
		if (machine_set_st.state == 0)
		{
			if (machine_set_st.mc_set_speed < 1000)
				machine_set_st.mc_target_pos = machine_set_st.mc_right_limit - 100;
			else
				machine_set_st.mc_target_pos = machine_set_st.mc_right_limit;
			machine_set_st.target_pos = machine_set_st.mc_target_pos - machine_set_st.mc_arr_dis;
		}
	}

	if (machine_set_st.mc_cur_pos > machine_set_st.mc_right_max)
		machine_set_st.mc_right_max = machine_set_st.mc_cur_pos;
	if (machine_set_st.mc_cur_pos < machine_set_st.mc_left_max)
		machine_set_st.mc_left_max = machine_set_st.mc_cur_pos;
	if (machine_set_st.mc_cur_pos < 10000)
		machine_set_st.mc_right_max = 20000;
	if (machine_set_st.mc_cur_pos > 20000)
		machine_set_st.mc_left_max = 10000;
	machine_set_st.mc_right_fmax = posOut2In(machine_set_st.mc_right_max);
	machine_set_st.mc_left_fmax = posOut2In(machine_set_st.mc_left_max);
	//	machine_set_st.mc_right_max = posOut2In(machine_set_st.mc_left_max) ;
	if (machine_set_st.mc_cal_speed < MACHINE_MIN_SPEED)
		machine_set_st.mc_cal_speed = MACHINE_MIN_SPEED;
	un_speed = machine_set_st.mc_un_speed_fScale * machine_set_st.mc_set_speed;
	if (machine_set_st.mc_cal_speed > un_speed)
		machine_set_st.mc_cal_speed = un_speed;
	machine_set_st.mc_cal_speed = un_speed;
	switch (machine_set_st.state)
	{
	case 0:
		if (machine_set_st.mc_sync_dir == LEFT_DIR) //
		{

			float sync_cur_pos = 0;
			sync_set.sync_set_pos = sync_set.sync_set_pos - machine_set_st.mc_push_pos * machine_set_st.mc_target_scale;
			sync_set.sync_cur_set_pos = sync_set.sync_cur_set_pos - machine_set_st.mc_push_pos * machine_set_st.mc_left_r2l_fScale;
			if (sync_set.sync_cur_set_pos <= machine_set_st.target_pos)
			{
				if (sync_set.sync_cur_set_pos <= machine_set_st.mc_target_pos)
				{

					sync_set.sync_cur_set_pos = machine_set_st.mc_target_pos + (machine_set_st.mc_target_pos - sync_set.sync_cur_set_pos) / machine_set_st.mc_left_r2l_fScale * machine_set_st.mc_left_l2r_fScale;

					machine_main_paras_use(1);
					machine_main_paras_use(2);

					sync_set.sync_set_pos = sync_set.sync_cur_set_pos;
					// sync_set.sync_set_pos = sync_set.sync_set_pos + (machine_set_st.mc_target_pos - sync_set.sync_set_pos) ;
					machine_set_st.state = 2;
					mot_run_state.report_type = 6;
					machine_set_st.mc_arr_pos = machine_set_st.mc_left_limit;
					machine_set_st.mc_change_pos = machine_set_st.mc_right_limit;
					machine_set_st.mc_change_send_flag = 1;
				}
				sync_set.sync_target_pos = machine_set_st.mc_target_pos;
				// machine_auto_cal_arr_dis(machine_set_st.mc_cal_speed) ;
				mot_run_state.posReq = sync_set.sync_target_pos;
				machine_set_st.mc_arr_pos_flag = 0;
				/*if(sync_set.sync_target_pos >= machine_set_st.mc_right_limit )
					sync_set.sync_target_pos = machine_set_st.mc_right_limit;
				else if(sync_set.sync_target_pos <= machine_set_st.mc_left_limit)
					sync_set.sync_target_pos = machine_set_st.mc_left_limit ;*/
				movePos(posOut2In(sync_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed), -1);
				// g_sM1Drive.sPosCtrl.fltAdvSpeed = -(machine_set_st.mc_cal_speed/g_fltM1speedAngularScale*machine_set_st.mc_adv_scale)  ;
				// g_sM1Drive.sPosCtrl.fltAdvSpeed = 0.0f;
				if (machine_set_st.mc_cur_pos <= machine_set_st.target_pos)
				{
					g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_stop_pos_p;
				}
			}
			else
			{

				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_sync_pos_p;
				sync_set.sync_target_pos = sync_set.sync_set_pos;
				if (sync_set.sync_target_pos <= machine_set_st.mc_target_pos)
					sync_set.sync_target_pos = machine_set_st.mc_target_pos;
				/*if(sync_set.sync_target_pos >= machine_set_st.mc_right_limit )
					sync_set.sync_target_pos = machine_set_st.mc_right_limit;
				else if(sync_set.sync_target_pos <= machine_set_st.mc_left_limit)
					sync_set.sync_target_pos = machine_set_st.mc_left_limit ;*/
				movePos(posOut2In(sync_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed * machine_set_st.mc_taget_speed_scale), -1);
				// g_sM1Drive.sPosCtrl.fltAdvSpeed = (machine_set_st.mc_cal_speed / g_fltM1speedAngularScale * machine_set_st.mc_adv_scale);
			}
		}
		else //
		{
			float sync_cur_pos = 0;
			sync_set.sync_set_pos = sync_set.sync_set_pos + machine_set_st.mc_push_pos * machine_set_st.mc_target_scale;
			sync_set.sync_cur_set_pos = sync_set.sync_cur_set_pos + machine_set_st.mc_push_pos * machine_set_st.mc_left_l2r_fScale;

			if (sync_set.sync_cur_set_pos >= machine_set_st.target_pos)
			{
				if (sync_set.sync_cur_set_pos >= machine_set_st.mc_target_pos)
				{

					sync_set.sync_cur_set_pos = machine_set_st.mc_target_pos - (sync_set.sync_cur_set_pos - machine_set_st.mc_target_pos) / machine_set_st.mc_left_l2r_fScale * machine_set_st.mc_left_r2l_fScale;

					machine_main_paras_use(0);
					// sync_set.sync_set_pos = sync_set.sync_set_pos -(sync_set.sync_set_pos -machine_set_st.mc_target_pos ) ;

					sync_set.sync_set_pos = sync_set.sync_cur_set_pos;
					machine_set_st.state = 1;
					mot_run_state.report_type = 6;
					machine_set_st.mc_arr_pos = machine_set_st.mc_right_limit;
					// machine_set_st.mc_arr_pos =machine_set_st.mc_jr_motor_encoder ;
					machine_set_st.mc_change_pos = machine_set_st.mc_left_limit;
					machine_set_st.mc_change_send_flag = 1;
				}
				sync_set.sync_target_pos = machine_set_st.mc_target_pos;
				mot_run_state.posReq = sync_set.sync_target_pos;
				// mot_run_state.target_pos_offset = 100;
				machine_set_st.mc_arr_pos_flag = 0;
				// machine_auto_cal_arr_dis(machine_set_st.mc_cal_speed) ;
				/*if(sync_set.sync_target_pos >= machine_set_st.mc_right_limit )
					sync_set.sync_target_pos = machine_set_st.mc_right_limit;
				else if(sync_set.sync_target_pos <= machine_set_st.mc_left_limit)
					sync_set.sync_target_pos = machine_set_st.mc_left_limit ;*/
				movePos(posOut2In(sync_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed), -1);
				// g_sM1Drive.sPosCtrl.fltAdvSpeed =(machine_set_st.mc_cal_speed/g_fltM1speedAngularScale*machine_set_st.mc_adv_scale)  ;
				// g_sM1Drive.sPosCtrl.fltAdvSpeed = 0.0f;

				if (machine_set_st.mc_cur_pos >= machine_set_st.target_pos)
				{
					g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_stop_pos_p;
				}
			}
			else
			{
				sync_set.sync_target_pos = sync_set.sync_set_pos;
				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_sync_pos_p;
				if (sync_set.sync_target_pos >= machine_set_st.mc_target_pos)
					sync_set.sync_target_pos = machine_set_st.mc_target_pos;
				/*if(sync_set.sync_target_pos >= machine_set_st.mc_right_limit )
					sync_set.sync_target_pos = machine_set_st.mc_right_limit;
				else if(sync_set.sync_target_pos <= machine_set_st.mc_left_limit)
					sync_set.sync_target_pos = machine_set_st.mc_left_limit ;*/
				movePos(posOut2In(sync_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed * machine_set_st.mc_taget_speed_scale), -1);
				// g_sM1Drive.sPosCtrl.fltAdvSpeed = -(machine_set_st.mc_cal_speed / g_fltM1speedAngularScale * machine_set_st.mc_adv_scale);
			}
		}
		break;
	case 1: //
	{
		float sync_cur_pos = 0;
		sync_set.sync_set_pos = sync_set.sync_set_pos - machine_set_st.mc_push_pos * machine_set_st.mc_target_scale;
		sync_set.sync_cur_set_pos = sync_set.sync_cur_set_pos - machine_set_st.mc_push_pos * machine_set_st.mc_left_r2l_fScale;
		if (sync_set.sync_cur_set_pos <= machine_set_st.mc_left_limit)
		{
			sync_set.sync_set_pos = machine_set_st.mc_left_limit;
			sync_set.sync_cur_set_pos = machine_set_st.mc_left_limit;
		}
		// g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = 15.0f;
		if (machine_set_st.mc_arr_pos_flag == 1) //
		{
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_sync_pos_p;
			sync_set.sync_target_pos = sync_set.sync_set_pos;
			machine_set_st.mc_sync_dir = LEFT_DIR;
			/*if(sync_set.sync_target_pos >= machine_set_st.mc_right_limit )
				sync_set.sync_target_pos = machine_set_st.mc_right_limit;
			else if(sync_set.sync_target_pos <= machine_set_st.mc_left_limit)
				sync_set.sync_target_pos = machine_set_st.mc_left_limit ;*/
			movePos(posOut2In(sync_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed * machine_set_st.mc_taget_speed_scale), -1);
			// g_sM1Drive.sPosCtrl.fltAdvSpeed = (machine_set_st.mc_cal_speed / g_fltM1speedAngularScale * machine_set_st.mc_adv_scale);
#ifdef DEBUG_PRINT
			machine_set_st.mc_debug_send_flag = 1;
			machine_set_st.mc_debug_value = machine_set_st.mc_right_limit;
			machine_set_st.mc_debug_value1 = machine_set_st.mc_right_max;
			machine_set_st.mc_debug_canid = 0x105;
#endif
			machine_set_st.state = 0;
		}
		else
		{
			if (machine_set_st.mc_cur_pos >= machine_set_st.target_pos)
			{
				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_stop_pos_p;
			}
			/*if(sync_set.sync_target_pos >= machine_set_st.mc_right_limit )
				sync_set.sync_target_pos = machine_set_st.mc_right_limit;
			else if(sync_set.sync_target_pos <= machine_set_st.mc_left_limit)
				sync_set.sync_target_pos = machine_set_st.mc_left_limit ;*/
			movePos(posOut2In(sync_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed), -1);
			// g_sM1Drive.sPosCtrl.fltAdvSpeed = (machine_set_st.mc_cal_speed / g_fltM1speedAngularScale * machine_set_st.mc_adv_scale);
			// machine_set_st.mc_sync_dir = LEFT_DIR ;
		}
	}
	break;
	case 2: //
	{
		float sync_cur_pos = 0;
		sync_set.sync_set_pos = sync_set.sync_set_pos + machine_set_st.mc_push_pos * machine_set_st.mc_target_scale;
		sync_set.sync_cur_set_pos = sync_set.sync_cur_set_pos + machine_set_st.mc_push_pos * machine_set_st.mc_left_l2r_fScale;
		//
		if (sync_set.sync_cur_set_pos >= machine_set_st.mc_right_limit)
		{
			sync_set.sync_set_pos = machine_set_st.mc_right_limit;
			sync_set.sync_cur_set_pos = machine_set_st.mc_right_limit;
		}
		if (machine_set_st.mc_arr_pos_flag == 1) //
		{
			machine_change_mode();
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_sync_pos_p;
			sync_set.sync_target_pos = sync_set.sync_set_pos;
			machine_set_st.mc_sync_dir = RIGHT_DIR;
			/*if(sync_set.sync_target_pos >= machine_set_st.mc_right_limit )
			sync_set.sync_target_pos = machine_set_st.mc_right_limit;
			else if(sync_set.sync_target_pos <= machine_set_st.mc_left_limit)
			sync_set.sync_target_pos = machine_set_st.mc_left_limit ;*/
			movePos(posOut2In(sync_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed * machine_set_st.mc_taget_speed_scale), -1);
			// g_sM1Drive.sPosCtrl.fltAdvSpeed = -(machine_set_st.mc_cal_speed / g_fltM1speedAngularScale * machine_set_st.mc_adv_scale);
#ifdef DEBUG_PRINT
			machine_set_st.mc_debug_send_flag = 1;
			machine_set_st.mc_debug_value = machine_set_st.mc_left_limit;
			machine_set_st.mc_debug_value1 = machine_set_st.mc_left_max;
			machine_set_st.mc_debug_canid = 0x106;
#endif
			machine_set_st.state = 0;
		}
		else
		{
			if (machine_set_st.mc_cur_pos <= machine_set_st.target_pos)
			{
				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_stop_pos_p;
			}
			/*if(sync_set.sync_target_pos >= machine_set_st.mc_right_limit )
			sync_set.sync_target_pos = machine_set_st.mc_right_limit;
			else if(sync_set.sync_target_pos <= machine_set_st.mc_left_limit)
			sync_set.sync_target_pos = machine_set_st.mc_left_limit ;*/
			movePos(posOut2In(sync_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed), -1);
			// g_sM1Drive.sPosCtrl.fltAdvSpeed = (machine_set_st.mc_cal_speed / g_fltM1speedAngularScale * machine_set_st.mc_adv_scale);
			// machine_set_st.mc_sync_dir = RIGHT_DIR ;
		}
	}

	break;
	}
}
#endif // !_WIN32

#if S_CURVE
float machine_cal_scurve_v(uint8_t type, float cur_t) //
{
	float v = 0;
	switch (type)
	{
	case RUN_SCURVE_T1:
		v = pScurveSt->s_curve_jj * cur_t * cur_t / 2;
		//	pScurveSt->s_curve_v1 = pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T1 / 2;
		break;
	case RUN_SCURVE_T2:
		v = pScurveSt->s_curve_v1 + pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * cur_t;
		//	pScurveSt->s_curve_v2 = pScurveSt->s_curve_v1 + pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T2;
		break;
	case RUN_SCURVE_T3:
		v = pScurveSt->s_curve_v2 + pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * cur_t - pScurveSt->s_curve_jj * cur_t * cur_t / 2;
		//	pScurveSt->s_curve_v3 = pScurveSt->s_curve_v2 + pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T1 / 2;
		break;
	case RUN_SCURVE_T4:
		v = pScurveSt->s_curve_v3;
		//	pScurveSt->s_curve_v4 = v;
		break;
	case RUN_SCURVE_T5:
		v = pScurveSt->s_curve_v4 - pScurveSt->s_curve_jj * cur_t * cur_t / 2;
		//	pScurveSt->s_curve_v5 = pScurveSt->s_curve_v4 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T5 * pScurveSt->s_curve_T5 / 2;
		break;
	case RUN_SCURVE_T6:
		v = pScurveSt->s_curve_v5 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T5 * cur_t;
		//	pScurveSt->s_curve_v6 = pScurveSt->s_curve_v5 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T5 * pScurveSt->s_curve_T6;
		break;
	case RUN_SCURVE_T7:
		v = pScurveSt->s_curve_v6 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T5 * cur_t + pScurveSt->s_curve_jj * cur_t * cur_t / 2;
		//	pScurveSt->s_curve_v7 = pScurveSt->s_curve_v6 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T7 * pScurveSt->s_curve_T7 / 2;
		break;
	default:
		break;
	}
	return v;
}
float machine_cal_scurve_s(uint8_t type, float cur_t) //
{
	float s = 0;
	switch (type)
	{
	case RUN_SCURVE_T1:
		s = pScurveSt->s_curve_jj * cur_t * cur_t * cur_t / 6;
		//		pScurveSt->s_curve_s1 = pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T1 / 6;
		break;
	case RUN_SCURVE_T2:
		s = pScurveSt->s_curve_s1 + pScurveSt->s_curve_v1 * cur_t + pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * cur_t * cur_t / 2;
		//		pScurveSt->s_curve_s2 = pScurveSt->s_curve_s1 + pScurveSt->s_curve_v1 * pScurveSt->s_curve_T2 + pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T2 * pScurveSt->s_curve_T2 / 2;
		break;
	case RUN_SCURVE_T3:
		s = pScurveSt->s_curve_s2 + pScurveSt->s_curve_v2 * cur_t + pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * cur_t * cur_t / 2 - pScurveSt->s_curve_jj * cur_t * cur_t * cur_t / 6;
		//		pScurveSt->s_curve_s3 = pScurveSt->s_curve_s2 + pScurveSt->s_curve_v2 * pScurveSt->s_curve_T3 + pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T3 * pScurveSt->s_curve_T3 / 2 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T3 * pScurveSt->s_curve_T3 * pScurveSt->s_curve_T3 / 6;;
		break;
	case RUN_SCURVE_T4:
		s = pScurveSt->s_curve_s3 + pScurveSt->s_curve_v3 * cur_t;
		//		pScurveSt->s_curve_s4 = pScurveSt->s_curve_s3 + pScurveSt->s_curve_v3 * pScurveSt->s_curve_T4;
		break;
	case RUN_SCURVE_T5:
		s = pScurveSt->s_curve_s4 + pScurveSt->s_curve_v4 * cur_t - pScurveSt->s_curve_jj * cur_t * cur_t * cur_t / 6;
		//		pScurveSt->s_curve_s5 = pScurveSt->s_curve_s4 + pScurveSt->s_curve_v4 * pScurveSt->s_curve_T5 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T5 * pScurveSt->s_curve_T5 * pScurveSt->s_curve_T5 / 6;
		break;
	case RUN_SCURVE_T6:
		s = pScurveSt->s_curve_s5 + pScurveSt->s_curve_v5 * cur_t - pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * cur_t * cur_t / 2;
		//		pScurveSt->s_curve_s6 = pScurveSt->s_curve_s5 + pScurveSt->s_curve_v5 * pScurveSt->s_curve_T6 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T6 * pScurveSt->s_curve_T6 / 2;
		break;
	case RUN_SCURVE_T7:
		s = pScurveSt->s_curve_s6 + pScurveSt->s_curve_v6 * cur_t - pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * cur_t * cur_t / 2 + pScurveSt->s_curve_jj * cur_t * cur_t * cur_t / 6;
		//		pScurveSt->s_curve_s7 = pScurveSt->s_curve_s6 + pScurveSt->s_curve_v6 * pScurveSt->s_curve_T7 - pScurveSt->s_curve_jj * pScurveSt->s_curve_T1 * pScurveSt->s_curve_T7 * pScurveSt->s_curve_T7 / 2 + pScurveSt->s_curve_jj * pScurveSt->s_curve_T7 * pScurveSt->s_curve_T7 * pScurveSt->s_curve_T7 / 6;
		break;
	default:
		break;
	}
	return s;
}
#else
float machine_cal_scurve_v(uint8_t type, float cur_t) //
{
	float v = 0;
	switch (type)
	{
	case RUN_SCURVE_T1:
		v = pScurveSt->s_curve_a_max * cur_t;
		// SCurveSt.s_curve_v1 = SCurveSt.s_curve_a_max* SCurveSt.s_curve_T1;
		break;
	case RUN_SCURVE_T2:
		if (pScurveSt->s_curve_T2 == 0)
		{
			v = pScurveSt->s_curve_v1;
			pScurveSt->s_curve_v2 = v;
		}
		else
		{
			v = pScurveSt->s_curve_v1 + pScurveSt->s_curve_a2 * cur_t;
		}
		break;
	case RUN_SCURVE_T3:
		v = pScurveSt->s_curve_v2 + pScurveSt->s_curve_a3 * cur_t;
		break;
	case RUN_SCURVE_T4:

		v = pScurveSt->s_curve_v3 - pScurveSt->s_curve_a4 * cur_t;

		break;
	case RUN_SCURVE_T5:
		v = pScurveSt->s_curve_v4 - pScurveSt->s_curve_a_max * cur_t;

		break;
		/*	case RUN_SCURVE_T6:
				v = SCurveSt.s_curve_v5 - SCurveSt.s_curve_jj * SCurveSt.s_curve_T5 * cur_t;
				SCurveSt.s_curve_v6 = SCurveSt.s_curve_v5 - SCurveSt.s_curve_jj * SCurveSt.s_curve_T5 * SCurveSt.s_curve_T6;
				break;
			case RUN_SCURVE_T7:
				v = SCurveSt.s_curve_v6 - SCurveSt.s_curve_jj * cur_t + SCurveSt.s_curve_jj * cur_t * cur_t / 2;
				SCurveSt.s_curve_v7 = SCurveSt.s_curve_v6 - SCurveSt.s_curve_jj * SCurveSt.s_curve_T7 * SCurveSt.s_curve_T7 / 2;
				break;*/
	default:
		break;
	}
	return v;
}
float machine_cal_scurve_s(uint8_t type, float cur_t) //
{
	float s = 0;
	switch (type)
	{
	case RUN_SCURVE_T1:
		s = 0.5f * pScurveSt->s_curve_a_max * cur_t * cur_t;
		//	SCurveSt.s_curve_s1 = 0.5f * SCurveSt.s_curve_a_max * SCurveSt.s_curve_T1 * SCurveSt.s_curve_T1;
		break;
	case RUN_SCURVE_T2:
		s = pScurveSt->s_curve_s1 + pScurveSt->s_curve_v1 * cur_t + pScurveSt->s_curve_a2 * cur_t * cur_t / 2;
		//	SCurveSt.s_curve_s2 = SCurveSt.s_curve_s1 + SCurveSt.s_curve_v1 * SCurveSt.s_curve_T2 + SCurveSt.s_curve_a2 *SCurveSt.s_curve_T2 * SCurveSt.s_curve_T2 / 2;
		break;
	case RUN_SCURVE_T3:
		s = pScurveSt->s_curve_s2 + pScurveSt->s_curve_v2 * cur_t + 0.5f * pScurveSt->s_curve_a3 * cur_t * cur_t;
		//	SCurveSt.s_curve_s3 = SCurveSt.s_curve_s2 + SCurveSt.s_curve_v2 * SCurveSt.s_curve_T3 + 0.5f * SCurveSt.s_curve_a3 * SCurveSt.s_curve_T3 * SCurveSt.s_curve_T3;
		break;
	case RUN_SCURVE_T4:
		s = pScurveSt->s_curve_s3 + pScurveSt->s_curve_v3 * cur_t - pScurveSt->s_curve_a4 * cur_t * cur_t / 2;
		//	SCurveSt.s_curve_s4 = SCurveSt.s_curve_s3 + SCurveSt.s_curve_v3 * SCurveSt.s_curve_T4 - SCurveSt.s_curve_a4 * SCurveSt.s_curve_T4 * SCurveSt.s_curve_T4 / 2;
		break;
	case RUN_SCURVE_T5:
		s = pScurveSt->s_curve_s4 + pScurveSt->s_curve_v4 * cur_t - pScurveSt->s_curve_a_max * cur_t * cur_t / 2;
		//	SCurveSt.s_curve_s5 = SCurveSt.s_curve_s4 + SCurveSt.s_curve_v4 * SCurveSt.s_curve_T5 - SCurveSt.s_curve_a_max * SCurveSt.s_curve_T5 * SCurveSt.s_curve_T5 / 2;
		break;
		/*	case RUN_SCURVE_T6:
				s = SCurveSt.s_curve_s5 + SCurveSt.s_curve_v5 * cur_t - SCurveSt.s_curve_jj * SCurveSt.s_curve_T1 * cur_t * cur_t / 2;
				SCurveSt.s_curve_s6 = SCurveSt.s_curve_s5 + SCurveSt.s_curve_v5 * SCurveSt.s_curve_T6 - SCurveSt.s_curve_jj * SCurveSt.s_curve_T1 * SCurveSt.s_curve_T6 * SCurveSt.s_curve_T6 / 2;
				break;
			case RUN_SCURVE_T7:
				s = SCurveSt.s_curve_s6 + SCurveSt.s_curve_v6 * cur_t - SCurveSt.s_curve_jj * SCurveSt.s_curve_T1 * cur_t * cur_t / 2 + SCurveSt.s_curve_jj * cur_t * cur_t * cur_t / 6;
				SCurveSt.s_curve_s7 = SCurveSt.s_curve_s6 + SCurveSt.s_curve_v6 * SCurveSt.s_curve_T7 - SCurveSt.s_curve_jj * SCurveSt.s_curve_T1 * SCurveSt.s_curve_T7 * SCurveSt.s_curve_T7 / 2 + SCurveSt.s_curve_jj * SCurveSt.s_curve_T7 * SCurveSt.s_curve_T7 * SCurveSt.s_curve_T7 / 6;
				break;*/
	default:
		break;
	}
	return s;
}
#endif

uint16_t av_speed = 0;
float g_s1 = 0;
float g_s2 = 0;
void machine_sync_deal(int push_pos) // push_pos
{

	char dir;
	uint16_t acce;
	static uint8_t end_follow_flag = 0;
	static uint8_t old_oc = 0;
	static int32_t old_dis_pos = 0;
	uint16_t start_spd = 0; //
	uint8_t alarm_flag = 0;
	float cur_t = 0;
	uint16_t un_speed = 0;
	uint32_t av_push_pos = 0;
	uint8_t speed_mode_flag = 0;
// machine_set_st.target_pos =0 ;
#ifdef TEST_SYNC
	static uint8_t first = 1;
	if (first)
	{
		machine_set_st.mc_pre_push_pos = push_pos;
		machine_set_st.mc_max_push_pos = 0;
		first = 0;
	}
#endif
	machine_set_st.mc_arr_pos_offset = 200; // 150
	machine_set_st.mc_jr_motor_encoder = push_pos;
	machine_set_st.mc_cur_pos = posIn2Out(getCurPos());
	machine_set_st.mc_push_pos = push_pos - machine_set_st.mc_pre_push_pos;
	machine_set_st.mc_cur_speed = getCurSpd(0);
	if (machine_set_st.mc_push_pos < -(16383 / 2))
		machine_set_st.mc_push_pos = machine_set_st.mc_push_pos + 16383;
	if (machine_set_st.mc_push_pos > machine_set_st.mc_max_push_pos)
		machine_set_st.mc_max_push_pos = machine_set_st.mc_push_pos;
	machine_set_st.mc_pre_push_pos = push_pos;
	old_dis_pos = machine_set_st.mc_push_pos;
#ifndef _WIN32
/*av_set_data(1,machine_set_st.mc_push_pos) ;
av_push_pos = av_get_data(1);
av_speed = av_push_pos * 600000 /16384;
return ;*/
#endif
	if (machine_set_st.mc_oc == 0)
	{
#ifndef _WIN32
		g_sM1Drive.sPosCtrl.fltAdvSpeed = 0;
#endif

#ifndef TEST_SYNC
		machine_set_st.mc_state = 0;
#endif
		// machine_set_st.mc_jl_pos =0 ;
		return;
	}
	if (machine_set_st.mc_set_speed == 0)
		machine_set_st.mc_set_speed = SYNC_MIN_SPEED;

	//	 machine_auto_cal_scale();
	machine_set_st.mc_jl_pos += (machine_set_st.mc_push_pos);
	if (machine_set_st.mc_jl_pos > 16383)
		machine_set_st.mc_jl_pos = machine_set_st.mc_jl_pos - 16383;
	if (machine_set_st.mc_cur_pos > machine_set_st.mc_right_max)
		machine_set_st.mc_right_max = machine_set_st.mc_cur_pos;
	if (machine_set_st.mc_cur_pos < machine_set_st.mc_left_max)
		machine_set_st.mc_left_max = machine_set_st.mc_cur_pos;
	if (machine_set_st.mc_cur_pos < 10000)
		machine_set_st.mc_right_max = 20000;
	if (machine_set_st.mc_cur_pos > 20000)
		machine_set_st.mc_left_max = 10000;
	machine_set_st.mc_right_fmax = posOut2In(machine_set_st.mc_right_max);
	machine_set_st.mc_left_fmax = posOut2In(machine_set_st.mc_left_max);
	if (machine_set_st.mc_cal_speed < MACHINE_MIN_SPEED)
		machine_set_st.mc_cal_speed = MACHINE_MIN_SPEED;
	un_speed = machine_set_st.mc_un_speed_fScale * machine_set_st.mc_set_speed;
	if (machine_set_st.mc_cal_speed > un_speed)
		machine_set_st.mc_cal_speed = un_speed;
	machine_set_st.mc_cal_speed = un_speed;
#ifndef _WIN32
	g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;
#endif
	if (machine_set_st.mc_sync_dir == LEFT_DIR) //
	{
		pScurveSt = &SCurve_rtl_St;
		// sync_un_speed_set.sync_cur_set_pos = sync_un_speed_set.sync_cur_set_pos - machine_set_st.mc_push_pos * machine_set_st.mc_left_scale;
		sync_un_speed_set.sync_cur_set_pos = sync_un_speed_set.sync_cur_set_pos - machine_set_st.mc_push_pos * machine_set_st.mc_left_r2l_fScale;
		cur_t = (machine_set_st.mc_right_limit - sync_un_speed_set.sync_cur_set_pos) / (machine_set_st.mc_right_limit - machine_set_st.mc_left_limit) * pScurveSt->s_curve_T;
		if (sync_un_speed_set.sync_cur_set_pos <= machine_set_st.mc_left_limit) //
		{
			machine_main_paras_use(1);
			machine_main_paras_use(2);
#ifdef TEST_SYNC
// machine_set_st.mc_oc = 0 ;
#endif
			sync_un_speed_set.sync_cur_set_pos = machine_set_st.mc_left_limit + (machine_set_st.mc_left_limit - sync_un_speed_set.sync_cur_set_pos) / machine_set_st.mc_left_r2l_fScale * machine_set_st.mc_left_l2r_fScale;
			// sync_un_speed_set.sync_cur_set_pos = machine_set_st.mc_left_limit + (machine_set_st.mc_left_limit - sync_un_speed_set.sync_cur_set_pos);
			machine_set_st.mc_state = RUN_CHANGE_DIR;
			machine_set_st.mc_ticks = 0;
#ifndef _WIN32
			mot_run_state.posReq = machine_set_st.mc_left_limit;
#endif
			machine_set_st.mc_arr_pos_flag = 0;
			sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm;

			machine_set_st.mc_sync_dir = RIGHT_DIR;
			machine_change_mode();
			if (machine_set_st.mc_acc_dec_flag == 1)
				return;
		}
	}
	else //
	{
		pScurveSt = &SCurve_ltr_St;
		// sync_un_speed_set.sync_cur_set_pos = sync_un_speed_set.sync_cur_set_pos + machine_set_st.mc_push_pos * machine_set_st.mc_left_scale;
		sync_un_speed_set.sync_cur_set_pos = sync_un_speed_set.sync_cur_set_pos + machine_set_st.mc_push_pos * machine_set_st.mc_left_l2r_fScale;
		cur_t = (sync_un_speed_set.sync_cur_set_pos - machine_set_st.mc_left_limit) / (machine_set_st.mc_right_limit - machine_set_st.mc_left_limit) * pScurveSt->s_curve_T;
		if (sync_un_speed_set.sync_cur_set_pos >= machine_set_st.mc_right_limit) //
		{
			machine_main_paras_use(0);
			sync_un_speed_set.sync_cur_set_pos = machine_set_st.mc_right_limit + (machine_set_st.mc_right_limit - sync_un_speed_set.sync_cur_set_pos) / machine_set_st.mc_left_r2l_fScale * machine_set_st.mc_left_l2r_fScale;
#ifdef TEST_SYNC
// machine_set_st.mc_oc = 0 ;
#endif
			// sync_un_speed_set.sync_cur_set_pos = machine_set_st.mc_right_limit - (sync_un_speed_set.sync_cur_set_pos - machine_set_st.mc_right_limit);
			machine_set_st.mc_state = RUN_CHANGE_DIR;
			machine_set_st.mc_ticks = 0;
#ifndef _WIN32
			mot_run_state.posReq = machine_set_st.mc_right_limit;
#endif
			machine_set_st.mc_arr_pos_flag = 0;
			sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm;
			machine_set_st.mc_sync_dir = LEFT_DIR;
		}
	}
	switch (machine_set_st.mc_state)
	{
	case RUN_SCURVE_INIT: //

		// machine_set_st.mc_target_scale = machine_set_st.mc_left_scale;
		if (sync_un_speed_set.sync_set_pos <= machine_set_st.mc_left_limit_mm) //
		{
			// sync_set.sync_set_pos = machine_main_set_st.mc_f_left_limit + (machine_main_set_st.mc_f_left_limit - sync_set.sync_set_pos);
			machine_set_st.mc_state = RUN_SCURVE_T1;
			machine_set_st.mc_ticks = 1;
			machine_set_st.mc_target_scale = 0;
			machine_set_st.mc_sync_dir = RIGHT_DIR;
			machine_set_st.mc_motor_dir = RIGHT_DIR;
		}
		else
		{
			sync_un_speed_set.sync_set_pos = sync_un_speed_set.sync_set_pos - ENCODER_TO_MM(machine_set_st.mc_push_pos) * machine_set_st.mc_left_r2l_fScale;
			machine_set_st.mc_cal_speed = machine_set_st.mc_set_speed * machine_set_st.mc_left_r2l_fScale;
		}

		break;
	case RUN_SCURVE_T1: // T1
	{
		float v0, s, t1, t2, t3, t4, t5, t6, t7;
		t1 = pScurveSt->s_curve_T1;
		if (pScurveSt->s_curve_T2 == 0)
		{
			t2 = t1;
		}
		else
		{
			t2 = pScurveSt->s_curve_T2 + pScurveSt->s_curve_T1;
		}
		t3 = pScurveSt->s_curve_T3 + t2;
		if (pScurveSt->s_curve_T4 == 0)
		{
			t4 = t3;
		}
		else
		{
			t4 = t3 + pScurveSt->s_curve_T4;
		}
		t5 = t4 + pScurveSt->s_curve_T5;
#ifdef S_CURVE
		t6 = t5 + pScurveSt->s_curve_T6;
		t7 = t6 + pScurveSt->s_curve_T7;
#endif // S_CURVE

		if (cur_t >= 0 && cur_t <= t1)
		{
#ifndef _WIN32
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;
#endif // ! win32

			v0 = machine_cal_scurve_v(RUN_SCURVE_T1, cur_t); // 1/2*JT^2
			s = machine_cal_scurve_s(RUN_SCURVE_T1, cur_t);
			if (machine_set_st.mc_motor_dir == LEFT_DIR) //
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
			}
			else
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
			}
			machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
#ifndef TEST_SYNC
			machine_set_st.mc_adv_scale = 1.0f;
			machine_set_st.mc_taget_speed_scale = 1.0f;
#else
			// machine_set_st.mc_adv_scale = 0.0f ;
			machine_set_st.mc_taget_speed_scale = 1.3f;
#endif
		}
		else if (cur_t > t1 && cur_t <= t2)
		{
#ifndef _WIN32
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;
#endif
			v0 = machine_cal_scurve_v(RUN_SCURVE_T2, cur_t - t1); // 1/2*JT^2
			s = machine_cal_scurve_s(RUN_SCURVE_T2, cur_t - t1);
			if (machine_set_st.mc_motor_dir == LEFT_DIR) //
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
			}
			else
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
			}
			machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
#ifndef TEST_SYNC
			machine_set_st.mc_adv_scale = 1.0f;
			machine_set_st.mc_taget_speed_scale = 1.0f;
#else
			// machine_set_st.mc_adv_scale = 0.0f ;
			machine_set_st.mc_taget_speed_scale = 1.3f;
#endif
			// machine_set_st.mc_cal_speed = VMM_TO_VRPM(pScurveSt->s_curve_v2);
		}
		else if (cur_t > t2 && cur_t <= t3)
		{
			// g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;
			/*if (cur_t > t2 && cur_t <= t2 + (t3 - t2) / 2)
			{
				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;

			}
			else
				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_dec_pos_p;*/
#ifndef _WIN32
			/*if (cur_t > t2 && cur_t <= t2 + (t3 - t2) / 4)
			{
				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;

			}
			else*/
			{

				machine_set_st.mc_adv_scale = 1.0f;
#ifndef TEST_SYNC
				machine_set_st.mc_taget_speed_scale = 1.0f;
#else
				machine_set_st.mc_taget_speed_scale = 1.3f;
#endif
			}
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;
#endif
			v0 = machine_cal_scurve_v(RUN_SCURVE_T3, cur_t - t2); // 1/2*JT^2
			g_s1 = v0 * (cur_t - t2);

			s = machine_cal_scurve_s(RUN_SCURVE_T3, cur_t - t2);
			g_s2 = s - pScurveSt->s_curve_s2;
			if (machine_set_st.mc_motor_dir == LEFT_DIR) //
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
			}
			else
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
			}
			machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
		}
		else if (cur_t > t3 && cur_t <= t4)
		{
#ifndef WIN32
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_dec_pos_p;
#endif
			v0 = machine_cal_scurve_v(RUN_SCURVE_T4, cur_t - t3); // 1/2*JT^2
			s = machine_cal_scurve_s(RUN_SCURVE_T4, cur_t - t3);
			if (machine_set_st.mc_motor_dir == LEFT_DIR) //
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
			}
			else
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
			}
			machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
			machine_set_st.mc_adv_scale = 1.0f;
#ifndef TEST_SYNC
			machine_set_st.mc_taget_speed_scale = 1.0f;
#else
			machine_set_st.mc_taget_speed_scale = 1.3f;
#endif
		}
		else if (cur_t > t4 && cur_t <= t5)
		{
#ifndef _WIN32
			machine_set_st.mc_adv_scale = 1.0f;
#ifndef TEST_SYNC
			machine_set_st.mc_taget_speed_scale = 1.0f;
#else
			machine_set_st.mc_taget_speed_scale = 1.3f;
#endif
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_dec_pos_p;
#endif
			v0 = machine_cal_scurve_v(RUN_SCURVE_T5, cur_t - t4); // 1/2*JT^2
			s = machine_cal_scurve_s(RUN_SCURVE_T5, cur_t - t4);
			if (machine_set_st.mc_motor_dir == LEFT_DIR) //
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
			}
			else
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
			}
			machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
			// speed_mode_flag =1 ;
		}
#ifdef S_CURVE
		else if (cur_t > t5 && cur_t <= t6)
		{
#ifndef _WIN32
			/*if (cur_t > t2 && cur_t <= t2 + (t3 - t2) / 4)
			{
				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;

			}
			else*/
			{

				machine_set_st.mc_adv_scale = 1.0f;
#ifndef TEST_SYNC
				machine_set_st.mc_taget_speed_scale = 1.1f;
#else
				machine_set_st.mc_taget_speed_scale = 1.01f;
#endif
			}
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_dec_pos_p;
#endif
			v0 = machine_cal_scurve_v(RUN_SCURVE_T6, cur_t - t5); // 1/2*JT^2
			s = machine_cal_scurve_s(RUN_SCURVE_T6, cur_t - t5);
			if (machine_set_st.mc_motor_dir == LEFT_DIR) //
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
			}
			else
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
			}
			machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
		}
		else if (cur_t > t6 && cur_t <= t7)
		{
#ifndef _WIN32
			/*if (cur_t > t2 && cur_t <= t2 + (t3 - t2) / 4)
			{
				g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_un_speed_pos_p;

			}
			else*/
			{

				machine_set_st.mc_adv_scale = 1.0f;
#ifndef TEST_SYNC
				machine_set_st.mc_taget_speed_scale = 1.1f;
#else
				machine_set_st.mc_taget_speed_scale = 1.01f;
#endif
			}
			g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain = machine_set_st.mc_dec_pos_p;
#endif
			v0 = machine_cal_scurve_v(RUN_SCURVE_T7, cur_t - t6); // 1/2*JT^2
			s = machine_cal_scurve_s(RUN_SCURVE_T7, cur_t - t6);
			if (machine_set_st.mc_motor_dir == LEFT_DIR) //
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
			}
			else
			{
				sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
			}
			machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
		}
#endif
		/*
				float cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
				float v0, s;
				machine_set_st.mc_ticks++;
				if (cur_t > SCurveSt.s_curve_T1) //T2
				{

					machine_change_mode() ;
					if(machine_set_st.mc_acc_dec_flag)
					return ;
					machine_set_st.mc_ticks = 1;
					cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
					if (SCurveSt.s_curve_T2 == 0)
					{
						SCurveSt.s_curve_s2 = SCurveSt.s_curve_s1;
						v0 = machine_cal_scurve_v(RUN_SCURVE_T3, cur_t); // 1/2*JT^2
						s = machine_cal_scurve_s(RUN_SCURVE_T3, cur_t);
						machine_set_st.mc_state = RUN_SCURVE_T3;
					}
					else
					{
						v0 = machine_cal_scurve_v(RUN_SCURVE_T2, cur_t); // 1/2*JT^2
						s = machine_cal_scurve_s(RUN_SCURVE_T2, cur_t);
						machine_set_st.mc_state = RUN_SCURVE_T2;
					}
				}
				else
				{
					v0 = machine_cal_scurve_v(RUN_SCURVE_T1, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T1, cur_t);

					//	machine_set_st.mc_target_scale = VMM_TO_VRPM(v0) / machine_set_st.mc_set_speed; //

				}
				if (machine_set_st.mc_motor_dir == LEFT_DIR) //
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
				}
				else
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
				}
				machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);

		*/
	}

	break;
		/*	case RUN_SCURVE_T2: //T2
			{
				float cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
				float v0, s;
				machine_set_st.mc_ticks++;
				if (cur_t > SCurveSt.s_curve_T2) //T3
				{
					machine_set_st.mc_ticks = 1;
					cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
					v0 = machine_cal_scurve_v(RUN_SCURVE_T3, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T3, cur_t);
					machine_set_st.mc_state = RUN_SCURVE_T3;
				}
				else
				{
					v0 = machine_cal_scurve_v(RUN_SCURVE_T2, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T2, cur_t);
				}
				if (machine_set_st.mc_motor_dir == LEFT_DIR) //
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
				}
				else
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
				}
				machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);

			}
			break;
			case RUN_SCURVE_T3: // T3
			{
				float cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
				float v0, s;
				machine_set_st.mc_ticks++;
				if (cur_t > SCurveSt.s_curve_T3) //T4
				{
					machine_set_st.mc_ticks = 1;
					cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
					if (SCurveSt.s_curve_T4 == 0)
					{
						SCurveSt.s_curve_s4= SCurveSt.s_curve_s3;
						v0 = machine_cal_scurve_v(RUN_SCURVE_T5, cur_t); // 1/2*JT^2
						s = machine_cal_scurve_s(RUN_SCURVE_T5, cur_t);
						machine_set_st.mc_state = RUN_SCURVE_T5;
					}
					else
					{
						v0 = machine_cal_scurve_v(RUN_SCURVE_T4, cur_t); // 1/2*JT^2
						s = machine_cal_scurve_s(RUN_SCURVE_T4, cur_t);
						machine_set_st.mc_state = RUN_SCURVE_T4;
					}
				}
				else
				{
					v0 = machine_cal_scurve_v(RUN_SCURVE_T3, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T3, cur_t);
				}
				if (machine_set_st.mc_motor_dir == LEFT_DIR) //
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
				}
				else
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
				}
				machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);


			}
			break;
			case RUN_SCURVE_T4: // T4
			{
				float cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
				float v0, s;
				machine_set_st.mc_ticks++;
				if (cur_t > SCurveSt.s_curve_T4) //T5
				{
					machine_set_st.mc_ticks = 1;
					cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
					v0 = machine_cal_scurve_v(RUN_SCURVE_T5, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T5, cur_t);
					machine_set_st.mc_state = RUN_SCURVE_T5;
				}
				else
				{
					v0 = machine_cal_scurve_v(RUN_SCURVE_T4, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T4, cur_t);
				}
				if (machine_set_st.mc_motor_dir == LEFT_DIR) //
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
				}
				else
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
				}
				machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);


			}
			break;
			case RUN_SCURVE_T5: // T5
			{
				float cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
				float v0, s;
				machine_set_st.mc_ticks++;
				if (cur_t > SCurveSt.s_curve_T5) //
				{

				}
				else
				{
					v0 = machine_cal_scurve_v(RUN_SCURVE_T5, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T5, cur_t);
					if (machine_set_st.mc_motor_dir == LEFT_DIR) //
					{
						sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
					}
					else
					{
						sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
					}
					machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
				}




			}
			break;
			*/
		/*	case RUN_SCURVE_T6: // T6
			{
				float cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
				float v0, s;
				machine_set_st.mc_ticks++;
				if (cur_t > SCurveSt.s_curve_T6) //T6
				{
					machine_set_st.mc_ticks = 1;
					cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
					v0 = machine_cal_scurve_v(RUN_SCURVE_T7, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T7, cur_t);
					machine_set_st.mc_state = RUN_SCURVE_T7;
				}
				else
				{
					v0 = machine_cal_scurve_v(RUN_SCURVE_T6, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T6, cur_t);
				}
				if (machine_set_st.mc_motor_dir == LEFT_DIR) //
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
				}
				else
				{
					sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
				}
				machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);


			}
			break;
			case RUN_SCURVE_T7: // T7
			{
				float cur_t = machine_set_st.mc_ticks * machine_set_st.mc_per;
				float v0, s;
				if (cur_t > SCurveSt.s_curve_T7) //T7
				{

				}
				else
				{
					v0 = machine_cal_scurve_v(RUN_SCURVE_T7, cur_t); // 1/2*JT^2
					s = machine_cal_scurve_s(RUN_SCURVE_T7, cur_t);
					if (machine_set_st.mc_motor_dir == LEFT_DIR) //
					{
						sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm - s;
					}
					else
					{
						sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm + s;
					}
					machine_set_st.mc_cal_speed = VMM_TO_VRPM(v0);
					machine_set_st.mc_ticks++;
				}


			}
			break;*/
	case RUN_CHANGE_DIR:							 //
		if (machine_set_st.mc_motor_dir == LEFT_DIR) //
		{

			// sync_un_speed_set.sync_set_pos = ENCODER_TO_MM(sync_un_speed_set.sync_cur_set_pos);
			sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm;
#ifndef _WIN32
			mot_run_state.report_type = 6;

			// if(machine_set_st.mc_arr_pos_flag ==1) //

#endif
			//			if(machine_set_st.mc_arr_pos_flag ==1) //
			{
				machine_set_st.mc_arr_pos = machine_set_st.mc_left_limit;
				machine_set_st.mc_change_pos = machine_set_st.mc_right_limit;
				machine_set_st.mc_change_send_flag = 1;

				machine_set_st.mc_state = RUN_SCURVE_T1;
				machine_set_st.mc_motor_dir = RIGHT_DIR;
				machine_set_st.mc_ticks++;
			}
			// else
			{
				//	machine_set_st.mc_ticks ++ ;
				//	machine_set_st.mc_cal_speed =500;
			}

			// machine_set_st.mc_sync_dir = RIGHT_DIR;
		}
		else //
		{
			// sync_un_speed_set.sync_set_pos = ENCODER_TO_MM(sync_un_speed_set.sync_cur_set_pos);
			sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm;

#ifndef _WIN32
			mot_run_state.report_type = 6;

			//	if(machine_set_st.mc_arr_pos_flag ==1) //

#endif
			{
				machine_set_st.mc_arr_pos = machine_set_st.mc_right_limit;
				machine_set_st.mc_change_pos = machine_set_st.mc_left_limit;
				machine_set_st.mc_change_send_flag = 1;

				machine_set_st.mc_state = RUN_SCURVE_T1;
				machine_set_st.mc_motor_dir = LEFT_DIR;
				//	machine_set_st.mc_ticks ++ ;
			}
			// else
			{
				machine_set_st.mc_ticks++;
				// machine_set_st.mc_cal_speed =500 ;
			}

			// sync_un_speed_set.sync_set_pos = ENCODER_TO_MM(sync_un_speed_set.sync_cur_set_pos);

			// machine_set_st.mc_sync_dir = LEFT_DIR;
		}
		break;
	}
#ifndef WIN32
	// #ifndef TEST_SYNC

	if (machine_set_st.mc_cal_speed < 200)
		machine_set_st.mc_cal_speed = 200;
// #endif
#endif // !1

	if (machine_set_st.mc_motor_dir == LEFT_DIR) //
	{

		int32_t pos;
		if (sync_un_speed_set.sync_set_pos < machine_set_st.mc_left_limit_mm)
		{
			sync_un_speed_set.sync_set_pos = machine_set_st.mc_left_limit_mm;
		}
		pos = MM_TO_ENCODER(sync_un_speed_set.sync_set_pos);
		sync_un_speed_set.sync_target_pos = pos;

#ifndef _WIN32

		//		mot_run_state.posReq = sync_un_speed_set.sync_target_pos;
		//		machine_set_st.mc_arr_pos_flag = 0;
		if (speed_mode_flag)
		{
			moveSpd(-(machine_set_st.mc_cal_speed / g_fltM1speedAngularScale), -1);
		}
		else
		{
			movePos(posOut2In(sync_un_speed_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, -machine_set_st.mc_cal_speed * machine_set_st.mc_taget_speed_scale), -1); // 1.05
			// movePos(posOut2In(sync_un_speed_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, VMM_TO_VRPM(pScurveSt->s_curve_v2)), -1);//1.05

			g_sM1Drive.sPosCtrl.fltAdvSpeed = -(machine_set_st.mc_cal_speed / g_fltM1speedAngularScale * machine_set_st.mc_adv_scale);
		}
#endif // WIN32
	}
	else
	{
		int32_t pos;
		if (sync_un_speed_set.sync_set_pos > machine_set_st.mc_right_limit_mm)
		{
			sync_un_speed_set.sync_set_pos = machine_set_st.mc_right_limit_mm;
		}

		pos = MM_TO_ENCODER(sync_un_speed_set.sync_set_pos);
		sync_un_speed_set.sync_target_pos = pos;

#ifndef _WIN32
		//		mot_run_state.posReq = sync_un_speed_set.sync_target_pos;
		//		machine_set_st.mc_arr_pos_flag = 0;
		if (speed_mode_flag)
		{
			moveSpd((machine_set_st.mc_cal_speed / g_fltM1speedAngularScale), -1);
		}
		else
		{
			movePos(posOut2In(sync_un_speed_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, machine_set_st.mc_cal_speed * machine_set_st.mc_taget_speed_scale), -1); // 1.05
			// movePos(posOut2In(sync_un_speed_set.sync_target_pos), spdOut2In(MOT_MSG_MOVE_POS, VMM_TO_VRPM(pScurveSt->s_curve_v2)), -1);//1.05
			g_sM1Drive.sPosCtrl.fltAdvSpeed = (machine_set_st.mc_cal_speed / g_fltM1speedAngularScale * machine_set_st.mc_adv_scale);
		}
#endif // WIN32
	}
}

void machine_set_un_speed_scale(float scale)
{
	machine_main_set_st.mc_un_speed_fScale = scale;
	if (machine_main_set_st.mc_un_speed_fScale > 1.0f)
		machine_main_set_st.mc_un_speed_fScale = 1.0f;
}
void machine_set_acc_speed_scale(float scale, uint16_t type)
{
	if (type == 0)
		machine_main_set_st.mc_left_acc_fScale = scale;
	else
		machine_main_set_st.mc_right_acc_fScale = scale;
	if (machine_main_set_st.mc_acc_speed_fScale > 1.0f)
		machine_main_set_st.mc_acc_speed_fScale = 1.0f;
}
void machine_set_acc_speed_pos(uint16_t pos, uint16_t pos1)
{

	machine_main_set_st.mc_left_acc_speed_pos = pos;
	machine_main_set_st.mc_right_acc_speed_pos = pos1;
	machine_main_set_st.mc_acc_speed_pos = pos;

	machine_main_set_st.mc_var_acc_pos = pos;
	machine_main_set_st.mc_var_dec_pos = pos1;
}
void machine_set_arr_pos_flag(uint8_t flag)
{
	machine_set_st.mc_arr_pos_flag = flag;
}
float machine_get_arr_pos_offset(void)
{
	return machine_set_st.mc_arr_pos_offset;
}
void machine_set_max_acc_fScale(float scale) //
{
	machine_main_set_st.mc_max_acc_fScale = scale;
}
void machine_set_max_dec_fScale(float scale) //
{
	machine_main_set_st.mc_max_dec_fScale = scale;
}
void machine_set_var_acc_fScale(float scale) //
{
	machine_main_set_st.mc_var_acc_fScale = scale;
}
void machine_set_var_dec_fScale(float scale) //
{
	machine_main_set_st.mc_var_acc_fScale = scale;
}
void machine_set_max_acc_dec_pos(uint16_t pos1, uint16_t pos2) //
{
	machine_main_set_st.mc_max_acc_pos = pos1;
	machine_main_set_st.mc_max_dec_pos = pos2;
}

void machine_set_acc(float f_acc) //
{
	machine_main_set_st.mc_acc = f_acc;
}
void machine_set_dec(float f_dec) //
{
	machine_main_set_st.mc_dec = f_dec;
}
void machine_set_acc_dec_flag(uint8_t flag) //
{
	machine_main_set_st.mc_acc_dec_flag = flag;
}
uint8_t machine_get_acc_dec_flag(void)
{
	return machine_set_st.mc_acc_dec_flag;
}
void machine_init(uint16_t per_ms)
{
	machine_set_st.mc_left_limit = 821 * 8;
	machine_set_st.mc_right_limit = 3501 * 8;
	// machine_set_st.mc_run_per = per_ms *3 ;
	machine_set_st.mc_run_per = 0;
	machine_set_st.mc_cur_pos = 0;
	machine_set_st.mc_old_enc = 0;
	machine_set_st.target_speed = 0;
	//  machine_set_st.mc_target_pos = machine_set_st.mc_right_limit ;
	machine_set_st.mc_target_pos = machine_set_st.mc_left_limit;
	machine_set_st.mc_oc = 0;
	machine_set_st.mc_left_scale = 1.0f;
	machine_set_st.mc_right_scale = 1.0f;

	// machine_set_st.mc_adv_scale = 0.65f ;
	// machine_set_st.mc_adv_scale = 0.55f ;
	machine_set_st.mc_adv_scale = 0.2f;
	machine_set_st.mc_taget_speed_scale = 1.2f;
	// machine_set_st.mc_taget_speed_scale =1.1f ;
	//   machine_set_st.mc_taget_speed_scale =2.0f ;
	// machine_set_st.mc_taget_speed_scale =2.0f ;
	// machine_set_st.mc_taget_speed_scale =1.0f ;
	machine_set_st.mc_change_send_flag = 0;
	machine_set_st.mc_per_cnt = machine_set_st.mc_run_per;
	machine_set_st.mc_arr_pos = machine_set_st.mc_left_limit;
	// machine_set_st.mc_arr_dis = 1300 ;
	machine_set_st.mc_arr_dis = 2000;
	machine_set_st.mc_cur_quan = 0;
	machine_set_st.mc_report_quan_flag = 0;
	machine_set_st.mc_set_speed = 0;
	machine_set_st.target_speed = 0;
	machine_set_st.mc_state = 0;
	machine_set_st.mc_right_max = -50000;
	machine_set_st.mc_left_max = 50000;
	machine_set_st.mc_jl_pos = 0;

	machine_set_st.mc_f_left_limit = posOut2In(machine_set_st.mc_left_limit);
	machine_set_st.mc_f_right_limit = posOut2In(machine_set_st.mc_right_limit);
	machine_set_st.mc_left_limit_mm = ENCODER_TO_MM(machine_set_st.mc_left_limit);
	machine_set_st.mc_right_limit_mm = ENCODER_TO_MM(machine_set_st.mc_f_right_limit);

	machine_set_st.mc_bc_limit_pos = 300;
	machine_set_st.mc_acc_speed_pos = 2000;
	machine_set_st.mc_un_speed_scale = 6000;
	machine_set_st.mc_acc_speed_scale = 9033;
	machine_set_st.mc_un_speed_fScale = 0.5;
	machine_set_st.mc_acc_speed_fScale = 0.3;
	machine_set_st.mc_debug_send_flag = 0;
	machine_set_st.mc_stop_pos_p = 7.0f;
	machine_set_st.mc_arr_pos_offset = 200; // 80 200
	machine_set_st.mc_acc_scale = 0.0f;
	machine_set_st.mc_sync_pos_p = 15.0f;
	machine_set_st.mc_per = 0.1f;
	machine_set_st.mc_j_acc_flag = 1;
	machine_set_st.mc_acc_dec_flag = 1;
	machine_main_set_st.mc_acc_dec_flag = 1;
	// machine_set_st.mc_dec_pos_p = 23.0f;
	//	machine_set_st.mc_dec_pos_p = 25.0f;//1: 19 2 :15 4 :25
	//	machine_set_st.mc_dec_pos_p = 19.0f;//1: 19 2 :15 4 :25
	machine_set_st.mc_dec_pos_p = 30.0f; // 1: 19 2 :15 4 :25
	machine_set_st.mc_un_speed_pos_p = 30.0f;
	InitScurve();
	//	 machine_adv_pos_init();
}
void machine_change_mode(void) //
{
	static uint8_t acc_dec_flag = 1;
	machine_set_st.mc_acc_dec_flag = machine_main_set_st.mc_acc_dec_flag;
	if (acc_dec_flag == 1 && machine_set_st.mc_acc_dec_flag == 0) //
	{
		machine_set_st.mc_state = RUN_SCURVE_T1;
		machine_set_st.mc_ticks = 1;

		sync_un_speed_set.sync_cur_ext_pos = ENCODER_TO_MM(sync_get_ext_pos()); //
		sync_un_speed_set.sync_set_pos = sync_un_speed_set.sync_cur_ext_pos;
		sync_un_speed_set.sync_cur_set_pos = sync_get_ext_pos();
		machine_set_st.mc_motor_dir = RIGHT_DIR;
		machine_set_st.mc_sync_dir = RIGHT_DIR;
	}
	else if (acc_dec_flag == 0 && machine_set_st.mc_acc_dec_flag == 1) //
	{
#ifndef _WIN32
		g_sM1Drive.sPosCtrl.fltAdvSpeed = 0;
#endif
		machine_set_st.state = 0;
		machine_set_st.mc_sync_dir = RIGHT_DIR;
		// sync_set.sync_cur_ext_pos = sync_get_ext_pos();//
		sync_set.sync_cur_ext_pos = machine_set_st.mc_left_limit + (machine_set_st.mc_right_limit - machine_set_st.mc_left_limit) / 2; //
		sync_set.sync_set_pos = sync_set.sync_cur_ext_pos;
		sync_set.sync_cur_set_pos = sync_set.sync_cur_ext_pos;
		// g_sM1Drive.sPosCtrl.sPositionPiParams.fltPGain =machine_set_st.mc_stop_pos_p ;
	}
	acc_dec_flag = machine_set_st.mc_acc_dec_flag;
}
void machine_set_scurve_a_max(float f_a_max) // S
{
	// SCurveSt_set_st.s_curve_a_max = f_a_max;
}
void machine_set_scurve_v_max(float f_v_max) // S
{
	// SCurveSt_set_st.s_curve_v_max = f_v_max;
}
void machine_set_scurve_as(uint8_t type, float a)
{
	/*switch (type)
	{
	case 1:
		SCurveSt_set_st.s_curve_a_max = a;
		break;
	case 2:
		SCurveSt_set_st.s_curve_a2 = a;
		break;
	case 3:
		SCurveSt_set_st.s_curve_a3 = a;
		break;
	case 4:
		SCurveSt_set_st.s_curve_a4 = a;
		break;

	default:
		break;
	}*/
}
void machine_set_scurve_ts(uint8_t type, float t) // ST
{
	/*switch (type)
	{
	case 1:
		SCurveSt_set_st.s_curve_T1 = t;
		break;
	case 2:
		SCurveSt_set_st.s_curve_T2 = t;
		break;
	case 3:
		SCurveSt_set_st.s_curve_T3 = t;
		break;
	case 4:
		SCurveSt_set_st.s_curve_T4 = t;
		break;
	case 5:
		SCurveSt_set_st.s_curve_T5 = t;
		break;
	case 6:
		SCurveSt_set_st.s_curve_T6 = t;
		break;
	case 7:
		SCurveSt_set_st.s_curve_T7 = t;
		break;
	default:
		break;
	}*/
}
void machine_set_scurve_t(float t) // S
{
	// SCurveSt_set_st.s_curve_T = t;
}
void machine_set_scurve_jj(float jj) // S
{
	// SCurveSt_set_st.s_curve_jj = jj;
}
void machine_set_j_scale(float scale)
{
	machine_main_set_st.mc_j_scale = scale;
}
void machine_reset_run_para(void)
{
#ifndef TEST_SYNC
/*	memset((void*)&SCurve_ltr_St, 0, sizeof(SCURVEST));
	memset((void*)&SCurve_rtl_St, 0, sizeof(SCURVEST));
	memset((void*)&SCurveSt_set_st, 0, sizeof(SCURVEST));*/
#endif
}
