#ifndef _MACHINE_H_
#define _MACHINE_H_
#include <stdint.h>

#define MACHINE_MAX_SPEED 2500 // 

#define MAX_JR_MOTOR_MAX_ENCODER 16383
#define MACHINE_MIN_SPEED 50
#define MACHINE_TRACK_POS 50 
#define MACHINE_STOP_DIS_POS 30 //50
//#define DEBUG_PRINT 1
#define MACHINE_INIT_TIME  10000 //6s 
#define MOTOR_CYCLE_POS 137.0f 
//#define S_CURVE 1
enum MACHINE_STATE { MACHINE_CHECK_POS = 0, MACHINE_INIT, MACHINE_IDLE, MACHINE_STOP, MACHINE_SETUP_RUN, MACHINE_RUN };
enum SYNC_TYPE { SYNC_NONE = 0, SYNC_SPEED = 1, SYNC_POS = 2 };
enum { RIGHT_DIR = 0, LEFT_DIR = 1 };
//enum { RUN_INIT = 0, RUN_ACC = 1, RUN_DEC, RUN_UN_SPEED, RUN_CHANGE_DIR };
enum {RUN_SCURVE_INIT =0 ,RUN_SCURVE_T1, RUN_SCURVE_T2, RUN_SCURVE_T3, RUN_SCURVE_T4, RUN_SCURVE_T5, RUN_SCURVE_T6, RUN_SCURVE_T7, RUN_CHANGE_DIR};
#ifdef _WIN32
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	double err_last;
	double err;
	double integral;
	float sync_set_pos;
	float sync_pos;
	float sync_error_pos;
	int sync_acc_speed;
	int sync_max_acc_speed;
	int sync_set_speed;
	int sync_base_speed;
	int follow_cnt;
	int sync_adv_pos;
	int sync_adv_max_pos;
	uint8_t  sync_debug_idx[2];
	float adv_speed_scale;
	int sync_end_follow_speed;
	int sync_start_l;
	float sync_start_dis;
	uint8_t  sync_torque_flag;
	int sync_jt_start_time;
	float sync_adv_dis_scale;
	int sync_start_speed;
	int sync_start_acc;
	int sync_acc_pos;
	int sync_acc_time;
	uint8_t  sync_first_acc;
	float sync_stop_pos;
	uint8_t  alarm_enable;
	float sync_end_follow_spd_scale; //结束跟随速度系数
	float sync_start_pos;
	float sync_lead; //提前量
	float sync_spd_ratio; //
	uint8_t  sync_enable;
	float sync_cur_ext_pos;
	uint32_t abs_spd;
	uint8_t set_stop_pos_first_flag;
	float sync_target_pos;
	float sync_cur_set_pos; //卷绕电机转化出来的位置
	float sync_un_speed_pos;
}SYNC_SET;
#endif

typedef struct
{
	float s_curve_jj; //加加速度 mm/ms^3
	float s_curve_T1; //s曲线T1 ms
	float s_curve_T2; //s曲线T2 ms
	float s_curve_T3; //s曲线T3 ms
	float s_curve_T4; //s曲线T4 ms
	float s_curve_T5; //s曲线T5 ms
	float s_curve_T6; //s曲线T6 ms
	float s_curve_T7; //s曲线T7 ms
	float s_curve_T; //s曲线T总 ms
	float s_curve_a_max; //s曲线最大加速度 mm/ms^2
	float s_curve_v_max; //s曲线最大速度 mm/ms 
	float s_curve_a2;
	float s_curve_a3;
	float s_curve_a4;
	float s_curve_s; //s曲线总距离 mm 
	float s_curve_s1;//s曲线S1距离 mm
	float s_curve_s2;//s曲线S2距离 mm
	float s_curve_s3;//s曲线S3距离 mm
	float s_curve_s4;//s曲线S4距离 mm
	float s_curve_s5;//s曲线S5距离 mm
	float s_curve_s6;//s曲线S6距离 mm
	float s_curve_s7;//s曲线S7距离 mm
	float s_curve_v1;
	float s_curve_v2;
	float s_curve_v3;
	float s_curve_v4;
	float s_curve_v5;
	float s_curve_v6;
	float s_curve_v7;
}SCURVEST;
typedef struct
{
	float  mc_cur_pos; //当前位置
	int32_t  mc_left_limit; //左限位
	int32_t  mc_right_limit;// 右限位
	float  	 mc_target_pos; //目标位置
	uint16_t mc_run_per; //运行周期 ms
	uint8_t  mc_state; //状态
	int32_t  mc_track_pos; //PID反馈误差
	uint16_t mc_init_speed_time; //开始速度维持时间
	uint32_t mc_init_ticks; //mc初始上电时的计数值
	uint16_t mc_old_enc;
	int32_t  target_speed;
	uint16_t mc_enc;
	uint8_t  mc_oc; //联动开关 
	float    mc_left_scale; //左比例系数 
	float    mc_right_scale; //右比例系数 
	int32_t  mc_set_speed; //主控设置速度
	uint16_t mc_cal_speed; //根据卷绕比设置出来的速度 
	float    mc_adv_scale; //速度前馈比例
	uint8_t  mc_change_send_flag; //换向发送标志
	uint16_t mc_per_cnt; //周期计时
	int32_t  mc_arr_dis; //提前到位换向
	int32_t  mc_arr_pos; //到位的位置 
	uint32_t mc_set_report_quan; //设置上报的圈数
	uint32_t mc_cur_quan; //当前圈
	uint8_t  mc_report_quan_flag;
	int32_t  mc_change_pos;//换向位置
	float    mc_push_pos;//卷绕电机的位置
	float    mc_pre_push_pos; //前一次卷绕电机位置
	uint8_t  mc_sync_dir;
	uint8_t state;
	uint16_t mc_left_set_value1;
	uint16_t mc_left_set_value2;
	uint16_t mc_right_set_value1;
	uint16_t mc_right_set_value2;
	int32_t  mc_right_max;
	int32_t  mc_left_max;
	float    mc_taget_speed_scale;
	float   mc_max_push_pos;
	float   mc_target_scale;
	uint32_t mc_jl_pos; //卷绕电机位置
	float   mc_f_left_limit;
	float   mc_f_right_limit;
	//float   mc_target_pos_scale ;
	int32_t  mc_bc_limit_pos;
	float    mc_cur_speed;
	uint16_t mc_un_speed_scale; //匀速段的卷绕比 
	int32_t  mc_acc_speed_pos; //加速段间距 
	uint16_t mc_acc_speed_scale; //加速段卷绕比
	uint16_t mc_init_speed_scale; //初始段卷绕比
	float    mc_init_speed_fScale; //初始段恒动和卷绕比
	float    mc_acc_speed_fScale; //加速段恒动和卷绕电机比例
	uint8_t  mc_send_flag;
	int32_t  mc_jr_motor_encoder; //卷绕电机的编码值
	int32_t  mc_jr_acc_encoder;
	float    target_pos;
	uint8_t  mc_debug_send_flag;
	uint16_t mc_debug_value;
	uint16_t mc_debug_value1;
	uint16_t mc_debug_canid;
	float  	 mc_left_acc_fScale;
	float    mc_right_acc_fScale;
	int32_t  mc_left_acc_speed_pos;
	int32_t  mc_right_acc_speed_pos;
	int32_t	 mc_left_l2r_acc_speed_pos; //可变加速段左边 左到右间距 
	int32_t  mc_left_r2l_acc_speed_pos; //可变加速段左边 右到左间距
	int32_t  mc_right_l2r_acc_speed_pos; //可变加速段右边 左到右间距
	int32_t  mc_right_r2l_acc_speed_pos; //可变加速段右边 右到左间距
	float    mc_left_l2r_acc_speed_fScale; //加速段左边左到右
	float    mc_left_r2l_acc_speed_fScale; //加速段左边右到左
	float    mc_right_l2r_acc_speed_fScale; //加速段右边左到右
	float    mc_right_r2l_acc_speed_fScale;//加速度右边 右到左
	float    mc_left_l2r_fScale; //左到右总的卷绕比
	float    mc_left_r2l_fScale; //右到左总的卷绕比
	float    mc_right_l2r_fScale;
	float    mc_right_r2l_fScale;
	float    mc_left_fmax;
	float    mc_right_fmax;
	float    mc_stop_pos_p;
	uint8_t  mc_arr_pos_flag;
	float    mc_arr_pos_offset;
	float    mc_acc_scale; //恒动加速系数
	float    mc_sync_pos_p;
	float    mc_max_acc_fScale; //最大加速段传动比
	float    mc_max_dec_fScale; //最大减速段传动比
	float    mc_var_acc_fScale; //可变加速段的传动比 
	float    mc_var_dec_fScale; //可变减速段的传动比
	float    mc_un_speed_fScale; //匀速段恒动和卷绕电机比例
	uint16_t  mc_var_acc_pos; //可变加速段距离
	uint16_t  mc_var_dec_pos; //可变减速段距离
	uint16_t  mc_max_acc_pos; //最大加速段距离
	uint16_t  mc_max_dec_pos; //最大减速度距离
	uint16_t  mc_un_speed_pos; //匀速段的距离
	float     mc_acc; //加速段加速度 mm/ms^2
	float     mc_dec; //减速度减速度 mm/ms^2
	float     mc_per; //运行周期 
	uint8_t   mc_j_acc_flag; //卷绕电机加减速阶段
	float     mc_ticks;
	float     mc_acc_t;
	float     mc_dec_t;
	float     mc_t;
	float     mc_un_speed_t;
	float     mc_v0;
	float     mc_left_limit_mm;
	float     mc_right_limit_mm;
	uint8_t   mc_acc_dec_flag ; //卷绕电机加减速标记
	uint8_t   mc_motor_dir ;
	float     mc_un_speed_acc ;
	float     mc_cur_part_v0 ; //当前段的初始速度
	float     mc_cur_part_s ; //当前端的初始位置
	float     mc_dec_pos_p ; 
	float     mc_un_speed_pos_p ;
	uint8_t   mc_cw_flag ;
	float     mc_j_scale ; 
	float     mc_cw_pre_left_limit_mm;
	float     mc_cw_pre_right_limit_mm;

}MACHINE_SET_ST;

typedef union
{
	float f1;
	int32_t n1;
}UNFLOAT;
void machine_reset(void);
void machine_init(uint16_t per_ms);
void machine_run(void);
void machine_set_switch(uint8_t sw);
void machine_set_left_limit(int32_t pos);
void machine_set_right_limit(int32_t pos);
void machine_set_target_spd(int32_t spd);
void machine_set_oc(uint8_t oc);
void machine_set_scale(uint8_t idx, float mot_value);
uint8_t machine_get_change_flag(void);
void machine_set_change_flag(uint8_t flag);
int32_t machine_get_arr_pos(void);
uint32_t machine_get_cur_quan(void);
void machine_set_report_quan(uint32_t quan);
void machine_set_cur_quan(uint32_t quan);
void machine_sync_deal(int push_pos); //

void machine_sync_test(void); //push_pos 周期内的增量
void machine_set_un_speed_scale(float scale);
void machine_set_acc_speed_scale(float scale, uint16_t type);
void machine_set_acc_speed_pos(uint16_t pos, uint16_t pos1);
void reportDebug(uint16_t value, uint16_t value1, uint16_t canid);
uint16_t machine_get_jr_encode(void);
uint8_t machine_get_debug_flag(void);
void machine_set_debug_flag(uint8_t flag);
uint16_t machine_get_debug_value(void);
uint16_t machine_get_debug_value1(void);
uint16_t machine_get_debug_canid(void);
void machine_set_arr_pos_flag(uint8_t flag);
float machine_get_arr_pos_offset(void);
uint8_t machine_get_oc(void);
void machine_set_max_acc_fScale(float scale);//设置最大加速传动比
void machine_set_max_dec_fScale(float scale);//设置最大减速传动比
void machine_set_var_acc_fScale(float scale); //设置可变加速段的传动比
void machine_set_var_dec_fScale(float scale); //设置可变减速度的传动比
void machine_set_max_acc_dec_pos(uint16_t pos1, uint16_t pos2);//设置最大加速段加速和减速距离
void machine_main_paras_use(uint8_t type); // 0表示左限位生效 1表示右限位生效 2表示卷绕比生效
void machine_set_acc(float f_acc); //设置加速度
void machine_set_dec(float f_dec); //设置减速度
uint8_t machine_get_acc_dec_flag(void) ;
void machine_acc_dec_sync_deal(int push_pos); //卷绕电机加减速时处理同步方式

void machine_set_total_time(float t); //设置总时间
void machine_set_acc_dec_flag(uint8_t flag); //设置加减速标记
void machine_change_mode(void); //卷绕电机加减速和匀速 横动电机控制模式切换
void machine_set_scurve_a_max(float f_a_max); //设置S曲线的最大加速度 
void machine_set_scurve_v_max(float f_v_max); //设置S曲线的最大速度 
void machine_set_scurve_ts(uint8_t type, float t); //设置S曲线的所有段T
void machine_set_scurve_t(float t); //设置S曲线的总时间
void machine_set_scurve_jj(float jj); //设置S曲线的加加速度
void machine_set_scurve_as(uint8_t type, float a); //设置S曲线各段加速度
//void machine_cal_scurve_para(float s);//计算S曲线的其余参数
void machine_reset_run_para(void); //重置运动参数
void machine_set_cw_flag(uint8_t flag); //璁剧疆宸綅鏍囪
void machine_set_j_scale(float scale);
void InitScurve(void);
#endif
