#include "stdint.h"
#define SYNC_POS_INIT  999999999
typedef enum {
CW,
CCW,
}MOT_DIR;

typedef struct
{
	float Kp ;
	float Ki ;
	float Kd ;
	double err_last ;
	double err ;
	double integral ;
	float sync_set_pos ;
	float sync_pos ;
	float sync_error_pos ;
	int sync_acc_speed; 
	int sync_max_acc_speed; 
	int sync_set_speed;
	int sync_base_speed; 
	int follow_cnt ;
	int sync_adv_pos ;
	int sync_adv_max_pos;
	uint8_t  sync_debug_idx[2] ;
	float adv_speed_scale ;
	int sync_end_follow_speed ;
	int sync_start_l ;
	float sync_start_dis;
	uint8_t  sync_torque_flag ;
	int sync_jt_start_time ;
	float sync_adv_dis_scale ;
	int sync_start_speed ;
	int sync_start_acc ;
	int sync_acc_pos ;
	int sync_acc_time ;
	uint8_t  sync_first_acc ; 
	float sync_stop_pos ;
	uint8_t  alarm_enable ; 
	float sync_end_follow_spd_scale ; //���������ٶ�ϵ��
	float sync_start_pos ;
    float sync_lead ; //��ǰ��
	float sync_spd_ratio ; //
	uint8_t  sync_enable ;
	float sync_cur_ext_pos ;
	uint32_t abs_spd;
	uint8_t set_stop_pos_first_flag ;
	float sync_target_pos ;
	float sync_cur_set_pos ; //���Ƶ��ת��������λ��
	float sync_un_speed_pos;
}SYNC_SET ;
enum PID_TYPE{PID_P,PID_I,PID_D,END_FOLLOW_SPD_SCALE} ;
extern SYNC_SET sync_set ;
void sync_set_follow_dwviation(int32_t val);
int32_t sync_get_follow_dwviation(void);
int sync_get_shazui_pos(void) ;
int sync_get_shazui_error_pos(void);
void sync_set_follow_error_limit(int value) ; 
void sync_set_start_acc_scale(int value) ;
void sync_set_gap_scale(int value) ;
void sync_set_pid(uint8_t type,float value) ;
int pid_acc_speed_interp(void);
void sync_deal_follow(void);
void sync_set_debug_idx(uint8_t type ,uint8_t value);
void sync_lock(void) ;
void sync_un_lock(void) ;
void sync_end_follow(int pos) ;
void sync_follow_power(void);
void sync_set_stop_pos(int pos);
void sync_set_enable(uint8_t enable) ;
void sync_start_follow(int16_t offset,int start_pos,char dir);
void sync_end_pos(int end_pos);
void sync_deal_can(int push_pos);
void sync_follow_init(void);
float sync_get_ext_pos(void);




