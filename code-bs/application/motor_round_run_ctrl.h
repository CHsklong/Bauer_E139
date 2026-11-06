#ifndef _MOTOR_ROUND_RUN_CTRL_H_
#define _MOTOR_ROUND_RUN_CTRL_H_
/*电机往返运动控制 
1、设置针运行左限位、右限位、左零位、右零位、总针数、信号提前时间（0.1ms）。
2、功能函数
（1）功能：提前工作。参数：目标针数、提前时间（0.1Ms）
（2）功能：时间预估。参数：目标针数 返回：预估时间（0.1Ms）
（3）功能：编码器设置
（4）功能：编码器读取  返回：编码器值，针位置
（5）功能：工作运行。参数：运行目标针位置、运行最高速度、摇床启动位置、摇床			结束位置、摇床运行时间
（6）功能：暂停继续。参数：启停、停止模式（0急停 1正常减速停 2发电停）、加速模式（0慢速一行 1匀加速）、运行最高速度
（7）功能：设置电机运行参数。参数：加加速，减减速
（8）功能：启停针中断。 
*/                     



#include "mcdrv.h"

enum ROUND_RUN_DIR {LEFT_RUN_DIR=1,RIGHT_RUN_DIR=2} ;
enum ROUND_RUN_STATE {RR_INIT =0 ,RR_TRANS_LEFT_RUN,RR_TRANS_RIGHT_RUN, RR_LEFT_RUNNING,RR_RIGHT_RUNNING ,RR_PAUSE,RR_EXIT} ;
enum ROUND_RUN_STOP_MODE {ROUND_RUN_SLOW_STOP=0,ROUND_RUN_EMERGENCY_STOP=1,ROUND_RUN_POWER_STOP=2} ;
enum ROUND_RUN_RECOVER_STARTUP_MODE {RR_STARTUP_SLOW_ONE=0,RR_STARTUP_UN_ACC} ; //0:慢速一行 1:匀加速
enum ROUND_RUN_REPORT_ACK {RR_REPORT_STOP_ACK =1 ,RR_REPORT_ARR_ACK =2,RR_REPOT_OFFSET_POS_ACK=3} ; //停止到位 位置偏移 ACK
enum ROUND_RUN_MAIN_CTRL {RR_CTRL_BEGIN =1,RR_CTRL_PAUSE =2,RR_CTRL_RECOVER=3,RR_CTRL_EXIT=4} ;
typedef struct 
{
	int16_t set_speed:12 ; //目标速度
	int16_t reserve:2 ; //预留
	int16_t set_dir:1 ; //目标方向
	int16_t stop_flag:1 ; //暂停标记 当前行暂停
}SET_CTRL_BIT_ST;

typedef union 
{
	int16_t set_ctrl_data;
	SET_CTRL_BIT_ST set_ctrl_bit_st ;
}SET_CTRL_ST;

typedef struct 
{
	int32_t set_pos ; //目标位置 
	SET_CTRL_ST set_ctrl_st ; //设置控制相关参数
}ROUND_RUN_SET_ST;


typedef struct
{
	int16_t cur_speed :12 ; //当前速度
	int16_t report_ack :4 ; //回复ACK
}REPORT_INFO_BIT_ST ;

typedef union 
{
	int16_t report_data ; 
	REPORT_INFO_BIT_ST report_bit_st ;
}REPORT_INFO_ST;

typedef struct 
{
	int32_t cur_pos ;//当前位置 
	REPORT_INFO_ST report_info_st ; 
}ROUND_RUN_REPORT_ST;



typedef struct
{
	int16_t pause :1 ; //暂停标记 1暂停 0连续
	int16_t change_dir_flag :2 ; //换向标记 1左换向 2右换向 
	int16_t reserve :13; // 预留
}CHANGE_DIR_BIT_ST ;

typedef union 
{
	int16_t change_dir_data ; 
	CHANGE_DIR_BIT_ST change_dir_bit_st ;
}CHANGE_DIR_ST;

typedef struct 
{
	int32_t cur_pos ;//当前位置 
	CHANGE_DIR_ST change_dir_st ; 
}ROUND_RUN_CHANGE_DIR_ST;


typedef struct 
{
	uint8_t state ; //当前状态
	uint8_t run_dir ; //运行方向
	uint8_t ctrl_flag ; //启动标记
	uint8_t pause_flag ; //暂停标记 
	uint8_t pause_state ; //暂停前状态
	uint8_t stop_mode ; //停止模式 
	uint8_t recover_flag ; //恢复标记
	uint8_t recover_mode ; //恢复模式
	uint8_t exit_flag ; //退出标记
	int32_t left_pos ; //左换向位置
	int32_t right_pos ;//右换向位置 
	uint16_t left_spd ; //左换向速度
	uint16_t right_spd ; //右换向速度
	uint16_t slow_spd ; //慢速一行的速度 
	uint8_t  right_to_left_flag ; //右到左运行标记
	uint8_t  left_to_right_flag ; //左到右运行标记
	uint8_t  right_to_left_pause_flag ; //右到左暂停标记 
	uint8_t  left_to_right_pause_flag ; //左到右暂停标记 
	uint16_t max_v ; //最大速度
	int32_t arr_pos ; //到位偏移量
	int32_t cur_pos ; //当前位置
	int32_t report_offpos ; //偏移位置上报 大于 0就上报
	int32_t report_cur_pos ; //上报时的当前位置 与下次上报做比较
	uint8_t report_stop_req_flag ; //停止请求上报标记
}ROUND_RUN_CTRL_ST;


void round_run_ctrl_init(void) ; //初始化
void round_run_ctrl_loop(void) ; //周期性调用
void round_run_ctrl_pause(uint8_t stop_mode) ; //暂停 
void round_run_ctrl_set_run_target(int32_t target_pos , int16_t ctrl_data) ; //设置控制数据
void round_run_ctrl_set_report_offpos(int32_t offpos) ; //设置上报的偏移位置
void round_run_ctrl_report(int32_t cur_pos ,int16_t ctrl_ack_data16,uint8_t change_dir_event) ;
void round_run_ctrl_main_set(uint8_t main_ctrl,uint8_t sub_ctrl) ;
void round_run_ctrl_set_slow_speed(uint16_t speed)  ;
#endif
