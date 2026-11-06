#ifndef _JACKING_MOTOR_H_
#define _JACKING_MOTOR_H_
#include "stdint.h"
#include "alert.h"
/************ ************************/

#define  DEBUG_CMD_PRINT 1

typedef uint8_t (*check_zero_signal)(void);
typedef uint8_t (*check_safe_signal)(void);

typedef enum //
{
	JM_CTRL_NULL = 0,
	JM_CTRL_RESET = 1,
	JM_CTRL_MOVE_TO_DOSSAL = 2,
	JM_CTRL_MOVE_DOSSAL_TO_SAFE = 3,
	JM_CTRL_PULL_DOWN_CLOTH = 4,
	JM_CTRL_MOVE_TO_SAFE = 5,
	JM_CTRL_MOVE_TO_CAST_OFF = 6,
	JM_CTRL_PAUSE = 7,
	JM_CTRL_RECOVER = 8,
	JM_CTRL_EXIT = 9,
	JM_CTRL_SET_PARA = 10,
	JM_CTRL_MOVE_TO_TARGET_POS = 11,
} JM_CTRL_TYPE; // 主控制类型

typedef enum
{
	JM_SUB_CTRL_STOP = 1,
	JM_SUB_CTRL_HANDLE_UP = 2,
	JM_SUB_CTRL_HANDLE_DOWN = 3,
	JM_SUB_SAVE_DOSSAL_POS = 4,
	JM_SUB_SAVE_CAST_OFF_POS = 5,
	JM_SUB_SAVE_SAFE_DOOR_POS = 6,
} JM_SUB_CTRL_TYPE; // 子控制类型

typedef enum
{
	HD_STOP = 0,
	HD_DIR_UP = 1,
	HD_DIR_DOWN = 2,
} HANDLE_CTRL_DIR; // 手动控制方向

typedef enum
{
	JM_SAFE_STATE_INIT = 0,
	JM_SAFE_STATE_TRANS_MOVE_SAFE = 1,
	JM_SAFE_STATE_MOVE_SAFE = 2,
	JM_SAFE_STATE_TRANS_MOVE_DOSSAL = 3,
	JM_SAFE_STATE_MOVE_DOSSAL = 4,
	JM_SAFE_STATE_ERROR_AGAIN = 5,
	JM_SAFE_STATE_READY_FINISH = 6,
	JM_SAFE_STATE_PAUSE = 10,
	JM_SAFE_STATE_EXIT = 11,
} JM_SAFE_STATE; // 安全位状态

typedef enum
{
	PARA_SET_SAFE_DOOR = 1,	 //
	PARA_SET_CAST_OFF = 2,	 //
	PARA_SET_DOSSAL_POS = 3, //

} PARA_SET_TYPE; // 参数类型

typedef enum
{
	JM_DOSSAL_STATE_INIT = 0,
	JM_DOSSAL_STATE_TRANS_MOVE_DOWN = 1,
	JM_DOSSAL_STATE_MOVE_DOWN = 2,
	JM_DOSSAL_STATE_TRANS_HIGH_MOVE_UP = 3,
	JM_DOSSAL_STATE_HIGH_MOVE_UP = 4,
	JM_DOSSAL_STATE_TRANS_LOW_MOVE_UP = 5,
	JM_DOSSAL_STATE_LOW_MOVE_UP = 6,
	JM_DOSSAL_STATE_RECORD_SAFE_POS = 7,
	JM_DOSSAL_STATE_TRANS_SAFE_TO_DOSSAL = 8,
	JM_DOSSAL_STATE_SAFE_TO_DOSSAL = 9,
	JM_DOSSAL_STATE_PAUSE = 10,
	JM_DOSSAL_STATE_EXIT = 11,
} JM_DOSSAL_STATE; // 最高位状态机

typedef enum
{
	JM_STATE_INIT = 0,
	JM_STATE_TRANS_RESET = 1,
	JM_STATE_RESET = 2,
	JM_STATE_PARA_SET = 3,
	JM_STATE_MOVE_DOSSAL_TO_SAFE = 4,
	JM_STATE_MOVE_DOSSAL = 5,
	JM_STATE_PULL_DOWN_CLOTH = 6,
	JM_STATE_MOVE_TO_SAFE = 7,
	JM_STATE_MOVE_TO_CAST_OFF = 8,
	JM_STATE_HANDLE_UP = 9,
	JM_STATE_HANDLE_DOWN = 10,
	JM_STATE_MOVE_TO_TARGET_POS = 11,
} JM_STATE; // 状态机

typedef enum
{
	JM_MAIN_STATUS_EXIT = 0,
	JM_MAIN_STATUS_RESET = 1,
	JM_MAIN_STATUS_DOSSAL = 2,
	JM_MAIN_STATUS_MOVE_DOSSAL_TO_SAFE = 3,
	JM_MAIN_STATUS_PDC = 4,
	JM_MAIN_STATUS_MOVE_TO_SAFE = 5,
	JM_MAIN_STATUS_MOVE_TO_CAST_OFF = 6,
	JM_MAIN_STATUS_PAUSE = 7,
	JM_MAIN_STATUS_PARA_SET = 8,
	JM_MAIN_STATUS_HANDLE_UP = 9,
	JM_MAIN_STATUS_HANDLE_DOWN = 10,
	JM_MAIN_STATUS_FINISH = 11,
	JM_MAIN_STATUS_ERROR = 12,
} JM_MAIN_STATUS; // 主状态

typedef enum
{
	JM_PDC_STATE_INIT = 0,
	JM_PDC_STATE_TRANS_TORQUE = 1,
	JM_PDC_STATE_TORQUE = 2,
	JM_PDC_STATE_BLANCE_TORQUE_BIG = 3,
	JM_PDC_STATE_SAFE_POS_TO_PULL_DOWN = 4,
	JM_PDC_STATE_PAUSE = 10,
	JM_PDC_STATE_EXIT = 11,
} JM_PULL_DOWN_CLOTH_STATE; // 挂布位状态机
typedef enum
{
	JM_RESET_INIT = 0,
	JM_RESET_TRANS_MOVE_DOWN = 1,
	JM_RESET_MOVE_DOWN = 2,
	JM_RESET_TRANS_MOVE_UP = 3,
	JM_RESET_MOVE_UP = 4,
	JM_RESET_RUN = 5,
	JM_RESET_PAUSE = 6,
	JM_RESET_EXIT = 7,
	JM_RESET_TRANS_MOVE_UP_FROM_STOP = 8,
	JM_RESET_MOVE_UP_FROM_STOP = 9,
} JM_RESET_STATE; // 复位状态机
typedef enum
{
	JM_MT_POS_STATE_TRANS_MOVE = 0,
	JM_MT_POS_STATE_MOVE = 1,
	JM_MT_POS_STATE_PAUSE = 6,
	JM_MT_POS_STATE_EXIT = 7,
} JM_MOVE_TO_POS_STATE; // 移动到任意位置的状态机

typedef struct
{
	uint16_t reset_down_speed;			   // 复位下降速度
	int32_t reset_limit_pos;			   // 复位最大行程
	int32_t reset_signal_max_pos;		   // 传感器最大行程
	uint8_t reset_state;				   // 复位状态
	uint8_t reset_pause_state;			   // 暂停之前的状态
	int32_t reset_begin_pos;			   // 复位开始的位置
	int32_t reset_signal_enable_begin_pos; // 传感器刚感应到的位置
	int32_t reset_zero_pos;				   // 零位初始位置

} JACKING_MOTOR_RESET_ST;

typedef struct
{
	uint8_t state;					  // 状态机
	uint8_t pause_state;			  // 暂停时状态
	int32_t pos;					  // 目标位置
	uint8_t jm_mt_pos_stae_move_flag; // 运动到指定位置，data5=1,表示回退，超过安全位就停止 JM_MT_POS_STATE_MOVE_FLAG
} MOVE_TO_POS_ST;					  // 移动位置

typedef struct
{
	int32_t dossal_clib_pos;				// 经过安全位校准后的拉布位置
	uint16_t dossal_high_speed;				// 最高位高速
	uint16_t dossal_low_speed;				// 最高位位低速
	uint16_t dossal_check_signal_speed;		// 检测传感器速度
	int32_t dossal_check_signal_begin_pos;	// 检测传感器开始位置
	uint32_t dossal_check_signal_limit_pos; // 检测传感器容差值
	int32_t dossal_safe_dec_pos;			// 安全位减速位置
	uint8_t dossal_state;					// 最高位状态
	uint8_t dossal_pause_state;				// 最高位暂停状态
	uint32_t dossal_time_out_max;			// 超时时间ms
	uint32_t dossal_begin_time;				// 安全位到最高位开始时间
} DOSSAL_ST;								//

typedef struct
{
	uint16_t safe_max_offset_pos;	// 安全位最大容差
	uint16_t safe_signal_check_pos; // 安全位传感器最大行程
	uint8_t safe_state;				// 安全位状态机
	int16_t safe_offset_pos;		// 安全位偏执
	uint8_t safe_pause_state;		// 安全位暂停前的状态
	uint16_t safe_torque;			// 挂布到安全位力矩
	uint16_t safe_speed;			// 安全位速度
	uint16_t safe_move_offset_pos;	// 安全位多移动的距离
} SAFE_SIGNAL_ST;					//

typedef struct
{
	int16_t pdc_torque;					   // 有效力矩
	uint8_t pdc_torque_change_flag;		   // 有效力矩更新
	int32_t pdc_begin_pos;				   // 挂布开始位置
	int32_t pdc_begin_pos_limit;		   // 拉布时每行的位置，作为拉力位置限制
	uint8_t pdc_state;					   // 挂布状态
	uint8_t pdc_pause_state;			   // 挂布暂停时的状态
	uint32_t pdc_change_torque_begin_time; // 开始切换的时间
	int16_t pdc_big_torque;				   // 大扭矩
	uint16_t pdc_torque_time;			   // 有效力矩时间
	uint16_t pdc_big_torque_time;		   // 大力矩有效时间
	uint8_t pdc_change_torque_enable;	   // 大小力矩切换的使能
	uint8_t pdc_main_ctrl_flag;			   // 主控力矩控制标记
	int16_t pdc_pull_max_pos;			   // 拉布时最大行程限制

} PULL_DOWN_CLOTH_ST;

typedef struct
{
	check_zero_signal pCkZeroSignal;	  // 零位信号检测函数
	check_safe_signal pCkSafeSignal;	  // 安全位信号检测函数
	JM_CTRL_TYPE ctrl_type;				  // 控制类型
	JM_SUB_CTRL_TYPE sub_ctrl_type;		  // 子控制类型
	HANDLE_CTRL_DIR handle_ctrl_dir;	  // 手动控制方向
	uint16_t move_speed_torque;			  // 速度模式下的力矩
	uint8_t ctrl_main_status;			  // 主状态
	uint8_t ctrl_sub_status;			  // 子状态
	uint8_t zero_signal;				  // 零位信号
	uint8_t safe_signal;				  // 安全位信号
	uint8_t exit_flag;					  // 退出标记
	uint8_t pause_flag;					  // 1 0 暂停标记
	uint8_t set_para_type;				  // 设置参数类型
	uint8_t set_para_finish_flag;		  // 设置参数完成标记
	uint16_t debug_speed;				  // 未设置参数时的速度
	uint8_t state;						  // 升降
	int32_t zero_pos;					  // 零位位置
	int32_t safe_door_pos;				  // 安全门位置
	int32_t cast_off_pos;				  // 脱圈位位置
	int32_t dossal_pos;					  // 最高位位置
	int32_t safe_signal_pos;			  // 安全位位置
	int16_t motor_target_speed;			  // 目标速度
	uint16_t signal_enable_speed;		  // 检测传感器行程的速度
	uint16_t zero_to_safe_door_speed;	  // 零位到安全门速度
	uint16_t safe_door_to_cast_off_speed; // 安全门到脱圈位速度
	uint16_t cast_off_to_safe_speed;	  // 脱圈位到安全位速度
	uint16_t safe_to_dossal_speed;		  // 安全位到最高位速度
	int32_t move_target_pos;			  // 移动到任意位置
	uint8_t report_dis_pos;				  // 上报的间隔距离
	uint16_t report_cur_ptr;			  // 当前按间隔算出来的位置
	uint16_t handle_speed;				  //
	uint16_t safe_move_offset_pos;		  // 安全位多移动的距离
	uint16_t safe_move_offset_flafpos;	  // 没有用
	DOSSAL_ST dossal_st;				  // 挂布
	SAFE_SIGNAL_ST safe_signal_st;		  // 安全位
	PULL_DOWN_CLOTH_ST pdc_st;			  // 拉布
	MOVE_TO_POS_ST move_to_cass_off_st;	  // 移动到脱圈位
	MOVE_TO_POS_ST move_to_safe_st;		  // 除最高位其他位置移动到安全位
	MOVE_TO_POS_ST move_to_target_pos_st; // 移动到任意位置
	uint8_t pre_safe_signal;			  // 安全位信号的前一次信号
        int16_t safe_to_dossal_torque ;      //安全位到最高位的力矩
} JACKING_MOTOR_INIT_ST;

void jacking_motor_init(void);														 // 初始化
void jacking_motor_task_loop(void);													 // 升降电机主任务
void jacking_motor_ctrl(JM_CTRL_TYPE type, JM_SUB_CTRL_TYPE sub_type, int16_t data); // 升降电机控制命令
uint8_t jacking_motor_get_main_stat(void);											 // 获取升降电机的主状态
void jacking_motor_report_current_from_pos(uint8_t dis_pos);						 // 根据位置上报电流
void jacking_motor_report_status(void);												 // 升降电机上报状态
#endif
