#ifndef _WIRE_FEED__MOTOR_H_
#define _WIRE_FEED__MOTOR_H_
#include "stdint.h"
#include "alert.h"
/************ ************************/
typedef uint8_t (*check_wire_zero_signal)(void);
typedef uint8_t (*check_end_signal)(void);
typedef uint8_t (*check_broken_signal)(void);

#define DEBUG_CMD_PRINT 1
#define SIGNAL_NOMAL_OPEN 0	 // 常开
#define SIGNAL_NOMAL_CLOSE 1 // 常闭
#define SIGNAL_NO_USE 2		 // 不使用
typedef enum
{
	WF_RESET_INIT = 0,
	WF_RESET_TRANS_FEED_WIRE = 1,
	WF_RESET_FEED_WIRE = 2,
	WF_RESET_TRANS_COLLECT_WIRE = 3,
	WF_RESET_COLLECT_WIRE = 4,
	WF_RESET_RUN = 5,
	WF_RESET_PAUSE = 6,
	WF_RESET_EXIT = 7,
	WF_RESET_TRANS_ZERO = 8, // 从零位传感器亮位置移动到设置零位位置
} WF_RESET_STATE;			 // 复位状态机

typedef enum
{
	WF_STOP = 0,			// 停止
	WF_RUN_FEED_DIR = 1,	// 送丝方向
	WF_RUN_COLLECT_DIR = 2, // 收丝方向
} WF_RUN_DIR;
typedef enum
{
	WF_WORK_INIT = 0,
	WF_WORK_TRANS_FEED_WIRE = 1,
	WF_WORK_FEED_WIRE = 2,
	WF_WORK_TRANS_COLLECT_WIRE = 3,
	WF_WORK_COLLECT_WIRE = 4,
	WF_WORK_TRANS_BACK_WIRE = 5,
	WF_WORK_BACK_WIRE = 6,
	WF_WORK_PAUSE = 7,
	WF_WORK_EXIT = 8,
} WF_WORK_STATE; // 工作状态机

typedef enum
{
	WF_SETTING_INIT = 0,
	WF_SETTING_TRANS_MOVE_HALF_CIRCLE = 1,
	WF_SETTING_MOVE_HALF_CIRCLE = 2,
	WF_SETTING_TRANS_SAVE_ZERO = 3,
	WF_SETTING_SAVE_ZERO = 4,
	WF_SETTING_SAVE_END_POS = 5,
	WF_SETTING_MOVE_END_POS = 6,
	WF_SETTING_MOVE_UP = 7,
	WF_SETTING_MOVE_DOWN = 8,
	WF_SETTING_STOP = 9,
	WF_SETTING_TRANS_MOVE_ZERO_POS = 10,
	WF_SETTING_MOVE_ZERO_POS = 11,
	WF_SETTING_MOVE_ZERO_SIGNAL = 12,
	WF_SETTING_TRANS_IMPACK_ZERO = 13,
	WF_SETTING_IMPACK_ZERO = 14,
	WF_SETTING_PAUSE = 15,
	WF_SETTING_EXIT = 16,
} WF_SETTING_STATE; // 工作状态机

typedef enum //
{
	WF_CTRL_NULL = 0,
	WF_CTRL_RESET = 1,
	WF_CTRL_CHECK = 2,
	WF_CTRL_WORK = 3,
	WF_CTRL_PAUSE = 4,
	WF_CTRL_RECOVER = 5,
	WF_CTRL_EXIT = 6,
	WF_CTRL_SET_PARA = 7,
} WF_CTRL_TYPE; // 主控制类型

typedef enum
{
	WF_SUB_CTRL_NULL = 0,
	WF_SUB_CTRL_STOP = 1,		 // 停止
	WF_SUB_CTRL_HANDLE_UP = 2,	 // 送丝
	WF_SUB_CTRL_HANDLE_DOWN = 3, // 收丝
	WF_SUB_SAVE_ZERO = 4,		 // 保存零位
	WF_SUB_MOVE_HALF_CIRCLE = 5, // 移动半圈
	WF_SUB_MOVE_ZERO_POS = 6,	 // 移动到零位位置
	WF_SUB_SAVE_END_POS = 7,	 // 记录末端位置
	WF_SUB_MOVE_ZERO_SIGNAL = 8, // 移动到传感器零位
	WF_SUB_IMPACK_ZERO = 9,		 // 撞零方式设置零位
} WF_SUB_CTRL_TYPE;				 // 子控制类型

typedef enum
{
	WF_STATE_INIT = 0,
	WF_STATE_TRANS_RESET = 1,
	WF_STATE_RESET = 2,
	WF_STATE_PARA_SET = 3,
	WF_STATE_TRANS_CHECK = 4,
	WF_STATE_CHECK = 5,
	WF_STATE_TRANS_WORK = 6,
	WF_STATE_WORK = 7,
	WF_STATE_HANDLE_UP = 8,
	WF_STATE_HANDLE_DOWN = 9,

	WF_MAIN_STATUS_FINISH = 11,
	WF_MAIN_STATUS_ERROR = 12,
} WF_STATE; // 状态机

typedef enum
{
	WF_CHECK_INIT = 0,
	WF_CHECK_TRANS_FEED_WIRE = 1,
	WF_CHECK_FEED_WIRE = 2,
	WF_CHECK_RESET = 3,
	WF_CHECK_PAUSE = 4,
	WF_CHECK_EXIT = 5,
} WF_CHECK_STATE; // 检测状态机

typedef struct
{
	uint16_t wire_zero_single_encoder; // 钢丝零位时 单圈绝对编码值
	int32_t zero_to_half_circle_pos;   // 零位送丝半圈的实际位置

	uint8_t setting_state;		 // 设置状态机
	uint8_t setting_pause_state; // 暂停之前的状态

} WF_SETTING_ST; // 送丝设置流程

typedef struct
{
	uint8_t reset_state;				   // 复位状态机
	int32_t reset_limit_pos;			   // 复位最大行程
	int32_t reset_signal_max_pos;		   // 传感器最大行程
	uint8_t reset_pause_state;			   // 暂停之前的状态
	int32_t reset_begin_pos;			   // 复位开始的位置
	int32_t reset_signal_enable_begin_pos; // 传感器刚感应到的位置
	int32_t reset_zero_pos;				   // 零位初始位置
} WF_RESET_ST;							   // 复位流程

typedef struct
{
	uint8_t check_state;  // 检测状态机
	uint16_t check_speed; // 检测时的速度
	// int32_t  cur_pos ;    //当前位置
	uint8_t check_pause_state; // 暂停之前的状态
} WF_CHECK_ST;				   // 工作流程

typedef struct
{
	uint8_t work_state;					// 工作状态机
	int32_t work_target_pos;			// 目标位置
	int32_t work_cur_pos;				// 当前位置
	int32_t work_cur_back_pos;			// 当前位置
	int32_t work_back_pos;				// 送丝堵转时回退位置
	uint8_t work_feed_block_retry_num;	// 送丝堵转重试次数
	uint8_t work_pause_state;			// 暂停之前的状态
	uint16_t work_feed_ref_torque;		// 送丝参考电流
	uint16_t work_collect_ref_torque;	// 收丝参考电流
	uint16_t work_feed_block_torque;	// 送丝堵转电流
	uint16_t work_collect_block_torque; // 收丝堵转电流
} WF_WORK_ST;							// 工作流程

typedef struct
{
	check_wire_zero_signal pCkZeroSignal; // 零位信号检测函数
	check_end_signal pCkEndSignal;		  // 到位信号检测函数
	check_broken_signal pCkBrokenSignal;  // 堵转检测函数
	uint16_t move_speed;				  // 钢丝移动速度 RPM
	uint16_t feed_speed;				  // 送丝移动速度 RPM
	uint16_t collection_speed;			  // 收丝移动速度 PRM
	int32_t zero_pos;					  // 送丝零位位置
	int32_t end_pos;					  // 送丝末端位置
	uint16_t move_speed_torque;			  // 速度模式下的力矩
	uint8_t ctrl_main_status;			  // 主状态
	uint8_t ctrl_sub_status;			  // 子状态
	uint16_t signal_enable_speed;		  // 检测传感器行程的速度
	uint8_t exit_flag;					  // 退出标记
	uint8_t pause_flag;					  // 1 0 暂停标记
	uint8_t state;						  // 状态
	uint8_t zero_signal;				  // 零位信号
	uint8_t end_signal;					  // 末端信号
	uint8_t broken_signal;				  // 堵转信号
	uint16_t cur_current;				  // 现有电流
	uint8_t move_to_end_pos_flag;		  // 送丝到末端标记
	int32_t feed_dec_pos;				  // 送丝减速点
	int32_t end_allowance_pos;			  // 末端允许超出的偏差
	uint16_t feed_dec_speed;			  // 送丝减速时的速度
	uint8_t run_dir;					  // 电机运行方向
	uint16_t zero_signal_allow_pos;		  // 零位传感器允许容差值
	uint32_t wire_broken_check_pos;		  // 断丝检测的编码值
	uint8_t wire_broken_check_type;		  // 断丝检测类型

	uint16_t wire_end_back_pos;			 // 最后一针时的回退位置
	uint16_t wire_signal_def_status;	 // 初始状态0表示常开 1表示常闭
	int32_t wire_signal_save_zero_jt;	 // 保存第一次传感器亮的位置;杰泰使用
	int16_t wire_signal_save_zero_no_jt; // 保存第一次传感器亮的位置;杰泰使用

	uint16_t wire_broken_retry_factory_type; // 断丝回退厂家类型

	uint16_t wire_motor_signal_set_type; // 两个电机设置传感器方式不同，表示电机类型
	int16_t wire_signal_dec_save_zero;	 // 零位减速点和第一次设置零位赋值给该变量
	uint8_t wire_use_param_no;			 // 当前使用的参数组
	uint8_t wire_params;				 // 总的参数组
	uint8_t report_dis_pos;
	uint16_t report_cur_ptr;
	int16_t handle_speed;
	uint16_t zero_to_half_flag;
	int32_t edge_change_pos;
	int32_t reserve[8];			// 预留参数
	uint8_t end_signal_stat;	// 到位传感器的设置状态
	uint8_t zero_signal_stat;	// 零位传感器的设置状态
	uint8_t broken_signal_stat; // 堵丝传感器的设置状态
        int32_t feed_zero_dec_pos ; //零位减速点
	// WF_RESET_ST wf_reset_st ; //复位流程
	WF_WORK_ST wf_work_st;			// 工作流程
	WF_CHECK_ST wf_check_st;		// 检测流程
	WF_CTRL_TYPE ctrl_type;			// 控制类型
	WF_SUB_CTRL_TYPE sub_ctrl_type; // 子控制类型
	WF_SETTING_ST setting_st;		// 设置流程
} WF_INIT_ST;						// 送丝初始化

void wire_feed_motor_init(void);													  // 初始化
void wire_feed_motor_task_loop(void);												  // 送丝电机主任务
void wire_feed_motor_ctrl(WF_CTRL_TYPE type, WF_SUB_CTRL_TYPE sub_type, int16_t pos); // 升降电机控制命令
void wire_feed_motor_set_target_pos(uint32_t pos);									  // 设置升降电机目标位置
void wire_feed_motor_report_status(void);											  // 送丝电机上报状态
void wire_feed_motor_report_current_from_pos(uint8_t dis_pos);						  // 根据位置上报电流
void wire_feed_motor_set_cur_pos(int32_t pos);										  // 设置送丝电机当前位置
void wire_feed_motor_set_broken_check_type(void);									  // 设置感应方式
#endif
