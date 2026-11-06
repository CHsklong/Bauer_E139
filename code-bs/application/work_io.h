#ifndef _VWORK_IO_H_
#define _VWORK_IO_H_

#include "stdint.h"
#define IO_STAT_INVAILD 0xff // IO
#define IO_ALARM_VALID 1	 //

#define IO_FUN_ZERO 1	  // 零位功能
#define IO_FUN_SAFE 2	  // 安全位功能
#define IO_FUN_PUNCHING 3 // 冲孔信号
typedef enum
{
	/* 0   */ IN_INPUT1 = 0,
	/* 1   */ IN_INPUT2 = 1,
	/* 1   */ IN_INPUT3 = 2,
	/* 2   */ INPUT_IDX_MAX = 3,
} input_idx_t; // IO

typedef enum
{
	/* 0   */
	OUTPUT_IDX1 = 0,
	OUTPUT_IDX_MAX = 1,
} output_idx_t; // IO

typedef struct
{
	int32_t in1_fun : 4;	  // 输入1功能码
	int32_t in2_fun : 4;	  // 输入2功能码
	int32_t in3_fun : 4;	  // 输入3功能码
	int32_t res_in_fun : 4;	  // 输入预留
	int32_t out1_fun : 4;	  // 输出功能码
	int32_t res_out_fun : 12; // 输出预留
} IO_FUN_ST;
typedef union
{
	IO_FUN_ST io_fun_st; // 输入输出结构体
	int32_t io_fun;		 // 输入输出功能
} IO_Fun_Tag;

typedef enum
{
	PIN_OFF = 0,
	PIN_ON = 1,
} PIN_STAT;

typedef struct
{
	input_idx_t idx;   //
	uint8_t use;	   // 0 1 2
	uint8_t type;	   // IO=0   IO =1
	uint8_t def_state; //
	uint8_t filter;	   //
	uint8_t alm_edge;  //
} IN_t;

typedef struct
{
	output_idx_t idx;	//
	uint8_t def_state;	//
	PIN_STAT out_state; //
} OUT_t;

typedef enum
{
	EDGE_NORMAL = 0, //
	EDGE_RISING,	 //
	EDGE_FALLING,	 //
} IN_EDGE_t;		 //

#define INPUT_BUFF_BYTES (INPUT_IDX_MAX / 8 + 1) //

// IN_t input_info[INPUT_IDX_MAX];

uint8_t work_input_chk_port(uint8_t idx); /*  */

uint8_t work_input_get_state(input_idx_t idx); /*  */

IN_EDGE_t work_input_get_edge(input_idx_t idx); /*  */

void work_input_init(void);

void work_input_scan(void); /*  */

void work_output(output_idx_t idx, PIN_STAT stat);

uint8_t work_input_get_stable(input_idx_t idx); /*  */

void work_io_set_fun(int32_t io_fun);

IO_Fun_Tag work_io_get_fun(void);
void work_input_set_def_state(input_idx_t idx, uint8_t def_state);

#endif // !_VWORK_INPUT_H_
