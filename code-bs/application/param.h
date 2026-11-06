#ifndef _PARAM_H_
#define _PARAM_H_
#include "stdint.h"
#include "mcdrv.h"
#include "user_app_fun.h"


#define PARNO_MOT_ID (0)
#define PARNO_RUN_DIR (1)
#define PARNO_ZERO_DIR (2)
#define PARNO_ZERO_STOP (3)
#define PARNO_ZERO_POS (4)
#define PARNO_NUME (5)
#define PARNO_DENO (6)
#define PARNO_LEFT_LMT (7)
#define PARNO_RIGHT_LMT (8)
#define PARNO_MAX_SYNC_OFF (9)
#define PARNO_MOTOR_TYPE (10)
#define PARNO_DIR_PROTOCOL (11)
#define PARNO_ENC_DIR (12)
#define PARNO_SYNC_OFF_RATIO (13)
#define PARNO_CAN_BAUND (14)

#define PARNO_POS_BUF_CLIB_SCALE (63)
#ifndef _BOOT
#if (BOARD_TYPE == BOARD_E139 ||BOARD_TYPE == BOARD_E884)
#define PARNO_MAX (64)
#define JACKING_MOTOR_BEING  (100)
#define JACKING_MOTOR_PARA_MAX (25)
#elif (BOARD_TYPE == BOARD_E914)
#define PARNO_MAX (59)
#elif (BOARD_TYPE == BOARD_E145)
#define PARNO_MAX           (56)
#ifdef JACKING_MOTOR_FUN //升降功能
#define JACKING_MOTOR_BEING  (100)
#define JACKING_MOTOR_PARA_MAX (25)
#endif
#ifdef WIRE_MOTOR_FUN //穿丝功能 
#define WF_MOTOR_PARA_MAX (30)
#endif

#endif
#else
#define PARNO_MAX (15)
#endif
#define PARANO_MOTOR_BEGIN (15)

#define over_load_groups (5)

enum
{
  DIR_PROTOCOL_ZHISHAN = 1,
  DIR_PROTOCOL_HQ
};
typedef union Param_Data_Tag
{
  float f4_data;
  int32_t i4_data;
  uint32_t u4_data;
  int16_t i2_data;
  uint16_t u2_data;
  int8_t i1_data;
  uint8_t u1_data;
} Param_Data;

void initParameter(void);
void writeParameter(uint16_t paraNo, Param_Data data);
void writeAlignData(void);
Param_Data *readParameter(uint16_t paraNo);

#if (BOARD_TYPE == BOARD_E139)
void writeJackingMotorParameter(uint16_t paraNo, Param_Data data);
uint32_t readJackingMotorParameter(uint16_t paraNo);
#elif (BOARD_TYPE == BOARD_E914)

#elif (BOARD_TYPE == BOARD_E145)
#define WF_PARARM_MAX (10)
typedef struct
{
	uint16_t param_head_flag ; //参数标记
	uint8_t  param_version ; //参数版本号
	uint8_t  param_groups ; //参数种类
	uint8_t  param_cur_group ;//当前参数号
	uint8_t  prarm_cur_idx ; //当前索引号
	uint8_t  param_group[WF_PARARM_MAX] ; //
  uint8_t  factory_group[WF_PARARM_MAX] ; //
}WIRE_PARAM_HEAD ;
void writeWfMotorParameter(uint16_t paraNo, Param_Data data);
uint32_t readWfMotorParameter(uint16_t paraNo);
void SaveWfMotorGroupParamToEpp(uint8_t group);
uint8_t GetWfMotorFactoryGroup(uint8_t idx);
void ReadWfMotorGroupParamFromEpp(uint8_t group);
void DeleteAllWfMotorGroup(void);
void AddWfMotorGroup(uint8_t gourp_id, uint8_t factory_id);
uint8_t GetWfMotorGroup(uint8_t idx);
void WFMotorParameterInit(void);
void SaveWfMotorCurGroup(uint8_t group);
uint8_t GetWfMotorCurGroup(void);
#endif

#endif
