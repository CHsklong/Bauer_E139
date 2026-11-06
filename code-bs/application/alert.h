#ifndef _ALERT_H_
#define _ALERT_H_
#include "mcdrv.h"
#include "user_app_fun.h"
#define MAX_CHAR 35
typedef struct
{
	uint8_t alt_no ;
	char  str[MAX_CHAR] ;
}ALT_STR;

/***********
bit0-3 :报警级别（0表示报警 1表示提示）
bit4:报警后是否需要复位 1表示是 0表示否
************/
#define ALT_RESET_FLAG 0X10  // (1<<4)

#define ALT_NONE                    (0)
#define ALT_MOT_OVER_CURRENT        (0x11)  //over current
#define ALT_MOT_ROTOR_BLOCKED       (0x12)
#define ALT_MOT_OVER_LOAD           (0x15)
#define ALT_MOT_OVER_DCBUS          (0x16)
#define ALT_MOT_OVER_SPEED          (0x20)
#define ALT_MOT_UNDER_DCBUS         (0x24)
#define ALT_SYNC_ERROR              (0x22) // 
#define ALT_LEFT_LIMIT              (7)    //over left limit position
#define ALT_RIGHT_LIMIT             (8)   //over right limit position
#define ALT_ZERO_TIMEOUT            (9)  //time out to find zero position 
#define ALT_NOT_FIND_SIZE           (10) //time out to find side
#define  ALT_MOT_NO_BIND            (15)

#define ALT_MAX                     (62)

#define ALT_MAX_STR_NUM   (25)

#ifdef JACKING_MOTOR_FUN //升降功能打开
#define	JM_MSG_NO_FIND_ZERO_SIGNAL (51) //
#define JM_MSG_BELT_BROKEN (52) //
#define	JM_MSG_NO_FIND_SAFE_SIGNAL (53) //
#define	JM_MSG_SAFE_SIGNAL_OFFSET_ERROR (54) //
#define	JM_MSG_SAFE_SIGNAL_OR_POS_ERROR (55)  //
#define	JM_MSG_MOVE_SAFE_TORQUE_SMALL (56)   //
#define	JM_MSG_BALANCE_TOO_BIG (57)  //
#define	JM_MSG_TORQUE_OVER_SPEED (58) //
#define	JM_MSG_SIGANL_ERROR (59) //
#define	JM_MSG_DOSSAL_SET_ERROR (60) //
#define JM_MSG_BLOCK_OR_DOSSAL_SET_EEROR (61)
#define	JM_MSG_SIGANL_SAFE_ERROR (62) //    原来的59里的第2条安全位传感器一直亮
#define	JM_MSG_PULL_SAFE_ERROR (63) //    拉布安全位位置限制报警
/**************  *************/


#endif 

#ifdef WIRE_MOTOR_FUN //穿丝功能打开 
#define	JM_MSG_ZERO_ERROR (51) //
#define JM_MSG_FEED_BLOCK (52) //
#define	JM_MSG_WIRE_BROKEN_OR_NO_CLAMP (53) //
#define	JM_MSG_WIRE_BROKEN (54)  //
#define	JM_MSG_WIRE_TOO_TIGHT (55)   //
#define	JM_MSG_ZERO_SET_ERR_OR_SIGNAL_ERR (56)   //
#define	JM_MSG_NO_SET_END_POS (57)  //
#define	JM_MSG_WIRE_FEED_LIMIT (58) //
#define	JM_MSG_WIRE_COLLECT_LIMIT (59) //
#define	JM_MSG_SET_ZERO_TIME_OUT (60) //
#define JM_MSG_SET_ZERO_ERROR (61)
#define JM_MSG_NO_FIND_END_SIGNAL (62)
#define	JM_MSG_ZERO_NO_ERROR (63) //  零位传感器错误63
#define	JM_MSG_WIRE_BROKEN_NO_ERROR (64) //  请检查是否断丝和钢丝压轮没有压紧
/**************  *************/


#endif


void reportState(uint16_t canid,uint8_t motor_id,uint8_t can_type);
void reportStart(uint16_t repTime);
void reportPos(int32_t pos);
void reportTask(void);
void reportMotorCurrent(int16_t pos ,int16_t current) ;
int alertProc(uint8_t alertNo);
int alertPush(uint8_t alertNo,uint8_t rank);
uint8_t alertPop(void);
uint8_t alertGet(void);
void alertClear(void);
void alert_send(uint8_t alert_id,int32_t pos);
void reportAlarmStr(uint8_t mot_id,uint8_t str_type,uint8_t alart_no) ;
void reportJkMotorStatus(uint32_t status) ;
void reportCmd(uint8_t cmd) ;
void reportDebug(uint16_t data1,uint16_t data2,uint16_t data3) ;
void servo_send_over_zero(uint16_t value);
#endif
