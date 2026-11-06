#ifndef _USER_APP_FUN_H_
#define _USER_APP_FUN_H_

#include "stdint.h"

//#define SYNC_MOTOR_FUN 1 //自跑电机功能 
//#define JACKING_MOTOR_FUN 2 //升降电机功能
//#define WIRE_MOTOR_FUN 3 //穿丝电机功能

//#define ROUND_RUN_FUN 4 //往返运行功能

void user_app_fun_init(void) ;//应用初始化
void user_app_fun_loop(void) ; //主循环调用
void user_app_isr_ms(void ); //1ms中断
#endif // !_VWORK_INPUT_H_


