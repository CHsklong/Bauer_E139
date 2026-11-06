
#include "user_app_fun.h"
#include "motor_sync.h"
#include "jacking_motor.h"
#include "mcdrv.h"
#include "eep_p24c32c.h"
#include "wire_feed_motor.h"
#include "motor_round_run_ctrl.h"
void user_app_fun_init(void) // 功能初始化
{
#if (BOARD_TYPE == BOARD_E139)
#ifdef JACKING_MOTOR_FUN // 升降功能打开
	jacking_motor_init();
#endif
#elif (BOARD_TYPE == BOARD_E914)
#ifdef SYNC_MOTOR_FUN
	sync_follow_power();
#endif
#elif (BOARD_TYPE == BOARD_E145)
#ifdef WIRE_MOTOR_FUN // 穿丝功能打开
    
	P24CLXX_Init();
	wire_feed_motor_init();
	WFMotorParameterInit();
        if (BOARD_E145_GET_VER() != 1) // 硬件版本V2.0
        {
           ReadWfMotorGroupParamFromEpp(0);
        }

        wire_feed_motor_set_broken_check_type();
#endif
#ifdef JACKING_MOTOR_FUN // 升降功能打开
	jacking_motor_init();
#endif

      
#endif

#ifdef ROUND_RUN_FUN
	round_run_ctrl_init() ;
#endif 

}
void user_app_fun_loop(void) // 主循环调用
{
#if (BOARD_TYPE == BOARD_E139)
#ifdef JACKING_MOTOR_FUN
	jacking_motor_task_loop();
#endif
#elif (BOARD_TYPE == BOARD_E145)
#ifdef JACKING_MOTOR_FUN //升降功能打开
        jacking_motor_task_loop();
#endif
#ifdef WIRE_MOTOR_FUN //穿丝功能打开        
	wire_feed_motor_task_loop();
#endif
#endif

#ifdef ROUND_RUN_FUN
	round_run_ctrl_loop() ;
#endif
}
void user_app_isr_ms(void ) //1ms中断
{
    
}