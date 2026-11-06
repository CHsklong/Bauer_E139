/*
 * Copyright (c) 2021 HPMicro
 *
 */
#include "mcu.h"
#include "mcdrv.h"
#include "freemaster.h"
#include "servo_can.h"
#include "param.h"
#include "motor.h"

#include "machine.h"
#include "freemaster_usb.h"
#include "gpio.h"
#include "work_io.h"
#include "user_app_fun.h"
//#include "hq_can_encrypt.h"

/*
void hq_can_encrypt_test(void)
{
    uint8_t data[8]={0x71 ,0x01,0x00,0x00,0x00,0x00,0x00,0x00} ;
    uint8_t en_data[8]={0} ;
    can_encrypt_data(1,data,en_data) ;
    can_decrypt_data(1,en_data,data) ;
    en_data[0] = 0 ;
}*/
int main(void)
{
  //  static uint32_t CSR_reg;
    board_init();
    motor_clock_hz = InitClock();
    MCDRV_Init_gpio() ;
    initMotor();
    initParameter() ;
    MCDRV_Init_M1();
    work_input_init() ;
    can_configure_init(mot_run_state.can_baund);
    #if(BOARD_TYPE == BOARD_E884)
    can1_configure_init((uint8_t)mot_run_state.can_baund2);  
    #endif
    BOARD_USB_Serial_Init();
    FMSTR_ExampleUsbInit();
    FMSTR_Init();
    board_delay_ms(200) ;
    motSysParamApply();
    initMotorPos() ;  
    user_app_fun_init() ;
    board_timer_create(1, board_ms_loop);
    #if(BOARD_TYPE == BOARD_E884)
    servo_ext_pos_ctrl_init() ;
   
    #endif
    while (1)
    {
        user_app_fun_loop() ;
        FMSTR_ExampleUsbLoop();
        FMSTR_Poll();
        easyCanProc();
        easyCanProc1();
        motRunSM(MOT_MSG_NONE, NULL);
        MainTask1ms();
    }
    return 0;
}

