/*
 * Copyright (c) 2021 HPMicro
 *
 */

#include "param.h"
#include "tick.h"
#include "hal_for_upg.h"
#include "upgrade.h"
#include "gpio.h"
#include "can.h"
#include "motor.h"
extern void board_ms_loop(void) ;
extern Mot_Run_State mot_run_state; 
ATTR_RAMFUNC int main(void)
{
    board_init();

    led_gpio_init();
   // SYS_LED_HIGH();
    initParameter() ;
    hal_flash_init();
    can_gpio_init();
    can_configure_init(mot_run_state.can_baund) ;

    
    board_timer_create(1, board_ms_loop);

    boot_main() ;
    while (1)
    {

    }
    return 0 ;
}

