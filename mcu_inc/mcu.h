
#ifndef MCU_H
#define MCU_H

#define SEGGER_RTT_ENABLE 0

#define     HIGH    1
#define     LOW     0

#define  APP_VERH		5
#define  APP_VERL       5

#define __NDS32_EXT_PERF__  0
#include "UserTypeDef.h"

#include "board.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "hpm_debug_console.h"
#include "hpm_sysctl_drv.h"
#include "hpm_gpio_drv.h"
#include "pinmux.h"
#include "hpm_pwm_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_qei_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_adc16_drv.h"
#include "hpm_mcan_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_acmp_drv.h"
#include "hpm_spi_drv.h"
#include "hpm_math.h"
#include "hpm_iomux.h"
#include "hpm_opamp_drv.h"
// mcu_configure
#ifdef CONFIG_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif

#include "hpm_dmamux_drv.h"
#include "hpm_l1c_drv.h"
#include "hpm_dac_drv.h"
#include "hpm_ppor_drv.h"

/*   */
#include "adc.h"
#include "can.h"
#include "pwm.h"
#include "encode.h"
#include "gpio.h"
#include "opamp_acmp.h"
#include "dac.h"
#include "abs_motor.h"
#include "abs_tor.h"
#include "spi.h"
#ifndef BOOT_DEF
#include "mt6835.h"
#endif
#include "eeprom.h"

#ifndef PI
#define  PI             3.14159265358979f
#endif

#ifndef FALSE
#define  FALSE               (0)
#endif

#ifndef TRUE
#define  TRUE                (1)
#endif 

#define  sizeof_16(c)           (sizeof(c)>>1)

#define  memcpy_16(a,b,c)       memcpy(a,b,c*2)
#define  memset_16(a,b,c)       memset(a,b,c*2)

#if SEGGER_RTT_ENABLE
#include "SEGGER_RTT.h"
#endif



extern uint16_t  timercnt ;
extern uint32_t  cntread ;

extern uint8_t read_u;
extern uint8_t read_v;
extern uint8_t read_w;

extern uint8_t     flash_save_enable ;
extern uint8_t flash_save_ok;
extern uint16_t  outVolt ;
extern uint8_t     mag_cal_start;
extern uint16_t    mag_cal_cnt;


#endif
