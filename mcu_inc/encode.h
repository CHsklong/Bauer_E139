#ifndef ENCODE_H
#define ENCODE_H

#include "mcu.h"

#define  HARD_EQP_CNT    qei_get_current_count(HPM_QEI0,qei_counter_type_phase)&0xfffff;
int qei_init(void);

extern uint32_t  encode_cnt;

//ATTR_PLACE_AT(".ahb_sram")  extern u
extern	int16_t        ENC_QEPCNT_REG ;


int qei_init(void);
uint32_t qei_cnt(void) ;

#endif