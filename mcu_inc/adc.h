#ifndef ADC_H
#define ADC_H
#include "hpm_common.h"
#include "hpm_pwm_regs.h"
#include "hpm_trgm_regs.h"
#include "mcdrv.h"
//-------------

#if (BOARD_TYPE == BOARD_E139||BOARD_TYPE == BOARD_E884)
#define VDC_ADC_CHANEL 1 // ADC0/1
#define REF_ADC_CHANEL 6 // ADC0/1
#define U_ADC_CHANEL 5   // ADC1
#define V_ADC_CHANEL 4   // ADC0
#elif (BOARD_TYPE == BOARD_E914)
#define VDC_ADC_CHANEL 11 // ADC0/11
#define U_ADC_CHANEL 1    // ADC1 -in1
#define V_ADC_CHANEL 2    // ADC0 - in2
#elif (BOARD_TYPE == BOARD_E145)
#define VDC_ADC_CHANEL 7 // ADC0/1
#define REF_ADC_CHANEL 6 // ADC0/1
#define U_ADC_CHANEL 1   // ADC1
#define V_ADC_CHANEL 2   // ADC0
#endif

//------------
#define BOARD_ADC_DMA_BUFFER_LEN 1024

#define BOARD_ADC_MODULE ADCX_MODULE_ADC16

#define ADC_LENS_STEP1 4
#define ADC_LENS 6

#define U_ADC0_INA8 8 // 11 //10 //
#define V_ADC1_INA5 5

#define REF_ADC2_INA3 3
#define VDC_ADC2_INA5 5

#define ZL_POS_ADC0_INA14 14
#define ZL_POS_ADC1_INA10 10
#define ZL_POS_ADC2_INA6 6

#define ZL_CUR_ADC0_INA15 15
#define ZL_CUR_ADC1_INA11 11
#define ZL_CUR_ADC2_INA7 7

#define ADC_IU_RES (adc0_sample[0] + adc0_sample[1] + adc0_sample[2] + adc0_sample[3]) >> 2
#define ADC_IV_RES (adc1_sample[0] + adc1_sample[1] + adc1_sample[2] + adc1_sample[3]) >> 2
#define ADC_VBUS_RES (adc0_sample[4] + adc0_sample[5]) >> 1

#define ZL_POS_ENC_RES (adc0_sample[4] + adc0_sample[5]) >> 1
#define ZL_CUR_PWM_RES (adc1_sample[4] + adc1_sample[5]) >> 1

extern __IO uint8_t seq_full_complete_flag;
extern __IO uint8_t Adc1_Trigger_Times;
// extern ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc_buff[16];

// extern uint32_t adc_buff[16];
extern uint32_t adc_data[1024 * 10];

extern uint32_t adc0_sample[16];
extern uint32_t adc1_sample[16];
extern uint32_t adc2_sample[16];
extern uint32_t adc0_buff[16];
extern uint32_t adc1_buff[16];
extern uint32_t adc2_buff[16];
extern int16_t zL_pos_adc_val;
extern int16_t zL_cur_pwm_val;

extern float AngOffSet_zl_pos;
extern float AngOffSet_zl_cur;

extern float zl_cur_f;
extern float zl_pos_f;

int8_t hw_ADC0_squ_Init(void);
int8_t hw_ADC1_squ_Init(void);
int8_t hw_ADC2_squ_Init(void);

void init_trigger(PWM_Type *ptr);
void init_trigger_mux_squ(TRGM_Type *ptr, uint8_t TRGOCFG_OUT);

// void acmp_config(void);

#endif