/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "SysMain.h"

int8_t  acm_enable = 0;
extern int8_t  over_current_flag ;

//Aug 14th,2023----- acmp_1
void isr_acmp(void)
{
    // acmp_output_toggle = true;
    //  pwm_clear_fault(HPM_PWM0);

    disable_all_pwm_output();
    over_current_flag = 1;
    //    gpio_toggle_pin(HPM_GPIO0 ,GPIO_DO_GPIOA, 19);
//     gpio_toggle_pin(HPM_GPIO0 ,GPIO_DO_GPIOA, 18);
    if (acm_enable)
    {
        //release cpu from interrupt
        acmp_channel_enable_irq(HPM_ACMP, ACMP_CHANNEL_CHN1, ACMP_EVENT_FALLING_EDGE, false);
        //intc_m_disable_irq(IRQn_ACMP_1);
        over_current_flag = 1;

    } else
    {
       // over_current_flag = 0;
    }
    acmp_channel_clear_status(HPM_ACMP, ACMP_CHANNEL_CHN0, ACMP_EVENT_RISING_EDGE | ACMP_EVENT_FALLING_EDGE);
    pwm_clear_status(HPM_PWM1, PWM_IRQ_FAULT);

}
SDK_DECLARE_EXT_ISR_M(IRQn_ACMP_0, isr_acmp)

//void isr_acmp(void)
//{
//    acmp_channel_enable_irq(HPM_ACMP, 0, ACMP_EVENT_RISING_EDGE, false);
//    intc_m_disable_irq(IRQn_ACMP_0);
//}
//SDK_DECLARE_EXT_ISR_M(IRQn_ACMP_0, isr_acmp)


void init_opamp(void)
{

    opamp_cfg_t opamp_cfg;

    opamp_cfg.mode = mode_non_invert_extern_vol;//mode_follow;//
 //   opamp_cfg.mode =mode_follow ; //mode_follow
   // opamp_cfg.negative_input_pin = inm_pad_floating;
    opamp_cfg.positive_input_pin = inp_pad_dac;
    opamp_cfg.negative_input_pin = inm_pad_dac;
  //  opamp_cfg.negative_input_pin = inm_pad_floating;//inp_pad_vip0;
   // opamp_cfg.positive_input_pin = inp_pad_vip2;//inp_pad_vip0;
    opamp_cfg.miller_cap = miller_cap_x_7;
   // opamp_cfg.miller_cap = miller_cap_x_18 ;
    opamp_cfg.enable_phase_margin_cap = true;
 //   opamp_cfg.gain = gain_x_16;//gain_x_16
    opamp_cfg.gain = gain_x_2;//
    opamp_init(HPM_OPAMP0, &opamp_cfg);
    opamp_enable(HPM_OPAMP0);

}


void init_acmp(ACMP_Type * ptr)
{
    acmp_channel_config_t acmp_channel_configure;

    //board_init();
#if defined(ACMP_SOC_BANDGAP) && ACMP_SOC_BANDGAP
    acmp_enable_bandgap();
#endif
    //init_acmp_pins();

    acmp_channel_get_default_config(ptr, &acmp_channel_configure);
    acmp_channel_configure.plus_input = ACMP_INPUT_DAC_OUT;
    acmp_channel_configure.minus_input = ACMP_INPUT_ANALOG_7;
    acmp_channel_configure.enable_dac = true;
    acmp_channel_configure.enable_cmp_output = true;
    acmp_channel_configure.enable_clock_sync = true;
    acmp_channel_configure.hyst_level = ACMP_HYST_LEVEL_3;
    acmp_channel_configure.invert_output = true;
    acmp_channel_configure.filter_mode = ACMP_FILTER_MODE_CHANGE_AFTER_FILTER;
    acmp_channel_configure.bypass_filter = false ;
    acmp_channel_configure.filter_length = 10 ;
    acmp_channel_config(ptr, 0, &acmp_channel_configure, true);
    acmp_channel_clear_status(ptr, 0, 3);
    ptr->CHANNEL[0].IRQEN |= ACMP_EVENT_RISING_EDGE;
    acmp_channel_enable_irq(IRQn_ACMP_0, 0, ACMP_EVENT_RISING_EDGE, true);
    intc_m_enable_irq_with_priority(IRQn_ACMP_0, 1);
}
