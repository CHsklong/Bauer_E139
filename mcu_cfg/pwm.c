#include "mcu.h"
//-------------------------------------------//
int32_t motor_clock_hz;
//-------------------------------------------//
uint32_t pwm3_reload;
uint32_t pwm3_reload_half;
uint32_t pwm3_cmp_value ;
uint16_t pwm_ratio_1000 = 0;
//-------------------------------------------//
int8_t  over_current_flag = 0;
void disable_all_pwm_output(void);
uint32_t pwm_isr_flag =0 ;

ATTR_RAMFUNC void pwm_isr(void)
{

    uint32_t flag;
    flag = pwm_get_status(BOARD_PWM_TYPE);
    pwm_clear_status(BOARD_PWM_TYPE, flag );
  
    pwm_isr_flag =flag ;
    if (flag & PWM_SR_FAULTF_MASK)
    {
        disable_all_pwm_output();
        over_current_flag = 1;
        brake_engage();          /* 故障时抱闸, 防止负载跌落 */
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_USER_PWM_IRQ, pwm_isr);
//------------------------------------------------------
void pwm_clear_all_fault(void)
{
    pwm_clear_fault(BOARD_PWM_TYPE) ;
}
void pwm_fault_init(void)
{
    #if (BOARD_TYPE == BOARD_E139)
    //内部运放+比较器 过流报警
    init_opamp();
    init_acmp(HPM_ACMP);
    acmp_channel_config_dac(HPM_ACMP,0, 124);  //   0.8/0.04 = 20
    #elif(BOARD_TYPE == BOARD_E884)
    init_acmp(HPM_ACMP);
    acmp_channel_config_dac(HPM_ACMP,0, 250);  //   满的16.5
    #endif
}
/**
 * [pwm_pins_init description]
 * @param ptr [description]
 */
void pwm_pins_init(PWM_Type *ptr)
{
  //  uint32_t pad_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0) | IOC_PAD_PAD_CTL_HYS_SET(1);
    //HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_GPIO_A_09;
    //HPM_IOC->PAD[IOC_PAD_PA09].PAD_CTL = pad_ctl;
    if (ptr == HPM_PWM0) {
       
        #if (BOARD_TYPE == BOARD_E139|| BOARD_TYPE == BOARD_E884)
        HPM_IOC->PAD[IOC_PAD_PY00].FUNC_CTL = IOC_PY00_FUNC_CTL_PWM0_P_0;   //phase a_l
        HPM_IOC->PAD[IOC_PAD_PY01].FUNC_CTL = IOC_PY01_FUNC_CTL_PWM0_P_1;

        HPM_IOC->PAD[IOC_PAD_PY02].FUNC_CTL = IOC_PY02_FUNC_CTL_PWM0_P_2;   //phase b_l
        HPM_IOC->PAD[IOC_PAD_PY03].FUNC_CTL = IOC_PY03_FUNC_CTL_PWM0_P_3;

        HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_PWM0_P_6;   //phase c_l
        HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_PWM0_P_7;

       // HPM_IOC->PAD[IOC_PAD_PA13].FUNC_CTL = IOC_PA13_FUNC_CTL_PWM0_P_1;
        /* Set PY00-PY03 default function to PGPIO */
        HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = IOC_PY00_FUNC_CTL_SOC_GPIO_Y_00;
        //    HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = pad_ctl;
        HPM_PIOC->PAD[IOC_PAD_PY01].FUNC_CTL = IOC_PY01_FUNC_CTL_SOC_GPIO_Y_01;
        //   HPM_PIOC->PAD[IOC_PAD_PY01].FUNC_CTL = pad_ctl;
        HPM_PIOC->PAD[IOC_PAD_PY02].FUNC_CTL = IOC_PY02_FUNC_CTL_SOC_GPIO_Y_02;
        //    HPM_PIOC->PAD[IOC_PAD_PY02].FUNC_CTL = pad_ctl;
        HPM_PIOC->PAD[IOC_PAD_PY03].FUNC_CTL = IOC_PY03_FUNC_CTL_SOC_GPIO_Y_03;

        #elif (BOARD_TYPE == BOARD_E145)
     //   HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_PWM0_FAULT_1;
        HPM_IOC->PAD[IOC_PAD_PY00].FUNC_CTL = IOC_PY00_FUNC_CTL_PWM0_P_0;   //phase a_l
        HPM_IOC->PAD[IOC_PAD_PY01].FUNC_CTL = IOC_PY01_FUNC_CTL_PWM0_P_1;

     /*   HPM_IOC->PAD[IOC_PAD_PY02].FUNC_CTL = IOC_PY02_FUNC_CTL_PWM0_P_2;   //phase b_l
        HPM_IOC->PAD[IOC_PAD_PY03].FUNC_CTL = IOC_PY03_FUNC_CTL_PWM0_P_3;*/
        HPM_IOC->PAD[IOC_PAD_PA26].FUNC_CTL = IOC_PA26_FUNC_CTL_PWM0_P_2;   //phase b_l
        HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL = IOC_PA27_FUNC_CTL_PWM0_P_3;

        HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_PWM0_P_4;   //phase c_l
        HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_PWM0_P_5;

        HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_PWM0_P_6;   //phase d_l
        HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_PWM0_P_7;

        HPM_IOC->PAD[IOC_PAD_PB14].FUNC_CTL = IOC_PB14_FUNC_CTL_PWM0_FAULT_0;   //phase c_l

        /* Set PY00-PY03 default function to PGPIO */
        HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = IOC_PY00_FUNC_CTL_SOC_GPIO_Y_00;
        //    HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = pad_ctl;
        HPM_PIOC->PAD[IOC_PAD_PY01].FUNC_CTL = IOC_PY01_FUNC_CTL_SOC_GPIO_Y_01;
        #endif
    }
    else if(ptr == HPM_PWM1)
    {
        HPM_IOC->PAD[IOC_PAD_PY00].FUNC_CTL = IOC_PY00_FUNC_CTL_PWM1_P_4;   //phase a_l
        HPM_IOC->PAD[IOC_PAD_PY01].FUNC_CTL = IOC_PY01_FUNC_CTL_PWM1_P_5;

        HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_PWM1_P_0;   //phase b_l
        HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_PWM1_P_1;

        HPM_IOC->PAD[IOC_PAD_PB14].FUNC_CTL = IOC_PB14_FUNC_CTL_PWM1_P_2;   //phase c_l
        HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_PWM1_P_3;

        HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = IOC_PY00_FUNC_CTL_SOC_GPIO_Y_00;
        //    HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = pad_ctl;
        HPM_PIOC->PAD[IOC_PAD_PY01].FUNC_CTL = IOC_PY01_FUNC_CTL_SOC_GPIO_Y_01;
        //   HPM_PIOC->PAD[IOC_PAD_PY01].FUNC_CTL = pad_ctl;

        HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_PWM1_FAULT_0;   //fault
    }

//    HPM_IOC->PAD[IOC_PAD_PA22].FUNC_CTL = IOC_PA22_FUNC_CTL_PWM3_P_02;
}
//--------------------------------------------//
void pwm_init(void)
{
    pwm_pins_init(BOARD_PWM_TYPE) ;
    pwm_central_aligned_waveform_in_pair_init() ;
    pwm_enable_irq(BOARD_PWM_TYPE, PWM_IRQ_FAULT);
    pwm_enable_irq(BOARD_PWM_TYPE, PWM_IRQ_RELOAD);
    brake_pwm_init();                                 /* E139 抱闸并进行PWM 初始化, 上电默认抱闸 */
    intc_m_enable_irq_with_priority(BOARD_USER_PWM_IRQ, 3);
    pwm_start_counter(BOARD_PWM_TYPE);
}
void pwm_central_aligned_waveform_in_pair_init(void)
{
    uint8_t cmp_index   =   CMP_INDEX_0;
    pwm_cmp_config_t        cmp_config[4] = {0};
    pwm_pair_config_t       pwm_pair_config = {0};
    pwm_output_channel_t    pwm_output_ch_cfg;

    pwm_cmp_config_t pwm_cmp_cfg = {0};

    pwm_stop_counter(BOARD_PWM_TYPE);
    reset_pwm_counter();

    //   * reload and start counter

    pwm_set_reload(BOARD_PWM_TYPE, 0, PWM_RELOAD);
    pwm_set_start_count(BOARD_PWM_TYPE, 0, 0);

    //  * config cmp1 and cmp2

    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = PWM_RELOAD >> 1; // + 1>;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[1].mode = pwm_cmp_mode_output_compare;
    cmp_config[1].cmp = PWM_RELOAD + 1;
    cmp_config[1].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[2].enable_ex_cmp  = false;
    cmp_config[2].mode           = pwm_cmp_mode_output_compare;
    cmp_config[2].cmp = PWM_RELOAD - 1 ; //5;
    cmp_config[2].update_trigger = pwm_shadow_register_update_on_shlk;

    cmp_config[3].mode = pwm_cmp_mode_output_compare;
    cmp_config[3].cmp = PWM_RELOAD;
    cmp_config[3].update_trigger = pwm_shadow_register_update_on_modify;

    pwm_get_default_pwm_pair_config(BOARD_PWM_TYPE, &pwm_pair_config);

    //----------------------
    pwm_pair_config.pwm[0].enable_output = true;
    pwm_pair_config.pwm[0].dead_zone_in_half_cycle = 60; //80;//150 ;// 150 ; //200 ;  //350 ; //800;//2us
    pwm_pair_config.pwm[0].invert_output = false;
    pwm_pair_config.pwm[0].fault_mode = pwm_fault_mode_force_output_0 ; //----------------add

    pwm_pair_config.pwm[1].enable_output = true;
    pwm_pair_config.pwm[1].dead_zone_in_half_cycle = 60; //80;//150 ;//150 ; //200 ; //350 ; //800;//2us
    pwm_pair_config.pwm[1].invert_output = false;
    pwm_pair_config.pwm[1].fault_mode = pwm_fault_mode_force_output_0 ;  // -------------add
    //-----------------

    pwm_output_ch_cfg.cmp_start_index = BOARD_BLDC_PWM_TRIG_CMP_INDEX;
    pwm_output_ch_cfg.cmp_end_index   = BOARD_BLDC_PWM_TRIG_CMP_INDEX;
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(BOARD_PWM_TYPE, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &pwm_output_ch_cfg);


    //  * config pwm

    disable_all_pwm_output();  //close output

#if (BOARD_TYPE == BOARD_E139|| BOARD_TYPE == BOARD_E884)
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, UH_PWM_OUTPIN, &pwm_pair_config, cmp_index, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, VH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 2, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, WH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 6, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
#elif (BOARD_TYPE == BOARD_E914)
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, UH_PWM_OUTPIN, &pwm_pair_config, cmp_index, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, VH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 2, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, WH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 4, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
#elif (BOARD_TYPE == BOARD_E145)
    pwm_pair_config.pwm[1].invert_output = true;
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, AH_PWM_OUTPIN, &pwm_pair_config, cmp_index, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
    pwm_pair_config.pwm[1].invert_output = true;
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, BH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 2, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
    pwm_pair_config.pwm[1].invert_output = true;
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, CH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 4, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }
    pwm_pair_config.pwm[1].invert_output = true;
    if (status_success != pwm_setup_waveform_in_pair(BOARD_PWM_TYPE, DH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 6, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
    }    
#endif

    
//-------------------------------------------

    /// Set comparator /
//    memset(&pwm_cmp_cfg, 0x00, sizeof(pwm_cmp_config_t));
    pwm_cmp_cfg.enable_ex_cmp  = false;
    pwm_cmp_cfg.mode           = pwm_cmp_mode_output_compare;
    pwm_cmp_cfg.update_trigger = pwm_shadow_register_update_on_modify;//pwm_shadow_register_update_on_shlk;

    /// Select comp8 and trigger at the middle of a pwm cycle /
    pwm_cmp_cfg.cmp = PWM_RELOAD - 1;
    pwm_config_cmp(BOARD_PWM_TYPE, 8, &pwm_cmp_cfg);

//------------------------------------
    //pwm 比较器中断
    pwm_cmp_cfg.cmp = PWM_RELOAD - 400;   // 提前时间
    pwm_config_cmp(BOARD_PWM_TYPE, 9, &pwm_cmp_cfg);
    pwm_enable_irq(BOARD_PWM_TYPE, PWM_IRQEN_CMPIRQEX_SET(9)) ;
//-----------------------------------------
    // Issue a shadow lock /
    //pwm_issue_shadow_register_lock_event(BOARD_APP_ADC_TRIG_PWM0);

    // Set comparator channel for trigger a /
    pwm_output_ch_cfg.cmp_start_index = 8;   // start channel
    pwm_output_ch_cfg.cmp_end_index   = 8;   // end channel
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(BOARD_PWM_TYPE, 8, &pwm_output_ch_cfg);

//----------------------------------------------


    //my fault internal 0
    //enable pwm fault protect
    //pwm_fault_source_config_t config;

    //config.external_fault_active_low = false;
    //config.source_mask = PWM_GCR_FAULTI0EN_MASK;
    //config.fault_recover_at_rising_edge = true;
    //config.fault_output_recovery_trigger = 3 ;   //0;
    //pwm_config_fault_source(HPM_PWM1, &config);// config trgm
    //trgm_output_update_source(HPM_TRGM0, TRGM_TRGOCFG_PWM_FAULTI0, HPM_TRGM0_INPUT_SRC_CMP1_OUT);


#if (BOARD_TYPE == BOARD_E139)
    pwm_load_cmp_shadow_on_match(BOARD_PWM_TYPE, BOARD_BLDCPWM_CMP_TRIG_CMP,  &cmp_config[3]);
    pwm_config_cmp(BOARD_PWM_TYPE, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &cmp_config[2]);


    disable_all_pwm_output();  //close output
    //----  pwm_start_counter(HPM_PWM0);
    pwm_issue_shadow_register_lock_event(BOARD_PWM_TYPE);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_0, CMP_INDEX_1, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_2, CMP_INDEX_3, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_4, CMP_INDEX_5, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
#elif (BOARD_TYPE == BOARD_E884)
    pwm_load_cmp_shadow_on_match(BOARD_PWM_TYPE, BOARD_BLDCPWM_CMP_TRIG_CMP,  &cmp_config[3]);
    pwm_config_cmp(BOARD_PWM_TYPE, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &cmp_config[2]);


    disable_all_pwm_output();  //close output
    //----  pwm_start_counter(HPM_PWM0);
    pwm_issue_shadow_register_lock_event(BOARD_PWM_TYPE);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_0, CMP_INDEX_1, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_2, CMP_INDEX_3, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_4, CMP_INDEX_5, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
#elif (BOARD_TYPE == BOARD_E914)
    pwm_fault_source_config_t config;

    config.fault_external_0_active_low = true;
    config.source_mask = PWM_GCR_FAULTE0EN_MASK;
    config.fault_recover_at_rising_edge = true;
    config.fault_output_recovery_trigger = 3 ;   //0;
    pwm_config_fault_source(BOARD_PWM_TYPE, &config);// config trgm
    trgm_output_update_source(HPM_TRGM0, TRGM_FILTCFG_PWM1_FAULT0, HPM_TRGM0_INPUT_SRC_PWM1_FAULT0);
//--------------------------------------------


//--------------------------------------------

    pwm_load_cmp_shadow_on_match(BOARD_PWM_TYPE, BOARD_BLDCPWM_CMP_TRIG_CMP,  &cmp_config[3]);
    pwm_config_cmp(BOARD_PWM_TYPE, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &cmp_config[2]);


    disable_all_pwm_output();  //close output
    //----  pwm_start_counter(HPM_PWM1);
    pwm_issue_shadow_register_lock_event(BOARD_PWM_TYPE);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_0, CMP_INDEX_1, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_2, CMP_INDEX_3, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_4, CMP_INDEX_5, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
#elif (BOARD_TYPE == BOARD_E145)
    //my fault internal 0
    //enable pwm fault protect
    pwm_fault_source_config_t config;

    config.fault_external_0_active_low = true;

    config.source_mask = PWM_GCR_FAULTE0EN_MASK;
    config.fault_recover_at_rising_edge = true;

    config.fault_output_recovery_trigger = 3 ;   //0;
    pwm_config_fault_source(BOARD_PWM_TYPE, &config);// config trgm
    trgm_output_update_source(HPM_TRGM0, TRGM_FILTCFG_PWM0_FAULT0, HPM_TRGM0_INPUT_SRC_PWM0_FAULT0);
    pwm_load_cmp_shadow_on_match(BOARD_PWM_TYPE, BOARD_BLDCPWM_CMP_TRIG_CMP,  &cmp_config[3]);
    pwm_config_cmp(BOARD_PWM_TYPE, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &cmp_config[2]);
    disable_all_pwm_output();  //close output
    //----  pwm_start_counter(HPM_PWM0);
    pwm_issue_shadow_register_lock_event(BOARD_PWM_TYPE);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_0, CMP_INDEX_1, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_2, CMP_INDEX_3, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_4, CMP_INDEX_5, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);
    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_6, CMP_INDEX_7, PWM_RELOAD / 4, PWM_RELOAD * 3 / 4);  
#endif
   

}

//-------------------------------------------//
void reset_pwm_counter(void)
{
    pwm_enable_reload_at_synci(BOARD_PWM_TYPE);
}
//-------------------------------------------//
ATTR_RAMFUNC void enable_all_pwm_output(void)
{
    pwm_disable_sw_force(BOARD_PWM_TYPE);
    pwm_config_force_cmd_timing(BOARD_PWM_TYPE, pwm_force_at_reload);

    pwm_disable_pwm_sw_force_output(BOARD_PWM_TYPE, 1);
    pwm_disable_pwm_sw_force_output(BOARD_PWM_TYPE, 3);
    pwm_disable_pwm_sw_force_output(BOARD_PWM_TYPE, 5);
    clock_cpu_delay_us(2);
    pwm_disable_pwm_sw_force_output(BOARD_PWM_TYPE, 0);

    pwm_disable_pwm_sw_force_output(BOARD_PWM_TYPE, 2);

    pwm_disable_pwm_sw_force_output(BOARD_PWM_TYPE, 4);
}
//-------------------------------------------//
ATTR_RAMFUNC void disable_all_pwm_output(void)
{
#if (BOARD_TYPE == BOARD_E145)
    pwm_config_force_cmd_timing(BOARD_PWM_TYPE, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, AH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, AL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, BH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, BL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, CH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, CL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, DH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, DL_PWM_OUTPIN);
    pwm_set_force_output(BOARD_PWM_TYPE,
                         PWM_FORCE_OUTPUT(AH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(AL_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(BH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(BL_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(CH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(CL_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(DH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(DL_PWM_OUTPIN, pwm_output_0));
    pwm_enable_sw_force(BOARD_PWM_TYPE);
#else
    pwm_config_force_cmd_timing(BOARD_PWM_TYPE, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, UH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, UL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, VH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, VL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, WH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, WL_PWM_OUTPIN);
    pwm_set_force_output(BOARD_PWM_TYPE,
                         PWM_FORCE_OUTPUT(UH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(UL_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(VH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(VL_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(WH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(WL_PWM_OUTPIN, pwm_output_0));
    pwm_enable_sw_force(BOARD_PWM_TYPE);
#endif
}
//-------------------------------------------//

void close_high_open_low_pwm_output(void)
{
#if (BOARD_TYPE == BOARD_E145)
    pwm_config_force_cmd_timing(BOARD_PWM_TYPE, pwm_force_at_reload);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, AH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, AL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, BH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, BL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, CH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, CL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, DH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, DL_PWM_OUTPIN);
    pwm_set_force_output(BOARD_PWM_TYPE,
                         PWM_FORCE_OUTPUT(AH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(AL_PWM_OUTPIN, pwm_output_1)
                         | PWM_FORCE_OUTPUT(BH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(BL_PWM_OUTPIN, pwm_output_1)
                         | PWM_FORCE_OUTPUT(CH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(CL_PWM_OUTPIN, pwm_output_1)
                         | PWM_FORCE_OUTPUT(DH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(DL_PWM_OUTPIN, pwm_output_1));
    pwm_enable_sw_force(BOARD_PWM_TYPE);
#else
    pwm_config_force_cmd_timing(BOARD_PWM_TYPE, pwm_force_at_reload);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, UH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, UL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, VH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, VL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, WH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_PWM_TYPE, WL_PWM_OUTPIN);
    pwm_set_force_output(BOARD_PWM_TYPE,
                         PWM_FORCE_OUTPUT(UH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(UL_PWM_OUTPIN, pwm_output_1)
                         | PWM_FORCE_OUTPUT(VH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(VL_PWM_OUTPIN, pwm_output_1)
                         | PWM_FORCE_OUTPUT(WH_PWM_OUTPIN, pwm_output_0)
                         | PWM_FORCE_OUTPUT(WL_PWM_OUTPIN, pwm_output_1));
    pwm_enable_sw_force(BOARD_PWM_TYPE);
#endif
}
//-------------------------------------------//


//-------------------------------------------//
void Fault_SWrecovery(PWM_Type *pwm_x, uint8_t chn, uint8_t FAULTM)
{
    uint32_t cfg;
    /* enable pwm fault protect */
    cfg = pwm_x->PWMCFG[chn];
    cfg &= (~PWM_PWMCFG_FAULTRECTIME_MASK);
    cfg &= (~PWM_PWMCFG_FAULTMODE_MASK);
    cfg |= (pwm_fault_recovery_on_fault_clear | PWM_PWMCFG_FAULTMODE_SET(FAULTM));
    pwm_x->PWMCFG[chn] = cfg;
}

void Fault_AUTOrecovery(PWM_Type *pwm_x, uint8_t chn, uint8_t FAULTM)
{
    uint32_t cfg;
    /* enable pwm fault protect */
    cfg = pwm_x->PWMCFG[chn];
    cfg &= (~PWM_PWMCFG_FAULTRECTIME_MASK);
    cfg &= (~PWM_PWMCFG_FAULTMODE_MASK);
    cfg |= (pwm_fault_recovery_immediately | PWM_PWMCFG_FAULTMODE_SET(FAULTM));
    pwm_x->PWMCFG[chn] = cfg;
}

void Turnoff_allupper(PWM_Type *pwm_x)
{
    pwm_config_force_cmd_timing(pwm_x, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(pwm_x, 0);
    pwm_enable_pwm_sw_force_output(pwm_x, 1);
    pwm_enable_pwm_sw_force_output(pwm_x, 2);
    pwm_enable_pwm_sw_force_output(pwm_x, 3);
    pwm_enable_pwm_sw_force_output(pwm_x, 4);
    pwm_enable_pwm_sw_force_output(pwm_x, 5);
    pwm_set_force_output(pwm_x,
                         PWM_FORCE_OUTPUT(0, pwm_output_1)
                         | PWM_FORCE_OUTPUT(1, pwm_output_0)
                         | PWM_FORCE_OUTPUT(2, pwm_output_1)
                         | PWM_FORCE_OUTPUT(3, pwm_output_0)
                         | PWM_FORCE_OUTPUT(4, pwm_output_1)
                         | PWM_FORCE_OUTPUT(5, pwm_output_0));
    pwm_enable_sw_force(BOARD_PWM_TYPE);
}
void Turnoff_alllower(PWM_Type *pwm_x)
{
    pwm_config_force_cmd_timing(pwm_x, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(pwm_x, 0);
    pwm_enable_pwm_sw_force_output(pwm_x, 1);
    pwm_enable_pwm_sw_force_output(pwm_x, 2);
    pwm_enable_pwm_sw_force_output(pwm_x, 3);
    pwm_enable_pwm_sw_force_output(pwm_x, 4);
    pwm_enable_pwm_sw_force_output(pwm_x, 5);
    pwm_set_force_output(pwm_x,
                         PWM_FORCE_OUTPUT(0, pwm_output_0)
                         | PWM_FORCE_OUTPUT(1, pwm_output_1)
                         | PWM_FORCE_OUTPUT(2, pwm_output_0)
                         | PWM_FORCE_OUTPUT(3, pwm_output_1)
                         | PWM_FORCE_OUTPUT(4, pwm_output_0)
                         | PWM_FORCE_OUTPUT(5, pwm_output_1));
    pwm_enable_sw_force(BOARD_PWM_TYPE);
}

void Turnoff_all(PWM_Type *pwm_x)
{
    pwm_config_force_cmd_timing(pwm_x, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(pwm_x, 0);
    pwm_enable_pwm_sw_force_output(pwm_x, 1);
    pwm_enable_pwm_sw_force_output(pwm_x, 2);
    pwm_enable_pwm_sw_force_output(pwm_x, 3);
    pwm_enable_pwm_sw_force_output(pwm_x, 4);
    pwm_enable_pwm_sw_force_output(pwm_x, 5);
    pwm_set_force_output(pwm_x,
                         PWM_FORCE_OUTPUT(0, pwm_output_0)
                         | PWM_FORCE_OUTPUT(1, pwm_output_0)
                         | PWM_FORCE_OUTPUT(2, pwm_output_0)
                         | PWM_FORCE_OUTPUT(3, pwm_output_0)
                         | PWM_FORCE_OUTPUT(4, pwm_output_0)
                         | PWM_FORCE_OUTPUT(5, pwm_output_0));
    pwm_enable_sw_force(BOARD_PWM_TYPE);
}
void Recover_all(PWM_Type *pwm_x)
{
    pwm_disable_sw_force(BOARD_PWM_TYPE);
    pwm_config_force_cmd_timing(pwm_x, pwm_force_at_reload);

    pwm_disable_pwm_sw_force_output(pwm_x, 1);
    pwm_disable_pwm_sw_force_output(pwm_x, 3);
    pwm_disable_pwm_sw_force_output(pwm_x, 5);
    clock_cpu_delay_us(2);
    pwm_disable_pwm_sw_force_output(pwm_x, 0);

    pwm_disable_pwm_sw_force_output(pwm_x, 2);

    pwm_disable_pwm_sw_force_output(pwm_x, 4);

}

//-----------------------------------------------------

//----------------------------------------------------

/*==============================================================================
 * E139 抱闸(刹车) PWM ---- OUT0 / PA13
 *   可配置频率/占空比方波(默认 10kHz/50%) 经 Q1(反相) 驱动 Q6(PMOS) 斩波 36V 励磁线圈
 *   用 GPTMR1 通道3 的比较输出，不占用电机 PWM0 通道
 *   松闸时序: 先 PA13 恒高(全压)强励磁 BRAKE_BOOST_MS, 再切到上述 PWM 保持
 *============================================================================*/
#if (BOARD_TYPE == BOARD_E139)
#define BRAKE_GPTMR             HPM_GPTMR1
#define BRAKE_GPTMR_CH          (3U)
#define BRAKE_GPTMR_CLK         (clock_gptmr1)
#define BRAKE_PIN_PAD           (IOC_PAD_PA13)
#define BRAKE_PIN_PORT          (GPIO_DO_GPIOA)
#define BRAKE_PIN_NUM           (13U)

static uint8_t s_brake_released = 0U;

/* 配置刹车 PWM 的频率与占空比 */
void brake_pwm_config(uint32_t freq_hz, uint8_t duty_pct)
{
    gptmr_channel_config_t cfg;
    uint32_t clk_hz;
    uint32_t reload;

    if (freq_hz == 0U) 
    {
        freq_hz = BRAKE_PWM_FREQ_HZ;
    }
    if (duty_pct == 0U) 
    {
        duty_pct = 1U;
    } 
    else if (duty_pct > 100U) 
    {
        duty_pct = 100U;
    }

    clock_add_to_group(BRAKE_GPTMR_CLK, 0);
    clk_hz = clock_get_frequency(BRAKE_GPTMR_CLK);
    reload = clk_hz / freq_hz;
    if (reload < 100U) 
    {
        reload = 100U;
    }

    gptmr_channel_get_default_config(BRAKE_GPTMR, &cfg);
    cfg.mode                      = gptmr_work_mode_no_capture;
    cfg.reload                    = reload;
    cfg.cmp[0]                    = 2U;                            /* 周期起点后拉高 */
    cfg.cmp[1]                    = (reload * duty_pct) / 100U;    /* 到占空比拉低 */
    cfg.cmp_initial_polarity_high = false;                         /* 初始低电平 */
    cfg.enable_cmp_output         = true;
    cfg.enable_software_sync      = false;
    cfg.dma_request_event         = gptmr_dma_request_disabled;

    gptmr_channel_config(BRAKE_GPTMR, BRAKE_GPTMR_CH, &cfg, false);
    gptmr_channel_reset_count(BRAKE_GPTMR, BRAKE_GPTMR_CH);
}


/* 抱闸：停 PWM，PA13 切回 GPIO 输出低 -> Q1 截止 -> Q6 关断 -> 线圈失电 */
ATTR_RAMFUNC void brake_engage(void)
{
    gptmr_stop_counter(BRAKE_GPTMR, BRAKE_GPTMR_CH);                                  //关闭刹车线圈的PWM输出，
    HPM_IOC->PAD[BRAKE_PIN_PAD].FUNC_CTL = IOC_PA13_FUNC_CTL_GPIO_A_13;               //PA13改为GPIO模式
    gpio_set_pin_output_with_initial(HPM_GPIO0, BRAKE_PIN_PORT, BRAKE_PIN_NUM, LOW);  //将PA13配置为输出模式，并输出低电平，抱闸
    s_brake_released = 0U;
}

/* 松闸：PA13 切到 GPTMR1_COMP_3，输出指定频率/占空比方波（保持阶段） */
ATTR_RAMFUNC void brake_release(void)
{
    gptmr_channel_reset_count(BRAKE_GPTMR, BRAKE_GPTMR_CH);                           //打开刹车线圈的PWM输出
    HPM_IOC->PAD[BRAKE_PIN_PAD].FUNC_CTL = IOC_PA13_FUNC_CTL_GPTMR1_COMP_3;           //PA13改为定时器复用功能
    gptmr_start_counter(BRAKE_GPTMR, BRAKE_GPTMR_CH);                                 //定时器开始计数，
    s_brake_released = 1U;
}

/* 强励磁：松闸前的 1s 高电平输出阶段 */
ATTR_RAMFUNC void brake_boost(void)
{
    gptmr_stop_counter(BRAKE_GPTMR, BRAKE_GPTMR_CH);                                  //关闭刹车线圈的PWM输出
    HPM_IOC->PAD[BRAKE_PIN_PAD].FUNC_CTL = IOC_PA13_FUNC_CTL_GPIO_A_13;               //PA13改为GPIO模式
    gpio_set_pin_output_with_initial(HPM_GPIO0, BRAKE_PIN_PORT, BRAKE_PIN_NUM, 1U);   //PA13输出恒高(全压强励磁)
    s_brake_released = 1U;
}

/* 1 = 松闸，0 = 抱闸 */
ATTR_RAMFUNC void brake_set(uint8_t release)
{
    if (release) {
        brake_release();
    } else {
        brake_engage();
    }
}

uint8_t brake_is_released(void)
{
    return s_brake_released;
}

/* 抱闸，并初始化配置pwm波 */
void brake_pwm_init(void)
{
    brake_engage();                     //启动前抱闸
    
    brake_pwm_config(BRAKE_PWM_FREQ_HZ, BRAKE_PWM_DUTY_PCT);        //配置 10kHz 频率、50% 占空比的 PWM 波(保持参数)
}

#else   /* 其它板型预留刹车 */

void brake_pwm_init(void) { }
void brake_pwm_config(uint32_t freq_hz, uint8_t duty_pct) { (void)freq_hz; (void)duty_pct; }
void brake_release(void) { }
void brake_engage(void) { }
void brake_set(uint8_t release) { (void)release; }
uint8_t brake_is_released(void) { return 1U; }
void brake_boost(void) { }

#endif

