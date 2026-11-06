#include "mcu.h"
#include "SysMain.h"
#include "mcdrv.h"
__IO uint8_t seq_full_complete_flag;

__IO uint8_t Adc1_Trigger_Times;
//__attribute__((section(".fast_ram"))) uint32_t adc_buff[BOARD_ADC_DMA_BUFFER_LEN];
//uint32_t adc_buff[BOARD_ADC_DMA_BUFFER_LEN];

uint32_t adc0_buff[16];
// ATTR_PLACE_AT(".ahb_sram")
//ATTR_PLACE_AT(".fast_ram")  uint32_t adc1_buff[16];
uint32_t adc1_buff[16];
//ATTR_PLACE_AT(".fast_ram")  uint32_t adc2_buff[16];
uint32_t adc2_buff[16];
//ATTR_PLACE_AT(".fast_ram")  uint32_t adc_sample[16];
uint32_t adc_sample[16];

uint32_t adc0_sample[16];
uint32_t adc1_sample[16];
uint32_t adc2_sample[16];

//ATTR_PLACE_AT(".fast_ram")  int16_t adc_delt[16];
int16_t adc_delt[16];
int16_t adc_delt_avg[16];
int32_t adcsum;
int16_t adcavg;
int16_t adcavg_delt;
//ATTR_PLACE_AT(".fast_ram")  float    adc_volt[16];
float    adc_volt[16];
//ATTR_PLACE_AT(".fast_ram")  int32_t  runTime ;
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(4) uint32_t adc_data[1024 * 10];

int16_t  runTime , runTimeMax, runj;
int16_t  zL_pos_adc_val;
int16_t  zL_cur_pwm_val;

float   AngOffSet_zl_pos ;
float   AngOffSet_zl_cur ;

float  zl_cur_f  ;
float  zl_pos_f  ;

uint32_t j = 0;
ATTR_RAMFUNC void isr_adc16(void)
{
    uint32_t status;
//    uint8_t  i;
    static u16 LedTicker = 0;
    status = adc16_get_status_flags(BOARD_APP_ADC16_BASE);

    if (ADC16_INT_STS_SEQ_CMPT_GET(status)) {

        // SYS_LED_TOG();

        /* Set flag to read memory data */
        seq_full_complete_flag = 1;
        Adc1_Trigger_Times = 1;
#ifndef NXP_MOTOR_CTRL

        adcsum = 0;
        for (i = 0; i < ADC_LENS ; i++) //ADC_LENS = 8
        {
            adc_sample[i] = (adc0_buff[i] & 0xffff);

            adc0_sample[i] = (adc0_buff[i] & 0xffff);
            adc1_sample[i] = (adc1_buff[i] & 0xffff);
            adc2_sample[i] = (adc2_buff[i] & 0xffff);

            adc_delt[i] = adc1_sample[i] - adc2_sample[i];
            adc_volt[i] = adc_sample[i] * (float)(3.3 / 65535.0) ;
        }
        for (i = 0; i < 4 ; i++) {

            adcsum += adc_sample[i] ;
            adc_delt_avg[i] = adc_sample[i] - adcavg;
        }

        adcavg = adcsum >> 2;
        adcavg_delt = adcavg - 32768;

        
        gMainIsrHanlde();
#else

	LedTicker++;
	if (LedTicker >= 10000)
	{
           // SYS_LED_TOG();
            LedTicker =0 ;
	}	
       // SYS_LED_HIGH();
        ADCA_IRQHandler() ;
       // SYS_LED_LOW();
#endif
        /* Clear seq_complete status */
        adc16_clear_status_flags(BOARD_APP_ADC16_BASE, adc16_event_seq_full_complete);


    }
}
SDK_DECLARE_EXT_ISR_M(IRQn_ADC0, isr_adc16)

#if (BOARD_TYPE == BOARD_E139 || BOARD_TYPE == BOARD_E884)
//-------------------------------------------------------------------------------
/*
 * @brief initialize sequence channel.
*/
int8_t init_adc0_sequence_config(ADC16_Type *ptr, uint32_t *buff)
{
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 21;
    ch_cfg.ch           = U_ADC_CHANEL ;  //U_ADC0_INA8;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
    ch_cfg.ch           =  VDC_ADC_CHANEL ; //ZL_POS_ADC0_INA14;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
//    ch_cfg.ch           = 9;
//    if(adc16_init_channel(ptr, &ch_cfg))return -1;
    /* Set a sequence config */
    seq_cfg.seq_len    = ADC_LENS;
    seq_cfg.restart_en = false;
    seq_cfg.cont_en    = true;
    seq_cfg.sw_trig_en = true;
    seq_cfg.hw_trig_en = true;



    for (int i = 0; i < ADC_LENS_STEP1; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = U_ADC_CHANEL ; //U_ADC0_INA8;
    }
    for (int i = ADC_LENS_STEP1; i < ADC_LENS; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = VDC_ADC_CHANEL ; //ZL_POS_ADC0_INA14;
    }

    //seq_cfg.queue[ADC_LENS-1].seq_int_en = true;

    /* Initialize a sequence */
    adc16_set_seq_config(ptr, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)buff);
    dma_cfg.buff_len_in_4bytes = ADC_LENS;
    dma_cfg.stop_en            = false  ; //
    dma_cfg.stop_pos           = 0; //dma_cfg.stop_pos           = ADC_LENS;

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(ptr, &dma_cfg);

    /* Enable sequence complete interrupt */
    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_single_complete);
    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_full_complete);

    /* enable irq 系统中断 */
    intc_m_enable_irq_with_priority(IRQn_ADC0, 4);

    return 0;
}

//-------------------------------------------------------------------------------
/*
 * @brief initialize sequence channel.
*/
int8_t init_adc1_sequence_config(ADC16_Type *ptr, uint32_t *buff)
{
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 21;
    ch_cfg.ch           = V_ADC_CHANEL ; //V_ADC1_INA5;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
    ch_cfg.ch           = REF_ADC_CHANEL ; //ZL_CUR_ADC1_INA11;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;

    /* Set a sequence config */
    seq_cfg.seq_len    = ADC_LENS;
    seq_cfg.restart_en = false;
    seq_cfg.cont_en    = true;
    seq_cfg.sw_trig_en = true;
    seq_cfg.hw_trig_en = true;

    for (int i = 0; i < ADC_LENS_STEP1; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = V_ADC_CHANEL ; //V_ADC1_INA5;
    }
    for (int i = ADC_LENS_STEP1; i < ADC_LENS; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = REF_ADC_CHANEL ; //ZL_CUR_ADC1_INA11;
    }
    /*
    for (int i = 8; i < 16; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = 9;
    }
    */
//   seq_cfg.queue[ADC_LENS-1].seq_int_en = true;



    /* Initialize a sequence */
    adc16_set_seq_config(ptr, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)buff);
    dma_cfg.buff_len_in_4bytes = ADC_LENS;
    dma_cfg.stop_en            = true;
    dma_cfg.stop_pos           = ADC_LENS;

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(ptr, &dma_cfg);

    /* Enable sequence complete interrupt */
//    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_single_complete);


    /* enable irq 系统中断 */
//    intc_m_enable_irq_with_priority(IRQn_ADC0, 1);

    return 0;
}
//-------------------------------------------------------------------------------

#elif (BOARD_TYPE == BOARD_E914)
//-------------------------------------------------------------------------------
/*
 * @brief initialize sequence channel.
*/
int8_t init_adc0_sequence_config(ADC16_Type *ptr, uint32_t *buff)
{
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 21;
    ch_cfg.ch           = V_ADC_CHANEL ;  //V_ADC0_2;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
    ch_cfg.ch           =  VDC_ADC_CHANEL ; //VDC_ADC0_11;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
//    ch_cfg.ch           = 9;
//    if(adc16_init_channel(ptr, &ch_cfg))return -1;
    /* Set a sequence config */
    seq_cfg.seq_len    = ADC_LENS;
    seq_cfg.restart_en = false;
    seq_cfg.cont_en    = true;
    seq_cfg.sw_trig_en = true;
    seq_cfg.hw_trig_en = true;



    for (int i = 0; i < ADC_LENS_STEP1; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = V_ADC_CHANEL ; 
    }
    for (int i = ADC_LENS_STEP1; i < ADC_LENS; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = VDC_ADC_CHANEL ; 
    }

    //seq_cfg.queue[ADC_LENS-1].seq_int_en = true;

    /* Initialize a sequence */
    adc16_set_seq_config(ptr, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)buff);
    dma_cfg.buff_len_in_4bytes = ADC_LENS;
    dma_cfg.stop_en            = false  ; //
    dma_cfg.stop_pos           = 0; //dma_cfg.stop_pos           = ADC_LENS;

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(ptr, &dma_cfg);

    /* Enable sequence complete interrupt */
    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_single_complete);
    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_full_complete);

    /* enable irq 系统中断 */
    intc_m_enable_irq_with_priority(IRQn_ADC0, 4);

    return 0;
}

//-------------------------------------------------------------------------------
/*
 * @brief initialize sequence channel.
*/
int8_t init_adc1_sequence_config(ADC16_Type *ptr, uint32_t *buff)
{
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 21;
    ch_cfg.ch           = U_ADC_CHANEL ; //V_ADC1_INA1;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
   /* ch_cfg.ch           = REF_ADC_CHANEL ; //ZL_CUR_ADC1_INA11;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;*/

    /* Set a sequence config */
    seq_cfg.seq_len    = 4;
    seq_cfg.restart_en = false;
    seq_cfg.cont_en    = true;
    seq_cfg.sw_trig_en = true;
    seq_cfg.hw_trig_en = true;

    for (int i = 0; i < 4; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = U_ADC_CHANEL ; //V_ADC1_INA5;
    }
   /* for (int i = ADC_LENS_STEP1; i < ADC_LENS; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = REF_ADC_CHANEL ; //ZL_CUR_ADC1_INA11;
    }*/
    /*
    for (int i = 8; i < 16; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = 9;
    }
    */
//   seq_cfg.queue[ADC_LENS-1].seq_int_en = true;



    /* Initialize a sequence */
    adc16_set_seq_config(ptr, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)buff);
    dma_cfg.buff_len_in_4bytes = 4;
    dma_cfg.stop_en            = true;
    dma_cfg.stop_pos           = 4;

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(ptr, &dma_cfg);

    /* Enable sequence complete interrupt */
//    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_single_complete);


    /* enable irq 系统中断 */
//    intc_m_enable_irq_with_priority(IRQn_ADC0, 1);

    return 0;
}
//-------------------------------------------------------------------------------
#elif (BOARD_TYPE == BOARD_E145)
//-------------------------------------------------------------------------------
/*
 * @brief initialize sequence channel.
*/
int8_t init_adc0_sequence_config(ADC16_Type *ptr, uint32_t *buff)
{
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 21;
    ch_cfg.ch           = U_ADC_CHANEL ;  //U_ADC0_INA8;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
    ch_cfg.ch           =  VDC_ADC_CHANEL ; //ZL_POS_ADC0_INA14;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
//    ch_cfg.ch           = 9;
//    if(adc16_init_channel(ptr, &ch_cfg))return -1;
    /* Set a sequence config */
    seq_cfg.seq_len    = ADC_LENS;
    seq_cfg.restart_en = false;
    seq_cfg.cont_en    = true;
    seq_cfg.sw_trig_en = true;
    seq_cfg.hw_trig_en = true;



    for (int i = 0; i < ADC_LENS_STEP1; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = U_ADC_CHANEL ; //U_ADC0_INA8;
    }
    for (int i = ADC_LENS_STEP1; i < ADC_LENS; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = VDC_ADC_CHANEL ; //ZL_POS_ADC0_INA14;
    }

    //seq_cfg.queue[ADC_LENS-1].seq_int_en = true;

    /* Initialize a sequence */
    adc16_set_seq_config(ptr, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)buff);
    dma_cfg.buff_len_in_4bytes = ADC_LENS;
    dma_cfg.stop_en            = false  ; //
    dma_cfg.stop_pos           = 0; //dma_cfg.stop_pos           = ADC_LENS;

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(ptr, &dma_cfg);

    /* Enable sequence complete interrupt */
    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_single_complete);
    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_full_complete);

    /* enable irq 绯荤粺涓柇 */
    intc_m_enable_irq_with_priority(IRQn_ADC0, 4);

    return 0;
}

//-------------------------------------------------------------------------------
/*
 * @brief initialize sequence channel.
*/
int8_t init_adc1_sequence_config(ADC16_Type *ptr, uint32_t *buff)
{
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 21;
    ch_cfg.ch           = V_ADC_CHANEL ; //V_ADC1_INA5;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;
    ch_cfg.ch           = VDC_ADC_CHANEL ; //ZL_CUR_ADC1_INA11;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;

    /* Set a sequence config */
    seq_cfg.seq_len    = ADC_LENS;
    seq_cfg.restart_en = false;
    seq_cfg.cont_en    = true;
    seq_cfg.sw_trig_en = true;
    seq_cfg.hw_trig_en = true;

    for (int i = 0; i < ADC_LENS_STEP1; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = V_ADC_CHANEL ; //V_ADC1_INA5;
    }
    for (int i = ADC_LENS_STEP1; i < ADC_LENS; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = VDC_ADC_CHANEL ; //;
    }
    /*
    for (int i = 8; i < 16; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = 9;
    }
    */
//   seq_cfg.queue[ADC_LENS-1].seq_int_en = true;



    /* Initialize a sequence */
    adc16_set_seq_config(ptr, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)buff);
    dma_cfg.buff_len_in_4bytes = ADC_LENS;
    dma_cfg.stop_en            = true;
    dma_cfg.stop_pos           = ADC_LENS;

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(ptr, &dma_cfg);

    /* Enable sequence complete interrupt */
//    adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_single_complete);


    /* enable irq 绯荤粺涓柇 */
//    intc_m_enable_irq_with_priority(IRQn_ADC0, 1);

    return 0;
}
//-------------------------------------------------------------------------------
#endif


int8_t init_adc2_sequence_config(ADC16_Type *ptr, uint32_t *buff)
{
    adc16_seq_config_t seq_cfg;
    adc16_dma_config_t dma_cfg;
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 21;
    ch_cfg.ch           = VDC_ADC2_INA5;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;

    ch_cfg.ch           = REF_ADC2_INA3;
    if (adc16_init_channel(ptr, &ch_cfg))return -1;

    /* Set a sequence config */
    seq_cfg.seq_len    = ADC_LENS;
    seq_cfg.restart_en = false;
    seq_cfg.cont_en    = true;
    seq_cfg.sw_trig_en = true;
    seq_cfg.hw_trig_en = true;

    for (int i = 0; i < 2; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch =  VDC_ADC2_INA5;
    }

    for (int i = 2; i < ADC_LENS; i++) {
        seq_cfg.queue[i].seq_int_en = false;
        seq_cfg.queue[i].ch = VDC_ADC2_INA5 ; // REF_ADC2_INA3;
    }

//   seq_cfg.queue[ADC_LENS-1].seq_int_en = true;



    /* Initialize a sequence */
    adc16_set_seq_config(ptr, &seq_cfg);

    /* Set a DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)buff);
    dma_cfg.buff_len_in_4bytes = ADC_LENS;
    dma_cfg.stop_en            = false;
    dma_cfg.stop_pos           = ADC_LENS;//dma_cfg.stop_pos           = ADC_LENS-1;//

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(ptr, &dma_cfg);

    /* Enable sequence complete interrupt */
//   adc16_enable_interrupts(HPM_ADC0, adc16_event_seq_single_complete);


    /* enable irq 系统中断 */
//    intc_m_enable_irq_with_priority(IRQn_ADC0, 1);

    return 0;
}

//-------------------------------------------------------------------------------
int8_t hw_ADC0_squ_Init(void)
{
    //  init_adc_pins();
    board_init_adc16_clock(HPM_ADC0, true);

    adc16_config_t cfg;
    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);

    cfg.res            = adc16_res_16_bits;
    cfg.conv_mode      = adc16_conv_mode_sequence;
    cfg.adc_clk_div    = 4;
    cfg.sel_sync_ahb   = false;

    cfg.adc_ahb_en = true;  //

    adc16_init(HPM_ADC0, &cfg);

    if (init_adc0_sequence_config(HPM_ADC0, &adc0_buff[0]))
        return  -1;
    /* Trigger source initialization */
//    init_trigger(HPM_PWM0);

    /* Trigger mux initialization */
    init_trigger_mux_squ(HPM_TRGM0, TRGM_TRGOCFG_ADC0_STRGI);
//   init_trigger_mux_squ(HPM_TRGM0,TRGM_TRGOCFG_ADC1_STRGI);
//   init_trigger_mux_squ(HPM_TRGM0,TRGM_TRGOCFG_ADC2_STRGI);

    return  0;
}

//-----------------------------------------------------------------------

int8_t hw_ADC1_squ_Init(void)
{
    //  init_adc_pins();
    board_init_adc16_clock(HPM_ADC1, true);

    adc16_config_t cfg;
    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);

    cfg.res            = adc16_res_16_bits;
    cfg.conv_mode      = adc16_conv_mode_sequence;
    cfg.adc_clk_div    = 4;
    cfg.sel_sync_ahb   = false;
    cfg.adc_ahb_en = true;

    adc16_init(HPM_ADC1, &cfg);



    if (init_adc1_sequence_config(HPM_ADC1, &adc1_buff[0]))
        return  -1;






    /* Trigger source initialization */
//    init_trigger(HPM_PWM0);
    /* Trigger mux initialization */
    init_trigger_mux_squ(HPM_TRGM0, TRGM_TRGOCFG_ADC1_STRGI);
    return  0;
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------


//----------------------------------------------------------------------


//----------------------------------------------------------------------

/*
 * @brief initialize pwm0 to trigger ADC.
*/

void init_trigger(PWM_Type * ptr)
{
    pwm_cmp_config_t pwm_cmp_cfg;
    pwm_output_channel_t pwm_output_ch_cfg;

    /* TODO: Set PWM Clock Source and divider */

    /* 33.33KHz reload at 200MHz */
    pwm_stop_counter(ptr);
    //---reset_pwm_counter()------
    //pwm_enable_reload_at_synci(ptr);
    //trgm_output_update_source(HPM_TRGM0, TRGM_TRGOCFG_PWM_SYNCI, 1);
    //trgm_output_update_source(HPM_TRGM0, TRGM_TRGOCFG_PWM_SYNCI, 0);

    uint32_t freq, reload;
    freq = clock_get_frequency(clock_mot0);
    //   freq = clock_get_frequency(clock_mot2);
    reload = freq / 10000 - 1;
    pwm_set_reload(ptr, 0, reload);
    /* Set comparator */
    memset(&pwm_cmp_cfg, 0x00, sizeof(pwm_cmp_config_t));
    pwm_cmp_cfg.enable_ex_cmp  = false;
    pwm_cmp_cfg.mode           = pwm_cmp_mode_output_compare;
    pwm_cmp_cfg.update_trigger = pwm_shadow_register_update_on_modify;//pwm_shadow_register_update_on_shlk;

    /* Select comp8 and trigger at the middle of a pwm cycle */
    pwm_cmp_cfg.cmp = reload / 2;
    pwm_config_cmp(ptr, 8, &pwm_cmp_cfg);

    /* Issue a shadow lock */
    //pwm_issue_shadow_register_lock_event(BOARD_APP_ADC_TRIG_PWM0);

    /* Set comparator channel for trigger a */
    pwm_output_ch_cfg.cmp_start_index = 8;   /* start channel */
    pwm_output_ch_cfg.cmp_end_index   = 8;   /* end channel */
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(ptr, 8, &pwm_output_ch_cfg);

    pwm_start_counter(ptr);
}

/*
 * @brief initialize trigger mux as PTRGI0A and PTRGI0B.
*/
void init_trigger_mux_squ(TRGM_Type * ptr, uint8_t TRGOCFG_OUT)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type = trgm_output_same_as_input;
#if (BOARD_TYPE == BOARD_E145)
    trgm_output_cfg.input  = HPM_TRGM0_INPUT_SRC_PWM0_CH8REF;
#elif (BOARD_TYPE == BOARD_E139 || BOARD_TYPE == BOARD_E884)
    trgm_output_cfg.input  = HPM_TRGM0_INPUT_SRC_PWM0_CH8REF;
#elif (BOARD_TYPE == BOARD_E914)
    trgm_output_cfg.input  = HPM_TRGM0_INPUT_SRC_PWM1_CH8REF;
#endif
    trgm_output_config(ptr, TRGOCFG_OUT, &trgm_output_cfg);
}

//-------------------------------------------------------------------------------



//------------------------------------------------------------------------------



//---------------------------------------------------------------------------------
