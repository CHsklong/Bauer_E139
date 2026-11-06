#include "encode.h"
//#include "hpm_qeiv2_drv.h"



uint32_t  encode_cnt;
uint32_t  encode_zcap;

//ATTR_PLACE_AT(".ahb_sram") u

int16_t        ENC_QEPCNT_REG ;

#define		ZCAP_NUM 		20

int16_t		zj = 0;
int16_t		zcap[ZCAP_NUM];
int16_t		zcaps[ZCAP_NUM];

#ifndef APP_QEI_BASE
#define APP_QEI_BASE BOARD_BLDC_QEIV2_BASE
#endif
int qei_init(void)
{
#if 0
    init_qei_trgm_pins();
    
//    qeiv2_phcnt_cmp_match_config_t phcnt_cmp_config = {0};
    qeiv2_reset_counter(APP_QEI_BASE);

    qeiv2_set_work_mode(APP_QEI_BASE, qeiv2_work_mode_abz);
    qeiv2_select_spd_tmr_register_content(APP_QEI_BASE, qeiv2_spd_tmr_as_spd_tm);
    qeiv2_config_z_phase_counter_mode(APP_QEI_BASE, qeiv2_z_count_inc_on_phase_count_max);
    qeiv2_config_phmax_phparam(APP_QEI_BASE, 16384);
    qeiv2_pause_pos_counter_on_fault(APP_QEI_BASE, true);
    qeiv2_config_abz_uvw_signal_edge(APP_QEI_BASE, true, true, false, true, true);

   // intc_m_enable_irq_with_priority(APP_QEI_IRQ, 1);

    qeiv2_release_counter(APP_QEI_BASE);
    qeiv2_set_phase_cnt(APP_QEI_BASE, 0);
#else
    
#endif
    return 0;
}
uint32_t qei_cnt(void)
{
   // return qeiv2_get_phase_cnt(APP_QEI_BASE);
   return 0 ;
}


//-------------spi-----------------------------




//------------------------------------




























































