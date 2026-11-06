#include "mcu.h"
#include "SysMain.h"
#include "servo_can.h"
#include "machine.h"
#include "motor_sync.h"
#include "motor.h"
typedef struct
{
    MCAN_Type *can_base;
    uint32_t clock_freq;
    uint32_t irq_num;
} can_info_t;

mcan_tx_frame_t tx_buf;

static volatile bool has_new_rcv_msg;

static volatile bool has_sent_out;

static volatile bool has_error;

static volatile bool tx_event_occurred;

static volatile mcan_rx_message_t s_can_rx_buf;
static volatile mcan_rx_message_t s_can_rx_buf1;
static volatile mcan_tx_event_fifo_elem_t s_can_tx_evt;

static volatile uint8_t error_flags;

void can_configure_init(uint8_t baund_type)
{
    mcan_config_t can_config;
    MCAN_Type *base = HPM_MCAN0; // BOARD_APP_CAN_BASE;
    mcan_filter_elem_t can_filters[16];
    mcan_get_default_config(base, &can_config);
    if (baund_type == 0)
    {
        can_config.baudrate = 1000 * 1000; // 1000kbps */
    }
    else if (baund_type == 1)
    {
        can_config.baudrate = 500 * 1000; // 1000kbps */
    }
    else if (baund_type == 2)
    {
        can_config.baudrate = 250 * 1000; // 250kbps */
    }
    else if (baund_type == 3)
    {
        can_config.baudrate = 125 * 1000; // 125kbps */
    }
    can_config.mode = mcan_mode_normal;

    uint32_t can_src_clk_freq = board_init_can_clock(base);
    uint32_t rxbuf_index = 1;
    can_filters[0].filter_type = MCAN_FILTER_TYPE_SPECIFIED_ID_FILTER;
    can_filters[0].filter_config = MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH;
    can_filters[0].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;
    can_filters[0].id1 = 1;
    can_filters[0].id2 = 1;
    can_filters[0].offset = rxbuf_index;
    can_filters[0].filter_event = 0;
    can_filters[0].store_location = 0;

    can_filters[1].filter_type = MCAN_FILTER_TYPE_SPECIFIED_ID_FILTER;
    can_filters[1].filter_config = MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH;
    can_filters[1].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;
    #if (BOARD_TYPE == BOARD_E884)
    can_filters[1].id1 = 0x380;
    can_filters[1].id2 = 0x380;
    #else
    can_filters[1].id1 = 0x100;
    can_filters[1].id2 = 0x100;
    #endif
    can_filters[1].offset = rxbuf_index;
    can_filters[1].filter_event = 0;
    can_filters[1].store_location = 0;
    can_config.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 0;
    can_config.all_filters_config.std_id_filter_list.filter_elem_list = &can_filters[0];
    can_config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 1;
    can_config.all_filters_config.std_id_filter_list.filter_elem_list = &can_filters[1];
    can_config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 2;
    can_config.all_filters_config.global_filter_config.accept_non_matching_std_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    can_config.all_filters_config.global_filter_config.accept_non_matching_ext_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    can_config.all_filters_config.global_filter_config.reject_remote_std_frame = true;
    can_config.all_filters_config.global_filter_config.reject_remote_ext_frame = true;
    /* can_config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 1 ;

     can_config.all_filters_config.std_id_filter_list.filter_elem_list = &std_id_filter;*/
    hpm_stat_t status = mcan_init(base, &can_config, can_src_clk_freq);
    if (status != status_success)
    {
        //  printf("CAN initialization failed, error code: %d\n", status);
        return;
    }

    mcan_enable_interrupts(base, MCAN_EVENT_RECEIVE); // mcan_enable_interrupts(base, MCAN_EVENT_TRANSMIT|MCAN_EVENT_ERROR|MCAN_EVENT_RECEIVE);//
    intc_m_enable_irq_with_priority(IRQn_MCAN0, 1);
}

//---=============================================================================================
void can1_configure_init(uint8_t baund_type)
{
    
    mcan_config_t can_config;
    MCAN_Type *base           = HPM_MCAN1 ;  //BOARD_APP_CAN_BASE;

    mcan_get_default_config(base, &can_config);
    if(baund_type == 0)
    {
        can_config.baudrate =  500 * 1000; //500kbps */
    }
    else if(baund_type == 1)
    {
        can_config.baudrate =  1000 * 1000; //1000kbps */
    }
    else if(baund_type == 2)
    {
        can_config.baudrate =  250 * 1000; //250kbps */
    }
    else if(baund_type == 3)
    {
        can_config.baudrate =  125 * 1000; //125kbps */
    }
    can_config.mode           = mcan_mode_normal;

    uint32_t can_src_clk_freq = board_init_can_clock(base);
    hpm_stat_t status         = mcan_init(base, &can_config, can_src_clk_freq);
    if (status                != status_success) {
        //  printf("CAN initialization failed, error code: %d\n", status);
        return;
    }

    mcan_enable_interrupts(base, MCAN_EVENT_RECEIVE); //mcan_enable_interrupts(base, MCAN_EVENT_TRANSMIT|MCAN_EVENT_ERROR|MCAN_EVENT_RECEIVE);//
    intc_m_enable_irq_with_priority(IRQn_MCAN1, 1);
    //printf("CAN echo test: Responder is waiting for echo message...\n");




  
}

/***********************************************************************************************************************
 *
 *  Codes
 *
 **********************************************************************************************************************/
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, board_can_isr);
void board_can_isr(void)
{
#if 0
    uint32_t flags = mcan_get_interrupt_flags(BOARD_APP_CAN_BASE);
    /* New message is available in RXFIFO0 */
    if ((flags & MCAN_INT_RXFIFI0_NEW_MSG) != 0) {
        mcan_read_rxfifo(BOARD_APP_CAN_BASE, 0, (mcan_rx_message_t *) &s_can_rx_buf);
        has_new_rcv_msg = true;
    }
    /* New message is available in RXFIFO1 */
    if ((flags & MCAN_INT_RXFIFO1_NEW_MSG) != 0U) {
        mcan_read_rxfifo(BOARD_APP_CAN_BASE, 0, (mcan_rx_message_t *) &s_can_rx_buf);
        has_new_rcv_msg = true;
    }
    /* New message is available in RXBUF */
    if ((flags & MCAN_INT_MSG_STORE_TO_RXBUF) != 0U) {
        has_new_rcv_msg = true;
        mcan_read_rxbuf(BOARD_APP_CAN_BASE, 0, (mcan_rx_message_t *) &s_can_rx_buf);
        mcan_clear_rxbuf_data_available_flag(BOARD_APP_CAN_BASE, 0);
    }
    /* New TX Event occurred */
    if ((flags & MCAN_INT_TX_EVT_FIFO_NEW_ENTRY) != 0) {
        mcan_read_tx_evt_fifo(BOARD_APP_CAN_BASE, (mcan_tx_event_fifo_elem_t *) &s_can_tx_evt);
        tx_event_occurred = true;
    }
    /* Transmit completed */
    if ((flags & (MCAN_EVENT_TRANSMIT)) != 0U) {
        has_sent_out = true;
    }
    /* Error happened */
    if ((flags & MCAN_EVENT_ERROR) != 0) {
        has_error = true;
    }
    mcan_clear_interrupt_flags(BOARD_APP_CAN_BASE, flags);

#endif
    uint32_t flags = mcan_get_interrupt_flags(HPM_MCAN0);

    /* New message is available in RXFIFO0 */
    if ((flags & MCAN_INT_RXFIFO0_NEW_MSG) != 0)
    {
        while (mcan_read_rxfifo(HPM_MCAN0, 0, (mcan_rx_message_t *)&s_can_rx_buf) == status_success)
        {
            if (s_can_rx_buf.std_id == 0x01 || s_can_rx_buf.std_id == 0x380)
            {
                easyCanPush((uint8_t *)s_can_rx_buf.data_8,s_can_rx_buf.std_id,s_can_rx_buf.dlc);
            }
#ifndef BOOT_DEF
            else if (s_can_rx_buf.std_id == 0x100)
            {
                sync_deal_can(GET_DWORD(s_can_rx_buf.data_8[0], s_can_rx_buf.data_8[1], s_can_rx_buf.data_8[2], s_can_rx_buf.data_8[3]) );
            }
#endif
        }
        has_new_rcv_msg = true;
    }
    /* New message is available in RXFIFO1 */
    if ((flags & MCAN_INT_RXFIFO1_NEW_MSG) != 0U)
    {
        while (mcan_read_rxfifo(HPM_MCAN0, 0, (mcan_rx_message_t *)&s_can_rx_buf) == status_success)
        {
            if (s_can_rx_buf.std_id == 0x01|| s_can_rx_buf.std_id == 0x380)
            {
                easyCanPush((uint8_t *)s_can_rx_buf.data_8,s_can_rx_buf.std_id,s_can_rx_buf.dlc);
            }
#ifndef BOOT_DEF
            else if (s_can_rx_buf.std_id == 0x100)
            {
                sync_deal_can(GET_DWORD(s_can_rx_buf.data_8[0], s_can_rx_buf.data_8[1], s_can_rx_buf.data_8[2], s_can_rx_buf.data_8[3]) );
            }
#endif
        }
        has_new_rcv_msg = true;
    }
    /* New message is available in RXBUF */
    if ((flags & MCAN_INT_MSG_STORE_TO_RXBUF) != 0U)
    {
        has_new_rcv_msg = true;
        while (mcan_read_rxbuf(HPM_MCAN0, 0, (mcan_rx_message_t *)&s_can_rx_buf))
        {
            if (s_can_rx_buf.std_id == 0x01|| s_can_rx_buf.std_id == 0x380)
            {
                easyCanPush((uint8_t *)s_can_rx_buf.data_8,s_can_rx_buf.std_id,s_can_rx_buf.dlc);
            }
#ifndef BOOT_DEF
            else if (s_can_rx_buf.std_id == 0x100)
            {
                sync_deal_can(GET_DWORD(s_can_rx_buf.data_8[0], s_can_rx_buf.data_8[1], s_can_rx_buf.data_8[2], s_can_rx_buf.data_8[3]) );
            }
#endif
        }
        mcan_clear_rxbuf_data_available_flag(HPM_MCAN0, 0);
    }
    /* New TX Event occurred */
    if ((flags & MCAN_INT_TX_EVT_FIFO_NEW_ENTRY) != 0)
    {
        mcan_read_tx_evt_fifo(HPM_MCAN0, (mcan_tx_event_fifo_elem_t *)&s_can_tx_evt);
        tx_event_occurred = true;
    }
    /* Transmit completed */
    if ((flags & (MCAN_EVENT_TRANSMIT)) != 0U)
    {
        has_sent_out = true;
    }
    /* Error happened */
    if ((flags & MCAN_EVENT_ERROR) != 0)
    {
        has_error = true;
    }
    mcan_clear_interrupt_flags(HPM_MCAN0, flags);
}
// #######################################################################################
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN1, board_can_isr1);
void board_can_isr1(void)
{
    uint32_t flags = mcan_get_interrupt_flags(HPM_MCAN1);

    /* New message is available in RXFIFO0 */
    if ((flags & MCAN_INT_RXFIFO0_NEW_MSG) != 0)
    {
        while (mcan_read_rxfifo(HPM_MCAN1, 0, (mcan_rx_message_t *)&s_can_rx_buf1) == status_success)
        {
            #ifndef BOOT_DEF
            if (s_can_rx_buf1.std_id ==0x01)
            {
               // easyCanPush((uint8_t *)s_can_rx_buf1.data_8,s_can_rx_buf1.std_id,s_can_rx_buf1.dlc);

              
               if(s_can_rx_buf1.data_8[0]==0x07)
               {
                  mot_set_ext_pos(GET_DWORD(s_can_rx_buf1.data_8[3],s_can_rx_buf1.data_8[4],s_can_rx_buf1.data_8[5],s_can_rx_buf1.data_8[6]))   ;
               }
              
            }
            else
            {
                easyCanPush1((uint8_t *)s_can_rx_buf1.data_8,s_can_rx_buf1.std_id,s_can_rx_buf1.dlc);
            }
            #endif
        }
        has_new_rcv_msg = true;
    }
    /* New message is available in RXFIFO1 */
    if ((flags & MCAN_INT_RXFIFO1_NEW_MSG) != 0U)
    {
        while (mcan_read_rxfifo(HPM_MCAN1, 0, (mcan_rx_message_t *)&s_can_rx_buf1) == status_success)
        {
            #ifndef BOOT_DEF
             if (s_can_rx_buf1.std_id ==0x01)
            {
               // easyCanPush((uint8_t *)s_can_rx_buf1.data_8,s_can_rx_buf1.std_id,s_can_rx_buf1.dlc);

              
               if(s_can_rx_buf1.data_8[0]==0x07)
               {
                  mot_set_ext_pos(GET_DWORD(s_can_rx_buf1.data_8[3],s_can_rx_buf1.data_8[4],s_can_rx_buf1.data_8[5],s_can_rx_buf1.data_8[6]))   ;
               }
              
            }
            else
            {
                easyCanPush1((uint8_t *)s_can_rx_buf1.data_8,s_can_rx_buf1.std_id,s_can_rx_buf1.dlc);
            }
            #endif
        }
        has_new_rcv_msg = true;
    }
    /* New message is available in RXBUF */
    if ((flags & MCAN_INT_MSG_STORE_TO_RXBUF) != 0U)
    {
        has_new_rcv_msg = true;
        while (mcan_read_rxbuf(HPM_MCAN1, 0, (mcan_rx_message_t *)&s_can_rx_buf1))
        {
            #ifndef BOOT_DEF
             if (s_can_rx_buf1.std_id ==0x01)
            {
               // easyCanPush((uint8_t *)s_can_rx_buf1.data_8,s_can_rx_buf1.std_id,s_can_rx_buf1.dlc);

              
               if(s_can_rx_buf1.data_8[0]==0x07)
               {
                  mot_set_ext_pos(GET_DWORD(s_can_rx_buf1.data_8[3],s_can_rx_buf1.data_8[4],s_can_rx_buf1.data_8[5],s_can_rx_buf1.data_8[6]))   ;
               }
               
            }
            else
            {
                easyCanPush1((uint8_t *)s_can_rx_buf1.data_8,s_can_rx_buf1.std_id,s_can_rx_buf1.dlc);
            }
            #endif
        }
        mcan_clear_rxbuf_data_available_flag(HPM_MCAN1, 0);
    }
    /* New TX Event occurred */
    if ((flags & MCAN_INT_TX_EVT_FIFO_NEW_ENTRY) != 0)
    {
        mcan_read_tx_evt_fifo(HPM_MCAN1, (mcan_tx_event_fifo_elem_t *)&s_can_tx_evt);
        tx_event_occurred = true;
    }
    /* Transmit completed */
    if ((flags & (MCAN_EVENT_TRANSMIT)) != 0U)
    {
        has_sent_out = true;
    }
    /* Error happened */
    if ((flags & MCAN_EVENT_ERROR) != 0)
    {
        has_error = true;
    }
    mcan_clear_interrupt_flags(HPM_MCAN1, flags);
}

ATTR_RAMFUNC hpm_stat_t can_send_msg(MCAN_Type *base, uint32_t can_id, uint8_t *pData, uint8_t len)
{
    hpm_stat_t status;
    memset(&tx_buf, 0, sizeof(tx_buf));
    tx_buf.dlc = len; // s_can_rx_buf.dlc;
    tx_buf.std_id = can_id;
    memcpy(tx_buf.data_8, pData, 8);
    status = mcan_transmit_blocking(base, &tx_buf);
    return status;
}
