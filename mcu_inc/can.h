#ifndef CAN_H
#define CAN_H

#define CAN_500K	1
#define CAN_1MHZ	2
#define CAN_BURD		CAN_1MHZ  		//CAN_500K  //


void can_configure_init(uint8_t baund_type);
void can1_configure_init(uint8_t baund_type);



ATTR_RAMFUNC hpm_stat_t can_send_msg(MCAN_Type *base ,uint32_t can_id,uint8_t *pData,uint8_t len);

#endif