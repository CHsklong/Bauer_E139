#include "SysMain.h"

EEPROM_PAR      eep_par;
EEPROM_PAR2     eep_par2;

GAN_POS_LOOP_CMD_TYPE    posloop;

ADC_CLA             Adc_gan_pos;

FILTER_TYPE_1       gFilter_gan_pos;



MACHINE_DATA_TYPE    mach_spd;
//POS_CNT_TYPE         poscnt;

//PI_CLA          Pid_gan_pos;
//float       gan_pos_err;

uint8_t     upmaxStop = 0 ;

LEN_MEAS_TYPEDEF    len_meas;

