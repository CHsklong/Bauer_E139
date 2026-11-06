#ifndef  _ABS_MOTOR_H_
#define  _ABS_MOTOR_H_

#include "hpm_common.h"

#define     MT_ABS_BIT      (21)
#define     MT_SHIFT        (21-MT_ABS_BIT)

enum CLIB_STATE{CLIB_STATE_IDLE=0,CLIB_STATE_READ_CLIB_RESULT,CLIB_STATE_RESV_CLIB_RESULT,CLIB_STATE_READ_SAVE_EPP_RESULT,CLIB_STATE_RESV_SAVE_EPP_RESULT,CLIB_STAT_CLIB_FAIL,CLIB_STATE_CLIB_SUCCESS} ;

typedef  struct {

	uint8_t index ;
	uint8_t enable ;
	uint8_t state ;
	uint32_t cnt ;
	uint8_t status112 ;
	uint8_t status113 ;
	uint8_t finish_3;
	uint8_t finish_write;
	uint16_t cnt3;
	uint16_t cntw;
	uint8_t  ret ;
	uint8_t  send_flag ;
} CALIBRATION_TYPEDEF;


extern CALIBRATION_TYPEDEF  cali_abs ;


//ATTR_RAMFUNC void ABS_EEP_READ_START(void);

//ATTR_RAMFUNC void ABS_EEP_READ_RCV(void);

void ABS_MOTOR_DAM_TX_START(void);

uint32_t ABS_MOTOR_READ_ANGLE(void);

void ABS_MOTOR_MT6835_clib_begin(void) ;

uint8_t ABS_MOTOR_Get_MT6835_Clib_Send_flag(void) ;

void ABS_MOTOR_Clear_MT6835_Clib_Send_flag(void) ;
	
uint8_t ABS_MOTOR_Get_MT6835_Clib_Result(void) ;
void ABS_MOTOR_Init(void);
// void SPI_READ_MT6835(void);

// void SPI_TEST_MT6835(void);
// void SPI_WIRTE_TEST_MT6835(void);





#endif
