#ifndef USER_ABS_H_
#define USER_ABS_H_
#include "dataTypeDef.h"

#define	NRZ_TXD_MAX_LEN		10
#define	NRZ_REC_MAX_LEN		30

#define	ABS_EE_LEN		4
#define ABS_CF_ABS 		0x02
#define	ABS_CF_ABM		0x8A
#define	ABS_CF_ID		0x92
#define	ABS_CF_ABS_ABM2	0x2B
#define	ABS_CF_ABS_ABM3	0x1A


#define ABS_CF_WRITE_EE 0x32
#define ABS_CF_READ_EE 	0xEA
/*
#define ABS_EE_ADS_0  0x00
#define ABS_EE_ADS_1  0x01
#define ABS_EE_ADS_2  0x02
#define ABS_EE_ADS_3  0x03
*/
#define ABS_EE_ADS_0  0x04
#define ABS_EE_ADS_1  0x05
#define ABS_EE_ADS_2  0x06
#define ABS_EE_ADS_3  0x07

#define	ABS_SH_N		4

typedef struct {
	u16 sts;
	u16 TxdLen;		//
	u16 RecLen;     //
	s16	abm;
	u32	abs;
	u32	absBk;
        u32     spiread;
	s64	abs64;
	s32	absMlt;		//		  100
	s32	absInc;
	s32     CalibratedAngle;		//
	s32	Half;

	s32	absIncSum;
	s32	absIncRes;
	s32	absIncOut;

	Uint32 ReferenceLine;	// *4
	Uint32 CircleLine;		// *4 * ENCODER_CIRCLE

	u8	absID;
	u8	absSF;					//
	u8	absRdErr;
	u8	absCrcErr;				//
	u8	WriteInit[4];
	u8	ReadInit[4];
	u8 TxdBuf[NRZ_TXD_MAX_LEN];
	u8 RcvBuf[NRZ_REC_MAX_LEN];	//

	uint8_t crc_abs ;
	uint8_t crc_spi ;


	u16 ZFlag;	//
	u16	intUartCnt;
	u16	txdmaCnt;
	u16	rxdmaCnt;
	u16	dma6_4Cnt;
	u16	dma5_4Cnt;
} ABS_TYPE;


extern	ABS_TYPE	AbsPar;
void ABS_POWER_WAIT(void);
void ABSInit(void);

void ABS_INIT_WRITE(void);
void ABS_INIT_READ(void);
void ABS_READ(void);


void ABS_RCV_OK(void);




#endif
