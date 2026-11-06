/*
 * UserSysPar.h
 *
 *  Created on: 201647
 *      Author: admin
 */

#ifndef USERSYSPAR_H_
#define USERSYSPAR_H_

#include "mcu.h"
#include "UserTypeDef.h"
#if POW_VER_STS == POW_VER_24V
//---------------------------------------------------------------------------------------//
//ID
enum {
	DRVBORD_INFO_2A = 1,
	DRVBORD_INFO_5A,
	DRVBORD_INFO_10A,
	DRVBORD_INFO_15A,
	DRVBORD_INFO_20A,
	DRVBORD_INFO_30A,
	DRVBORD_INFO_50A,
	DRVBORD_INFO_70A,
	DRVBORD_INFO_80A
};

//---------------------------------------------------------------------------------------//
#else
//---------------------------------------------------------------------------------------//
//ID
enum {
	DRVBORD_INFO_5A = 11,
	DRVBORD_INFO_10A,
	DRVBORD_INFO_15A,
	DRVBORD_INFO_25A,
	DRVBORD_INFO_35A,
	DRVBORD_INFO_50A,
	DRVBORD_INFO_75A,
	DRVBORD_INFO_100A
};

//---------------------------------------------------------------------------------------//

#endif

//---------------------------------------------------------------------------------------//
//EEPROM24C08 64

//EEPROM24C08 6432word/8 = 4page
//i2c 24c16 one page = 16byte = 8 word  ( <<4 = 16bytes)


enum {
	SYSPAR_P0_ADS =  0,			////11
	SYSPAR_P1_ADS =  2 << 4,			////30
	SYSPAR_P2_ADS =  6 << 4,			////19
	SYSPAR_P3_ADS = 10 << 4,			////15
	SYSPAR_P4_ADS = 14 << 4,			////13
	SYSPAR_P5_ADS = 16 << 4,			////9
	SYSPAR_P6_ADS = 18 << 4,			////8
	SYSPAR_P7_ADS = 20 << 4,			////11
	SYSPAR_P8_ADS = 22 << 4,			////14
	SYSPAR_P9_ADS = 24 << 4,			////22
	SYSPAR_P10_ADS = 28 << 4			////
};
#define ADS_PAGE_OFFSET 32

//---------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------//
enum {
	SYSPAR_OK = 0,		//
	SYSPAR_READ_ERR,	//
	SYSPAR_WRITE_ERR,	//
	SYSPAR_CHECK_ERR	//
};
//---------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------//
#define EROM_INIT 0X11
#define EPROM_INIT_CMD_ALL 999 	//
#define EPROM_INIT_CMD_L1  1	//
//---------------------------------------------------------------------------------------//


//---------------------------------------------------------------------------------------//
typedef struct {
	float Kpp;	//
	float Kps_R;//
	float Kis;	//
	float Kpt;	//
	float Kit;	//
	float KWe;	//
	float Flux;	//Wb
	float CurScale;	//
	float CurMax;	//
} MC_SYS_CTL_PAR;
extern MC_SYS_CTL_PAR gMcCtlPar;
//---------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------//
//
typedef struct {
	u16 	Index;	//
	float 	CurMax;	//
	float 	Scale;	//
} MC_DRVBORD_INFO;
//---------------------------------------------------------------------------------------//



//------------------------------P0--------------------------------------------------------//
typedef struct {
	u16 CTL;	//
	u16 PDIR;	//
	u16 LSTP;	//
	u16	SON;	//
	u16 KPP1;	//
	u16 KVP1;	//
	u16 KVI;	//
	u16 PFG;	//7
	u16 PFF;	//8
	u16 KVF;	//9
	u16 MSPD;	//

	u16 MTOR;	//
	u16 DisId;	//
	/*
		u16 KPP2;	//
		u16 KPP3;	//
		u16 KPP4;	//

		u16 KVP2;	//
		u16 KVP3;	//
		u16 KVP4;	//

		u16 GCC;	//
		u16 GPE;	//
		u16 GUT;	//
	*/
} MC_SYS_PAR_P0;
//---------------------------------------------------------------------------------------//


//---------------------------P1  input
//---------------------------------------------------------//
typedef struct {

	u16	INUM;	//
	u16 DI1;	//DI1
	u16 DI2;	//DI2
	u16 DI3;	//DI3
	u16 DI4;	//DI4
	u16 DI5;	//DI5
	u16 DI6;	//DI6
	u16 DI7;	//DI7
	u16 DI8;	//DI8
	u16 DI9;	//DI9
	u16 DRT;	//DI
	u16 LOGI1 ;
	u16 LOGI2 ;
	u16 LOGI3 ;
	u16 LOGI4 ;
	u16 LOGI5 ;
	u16 LOGI6 ;
	u16 LOGI7 ;
	u16 LOGI8 ;
	u16 LOGI9 ;

	u16 OSET;	//offset
	u16 SFLT;	//						PS:OK
	u16 VCM;	//					PS:OK
	u16 ZST;	//
	s16 SAO;	//OFFSET
	u16 VCMLPF;	//
	u16 TFLT;	//							PS:OK
	u16 TCM;	//					PS:OK
	u16 TST;	//
	s16 TAO;	//OFFSET
	u16 TCMLPF;	//


} MC_SYS_PAR_P1;
//----------------------P2  OUTPUT
//-----------------------------------------------------------------//

typedef struct {
	u16 ONUM;	//

	u16 DO1;	//DOUT1
	u16 DO2;	//DOUT2
	u16 DO3;	//DOUT3
	u16 DO4;	//DOUT4
	u16 DO5;	//DOUT5
	u16	DO6;	//DOUT6

	u16	LOGO1;
	u16 LOGO2;
	u16 LOGO3;
	u16 LOGO4;
	u16 LOGO5;
	u16 LOGO6;

	u16 MBT1;	//
	s16 MBT2;	//

	u16 PER;	//
	u16 SPOK;	//
	s16 TNCM;	//
	u16 ZSPD;	//
	s16 SSPD;	//


} MC_SYS_PAR_P2;

//---------------P3 ----------------
typedef struct {
	u16 PAS;	//
	u16 PPT;	//
	u16	PFT;	//
	u16 DCLR;	//
	u16 PFLT;	//

	u16 GRN1;	//1								PS:OK
	u16 GRM1;	//1								PS:OK
	u16 POSH;	//
	u16 POSL;	//
	s16	PSPD;	//

	u16 PACC;	//
	u16 PDEC;	//
	u16 GRN2;	//2								PS:OK
	u16 GRM2;	//2								PS:OK


	u16 AOUT;	// 0/1							PS:OK
	u16 GROUT;	//							PS:OK


} MC_SYS_PAR_P3;

//---------------P4 ----------------
typedef struct {
	u16 SAS;	//
	u16	SPD0;	//
	s16 CSPD;	//
	u16 TACC;	//0	PS:OK
	u16 TDEC;	//0	PS:OK
	u16	TSL;	//						PS:NULL

	u16 LT_TOR;	//PSTL;	//
	s16 SPD1;	//									PS:OK
	s16 SPD2;	//									PS:OK
	s16 SPD3;	//									PS:OK
	s16 SPD4;	//									PS:OK
	s16 SPD5;	//									PS:OK
	s16 SPD6;	//									PS:OK
	s16 SPD7;	//									PS:OK




} MC_SYS_PAR_P4;

//---------------P5 ----------------
typedef struct {
	u16 TAS;	//
	u16	TQ0 ;	//
	s16     CTOR;	//
	u16	CMDFT;	//
	u16	UVFT;	//
	u16	DQFT;
	u16	LT_SPD;	//
	s16     TQ1;	//									PS:OK
	s16     TQ2;	//									PS:OK
	s16     TQ3;	//									PS:OK

} MC_SYS_PAR_P5;



//-------------------------P6------------------------------------------------------------//
typedef struct {
	u16 ADR;	//485
	u16 BRT;	//485
	u16 PTL;	//485
	u16 FLT;	//				//NO
	u16 CWD;	//				//NO
	u16 CMM;	//
	u16 CDT;	//				//NO
	u16 MNS;	//					//NO
	u16 SAVE;	//


} MC_SYS_PAR_P6;
//---------------------------------------------------------------------------------------//

//-------------------------P7 ------------------------------------------------------------//
typedef struct {

	u16 PDEV;	//
	u16 SDEV;	//
	u16 LVH;	//									PS:OK
	u16 OVH;	//									PS:OK
	u16 TMPH;	//

	u16 BRES;	//									PS:OK
	u16 BPWR;	//									PS:OK
	u16 BPWH;	//
	u16 OLH;	//								PS:OK
	u16 OLT;	//								PS:OK

	u16 CRSHA;	//			//NO
	u16 CRSHT;	// 			//NO
} MC_SYS_PAR_P7;
//---------------------------------------------------------------------------------------//

//-------------------------P8 ------------------------------------------------------------//
typedef struct {
	u16 SVER;	//0
	u16 HVER;	//1
	u16 MTYPE;	//2
	u16 ENCT;	//3
	u16	POLES;	//4

	u16 INIT;	//5
	u16 SMOD;	//6
	u16	ENFT;	//7ABS
	u16 CEN;	//8
	u16 MTBK;	//9

	u16 ASH1;	//10N
	u16 ASH2;	//11N-1
	u16 ASH3;	//N-2
	u16 ASH4;	//N-3
	u16 ASH5;	//N-4



} MC_SYS_PAR_P8;

//-------------------------P9------------------------------------------------------------//

typedef struct {
	u16 Index;

	u16 NCF1;	//									PS:NULL
	u16 DPH1;	//								PS:NULL
	u16 NCF2;	//									PS:NULL
	u16 DPH2;	//								PS:NULL
	u16 NCF3;	//									PS:NULL
	u16 DPH3;	//								PS:NULL
	u16 NLP;	//				USE
	u16 ANCF;	//
	u16 GBIT2;	//				USE
	u16 RES1;	// 09
	u16 GDR;	//		10					PS:OK
	s16 MCLCW;	//
	s16 MCLCCW;	//
	u16 GEN;	//-
	u16 GMD;	//-0cw1ccw2cw+ccw
	u16 GPOSH;	//-H
	u16 GPOSL;	//-L
	u16 GSPD;	//-
	u16 GADT;	//-
	u16 GTIM;	//-
	u16 JSL;	//-
	u16 TEP;	//


} MC_SYS_PAR_P9;
//---------------------------------------------------------------------------------------//


#define SYS_PAR_P0_LEN sizeof_16(MC_SYS_PAR_P0)
#define SYS_PAR_P1_LEN sizeof_16(MC_SYS_PAR_P1)
#define SYS_PAR_P2_LEN sizeof_16(MC_SYS_PAR_P2)
#define SYS_PAR_P3_LEN sizeof_16(MC_SYS_PAR_P3)
#define SYS_PAR_P4_LEN sizeof_16(MC_SYS_PAR_P4)
#define SYS_PAR_P5_LEN sizeof_16(MC_SYS_PAR_P5)
#define SYS_PAR_P6_LEN sizeof_16(MC_SYS_PAR_P6)
#define SYS_PAR_P7_LEN sizeof_16(MC_SYS_PAR_P7)
#define SYS_PAR_P8_LEN sizeof_16(MC_SYS_PAR_P8)
#define SYS_PAR_P9_LEN sizeof_16(MC_SYS_PAR_P9)


//---------------------------------------------------------------------------------------//
//

//
struct DATATYPEBIT {
	u16 Symbol: 4;		//	01
	u16 BitWide: 4;		//
	u16	Modify: 4;		//
	u16 Save: 2;			//
	u16 FloatPoint: 2;	//
};

union DATATYPE {
	u16 All;
	struct DATATYPEBIT bit;
};

//
typedef struct
{
	unsigned int DataMax;		//
	int DataMin;		//
	int DefaultData;	//
	union DATATYPE DataType;		//

} ParRomStruct;
//---------------------------------------------------------------------------------------//


extern MC_SYS_PAR_P0 SysParP0;
extern MC_SYS_PAR_P1 SysParP1;
extern MC_SYS_PAR_P2 SysParP2;
extern MC_SYS_PAR_P3 SysParP3;
extern MC_SYS_PAR_P4 SysParP4;
extern MC_SYS_PAR_P5 SysParP5;
extern MC_SYS_PAR_P6 SysParP6;
extern MC_SYS_PAR_P7 SysParP7;
extern MC_SYS_PAR_P8 SysParP8;
extern MC_SYS_PAR_P9 SysParP9;

extern MC_DRVBORD_INFO DrvBoarInfo;
extern const MC_DRVBORD_INFO DRVBORD_INFO_INIT[];

extern const ParRomStruct SYSPAR_INIT_P0[sizeof_16(MC_SYS_PAR_P0)];
extern const ParRomStruct SYSPAR_INIT_P1[sizeof_16(MC_SYS_PAR_P1)];
extern const ParRomStruct SYSPAR_INIT_P2[sizeof_16(MC_SYS_PAR_P2)];
extern const ParRomStruct SYSPAR_INIT_P3[sizeof_16(MC_SYS_PAR_P3)];
extern const ParRomStruct SYSPAR_INIT_P4[sizeof_16(MC_SYS_PAR_P4)];
extern const ParRomStruct SYSPAR_INIT_P5[sizeof_16(MC_SYS_PAR_P5)];
extern const ParRomStruct SYSPAR_INIT_P6[sizeof_16(MC_SYS_PAR_P6)];
extern const ParRomStruct SYSPAR_INIT_P7[sizeof_16(MC_SYS_PAR_P7)];
extern const ParRomStruct SYSPAR_INIT_P8[sizeof_16(MC_SYS_PAR_P8)];
extern const ParRomStruct SYSPAR_INIT_P9[sizeof_16(MC_SYS_PAR_P9)];


u16 gSysParPartRead(u16 *pbf, u16 ads, u16 len);
u16 gSysParPartWrite(u16 *pbf, u16 ads, u16 len);
void gSysParInitAll(u16 level);
u16 gSysParCheckAll(void);
u16 gSysParLoadAll(void);
void SysParHandle(void);
void MotoCtlParMacro(void);

#endif /* USERSYSPARNEW_H_ */
