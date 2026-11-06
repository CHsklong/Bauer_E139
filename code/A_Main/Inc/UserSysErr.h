/*
 * UserSysErr.h
 *
 *  Created on: 2016412
 *      Author: admin
 */

#ifndef USERSYSERR_H_
#define USERSYSERR_H_

#include "mcu.h"


/*********************************************************************************************/
//
struct MC_SYS_ERR_HD_BIT
{
	u32 ShortCircuit:1;			//   					E-1
	u32 CurSensorError1:1;  	//			E-2
	u32 CurSensorError2:1;  	//			E-3
	u32 OverPhaseCurrent:1;		//				E-4
	u32 BrkOvrCur:1;			//				E-5
	u32 PowerUpErr:1;			//		E-6
	u32 EPROMError:1;			//EEPROM 			E-7
	u32 CpldError:1;			//cpld			E-8
	u32 VrefErrV:1;				//	E-9
	u32 VrefErrT:1;				//		E-10
	u32 PowLos:1;				//				E-11
	u32 PowInterp:1;			//				E-12
	u32 MotoOvrTmp:1;			//				E-13
	u32 MotoTmpSenErr:1;                    //		E-14
	u32 OverTemp:1;				//					E-15
	u32 OverVoltage:1;			//					E-16
	u32 LowVoltage:1;			//				E-17


	u32 Res:15;
};

#define LV_ERR_ID 17

struct MC_SYS_ERR_CTL_BIT
{
	u32 MotoErr:1;				//				E-101
	u32 SysParErr:1;			//				E-102
	u32 PulseErr:1;				//				E-103
	u32 OverPosition:1;			//				E-104
	u32 OverSpeed:1;			//					E-105
	u32 OverBreak:1;			//				E-106
	u32 BreakOverVolt:1;		//				E-107
	u32 OverPower:1;			//					E-108
	u32 PZErr:1;				//				E-109
	u32 MotoWireLose:1;			//				E-110
	u32 EncErrUVW:1;			//				E-111
	u32 EncErr_ABZ:1;			//		E-112
	u32 EncPar:1;				//			    e113
	u32 EncRx_null:1;			//abs - 				E-114
	u32 Res:18;
};

struct MC_SYS_ERR_EMG_BIT
{
	u16 SEmgsErr:1;				//				E-201
	u16 PosLmtR:1;				//				E-202
	u16 PosLmtL:1;				//				E-203
	u16 Res:13;
};


struct MC_SYS_ERR_COM_BIT
{
	u16 COMErr:1;				//				E-301
	u16 ComOvt:1;				//				E-302
	u16 Res:14;
};



union MC_SYS_ERR_HD
{
	u32 all;
	struct MC_SYS_ERR_HD_BIT bit;
};

union MC_SYS_ERR_CTL
{
	u32 all;
	struct MC_SYS_ERR_CTL_BIT bit;
};


union MC_SYS_ERR_EMG
{
	u32 all;
	struct MC_SYS_ERR_EMG_BIT bit;
};

union MC_SYS_ERR_COM
{
	u32 all;
	struct MC_SYS_ERR_COM_BIT bit;
};


typedef struct{
	u16 Index;
	union MC_SYS_ERR_HD ErrHd;
	union MC_SYS_ERR_CTL ErrCtl;
	union MC_SYS_ERR_EMG ErrEmg;
	union MC_SYS_ERR_COM ErrCom;

}SYS_ERR_TYPEDEF;


extern SYS_ERR_TYPEDEF gSysErr;
extern SYS_ERR_TYPEDEF gSysErcEn;
extern SYS_ERR_TYPEDEF gSysErrStop;
/*********************************************************************************************/



/*********************************************************************************************/
void gSysErrVarInit(void);
u16 gSysErrGetId(void);
void gSysErrClear(u16 ErrId);
void gSysErrClearAll(void);
/*********************************************************************************************/

#endif /* USERSYSERR_H_ */
