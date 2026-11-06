/*
 *  UserPulse.h
 *
 *  Created on: 2015321
 *  Author: admin
 */

#ifndef USERPULSE_H_
#define USERPULSE_H_

#include "mcu.h"
#include "UserTypeDef.h"

#define PULSE_FILT_LEN 512

typedef struct{
	u16 PwmEn;				//
	u16 Dir;				//
	u16 LmtL;
	u16 LmtR;
	u16 Emgs;
	u32 GrnScale;			//
	u32 GrmScale;			//
	u32 Grn;				//
	u32 Grm;				//
	u32 Grst;				//

	s32 Pool;				//

	s32 Preg;				//
	s32 PregGn;				//

	float MaxSpd;
	float MaxFre;			//
	u16   OvrMaxFreFlag;

	u32   QepLine;
	float QepDline;			//

	u16 PFlag;				//
	u16 NoPulseCnt;			//

	float PincOut;			//
	s16 PincOutI;

	u16 FiltLen;			//
	s32 FltPulSum;			//
	s16 FltRest;			//
	u16 FltIndex;			//
	u16 FltZeroFlag;		//
	u16 FltClrIndex;
	s32 FltAvg;				//
	s16 FltBuf[PULSE_FILT_LEN];

	s16 pOfBuf[POS_LOOP_FRE*10];
	s32 pOfSum;
	u16 pIndex;
	s16 pAvg;
}PulseTypedef;

extern PulseTypedef gPulse;


void gPulseInit(void);
void gPulseHandle(PulseTypedef *dp);
#endif /* USEREXCMD_H_ */
