/*
 * UserMCZero.h
 *
 *  Created on: 201546
 *      Author: admin
 */

#ifndef USERMCZERO_H_
#define USERMCZERO_H_

#include "mcu.h"


typedef struct {
	u16 Sts;
	u16 Mode;
	u16 Ticker;
	u32 EzQepFbk;
	s32 EzEepErr;
	u32 QepMax;
	u32 QepMin;
	u16 ZeroEn;
	u32 ZeroTicker;
	u16 ZeroSts;
	u16 PwmEn;
	float Theta;
	float Qref;
	float Dref;
	float Dgvn;
	u32	  HoldTime;
	u32 QepLine;
	u32 QepElc;

	uint16_t DrefAddTim;
	uint16_t DrefHoldTim;
	uint16_t refSpd;
	uint16_t stop;

} MOTO_ZERO_TYPEDEF;

extern MOTO_ZERO_TYPEDEF MotoZeroVar;

void MC_Zero(void);


#endif /* USERMCZERO_H_ */
