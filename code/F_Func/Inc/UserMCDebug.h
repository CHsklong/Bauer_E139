/*
 * UserMCDebug.h
 *
 *  Created on: 2015Äê4ÔÂ27ÈÕ
 *      Author: admin
 */

#ifndef USERMCDEBUG_H_
#define USERMCDEBUG_H_

#include "Bsp.h"

typedef struct{
	u16 En;
	u16 Enbk;
	u16 Sts;
	u16 Ticker;
	u16 Times;
	u16 Mode;
	float Kp;
	float Ki;
	float Kc;
	float KPV;
	float KIV;
	float Ref;
	float OutRef;

}MC_DEBUG_TYPEDEF;

 


void MC_Debug(void);

#endif /* USERMCDEBUG_H_ */
