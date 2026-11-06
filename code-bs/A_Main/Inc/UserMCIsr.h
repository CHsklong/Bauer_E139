/*
 * UserMCISR.h
 *
 *  Created on: 2016-4-5
 *  Author: Administrator
 */

#ifndef USERMCISR_H_
#define USERMCISR_H_

#include "mcu.h"


enum{
	SYS_PWUP = 0,
	SYS_INIT,
	SYS_RUNN,
	SYS_TEST,
	SYS_ZERO,
	SYS_DEBUG,
	SYS_HALT
};



/*********************************************************************************************/
//
struct MC_CTL_BIT{
	u32 SysSts:4;		//
	u32 SonBk:1;		//
	u32 PwmEn:1;		//PWM
	u32 HdPwmEn:1;		//PWM
	u32 SON:1;
	u32 JOG:1;
	u32 res:23;
};

union MC_CTL{
	u32 All;
	struct MC_CTL_BIT bit;
};
/*********************************************************************************************/

/*********************************************************************************************/
//
struct MC_SON_BIT{
	u16 SoftSon:1;		//
	u16 JogSon:1;		//
	u16 DebugSon:1;
	u16 ExSon:1;
	u16 SonAll:1;
	u16 SonAllBk:1;
	u16 res:10;
};

union MC_SON{
	u16 All;
	struct MC_SON_BIT bit;
};
/*********************************************************************************************/


typedef struct{
	u16 Index;
	union MC_CTL Ctl;
	union MC_SON Son;
}SYS_CTL_VAR_TYPEDEF;


extern SYS_CTL_VAR_TYPEDEF gSysCtlVar;

#define SYS_CTL_VAR_DEFAULTS {0,{0},{0}}

extern u16 gPwmIsrTacker;

extern void (*MC_Task_Ptr)(void);//
extern	void (*ABS_TASK_PTR)(void);

void gMainIsrHanlde(void);

#endif /* USERTIM_H_ */
