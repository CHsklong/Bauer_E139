/*
 * MathPidSpd.c
 *
 *  Created on: 2015年3月18日
 *      Author: admin
 */

#include "SysMain.h"

PI_CONTROLLER_SPD gPidSpd;

void PI_SPD_INIT(void)
{
	memset_16(&gPidSpd.Ref,0,sizeof_16(PI_CONTROLLER_SPD));
	gPidSpd.Umax = 0.99f;
	gPidSpd.Umin = -0.99f;
	gPidSpd.Kp = 0.5f;
	gPidSpd.Ki = 0.01f;
	gPidSpd.Kc = 4*0.01f;
	gPidSpd.InterThd = 0.033f;		//积分项
}


