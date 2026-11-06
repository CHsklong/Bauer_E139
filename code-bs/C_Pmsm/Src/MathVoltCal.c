/*
 * MathVoltCal.c
 *
 *  Created on: 2016617
 *      Author: admin  MOTO_BASE_VOLT
 */

#include "SysMain.h"

PHASEVOLTAGE PaseVolt;

void PaseVoltInit(void)
{
	memset_16(&PaseVolt.DcBusVolt,0,sizeof_16(PHASEVOLTAGE));
	PaseVolt.DcBusVolt = 1;//MOTO_FASE_VOLT*1.414;
}
