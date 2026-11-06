/*
 * UserSysErr.c
 *
 *  Created on: 2016年4月12日
 *      Author: admin
 */


#include "SysMain.h"

/*********************************************************************************************/
SYS_ERR_TYPEDEF gSysErr;
SYS_ERR_TYPEDEF gSysErcEn;
SYS_ERR_TYPEDEF gSysErrStop;
/*********************************************************************************************/

/*********************************************************************************************/
void gSysErrVarInit(void)
{
	memset_16(&gSysErr.Index,0,sizeof_16(SYS_ERR_TYPEDEF));

	memset_16(&gSysErcEn.Index,0,sizeof_16(SYS_ERR_TYPEDEF));

	memset_16(&gSysErrStop.Index,0,sizeof_16(SYS_ERR_TYPEDEF));

	//----------------------------------------------------------
	//置位，说明对应的报警号不可以清楚
	gSysErcEn.ErrHd.bit.CurSensorError1 = 1;
	gSysErcEn.ErrHd.bit.CurSensorError2 = 1;
	gSysErcEn.ErrHd.bit.PowerUpErr = 1;
	gSysErcEn.ErrHd.bit.EPROMError = 1;
	gSysErcEn.ErrHd.bit.CpldError = 1;
	gSysErcEn.ErrHd.bit.VrefErrV = 1;
	gSysErcEn.ErrHd.bit.VrefErrT = 1;

	gSysErcEn.ErrCtl.bit.EncErrUVW = 1;
	gSysErcEn.ErrCtl.bit.EncErr_ABZ = 1;
	gSysErcEn.ErrCtl.bit.MotoWireLose = 1;
	gSysErcEn.ErrCtl.bit.SysParErr = 1;
	//----------------------------------------------------------

	//----------------------------------------------------------
	gSysErrStop.ErrHd.all = ~gSysErrStop.ErrHd.all;
	gSysErrStop.ErrCtl.all = ~gSysErrStop.ErrCtl.all;

	gSysErrStop.ErrEmg.bit.SEmgsErr = TRUE;
	gSysErrStop.ErrEmg.bit.PosLmtL = TRUE;
	gSysErrStop.ErrEmg.bit.PosLmtR = TRUE;
	gSysErrStop.ErrEmg.all = ~gSysErrStop.ErrEmg.all;

	gSysErrStop.ErrCom.bit.COMErr = TRUE;
	gSysErrStop.ErrCom.bit.ComOvt = TRUE;
	gSysErrStop.ErrCom.all = ~gSysErrStop.ErrCom.all;
	//----------------------------------------------------------
}
/*********************************************************************************************/

/*********************************************************************************************/
u16 gSysErrGetId(void)
{
	u16 i=0;

	if( !gSysErr.ErrHd.all && !gSysErr.ErrCtl.all && !gSysErr.ErrEmg.all && !gSysErr.ErrCom.all){
		return 0;
	}
	for(i=0;i<32;i++)
	{
		if((gSysErr.ErrHd.all>>i)&0x0001)
		{
			return (i+1);
		}
	}
	for(i=0;i<32;i++)
	{
		if((gSysErr.ErrCtl.all>>i)&0x0001)
		{
			return (100+i+1);
		}
	}
	for(i=0;i<16;i++)
	{
		if((gSysErr.ErrEmg.all>>i)&0x0001)
		{
			return (200+i+1);
		}
	}

	for(i=0;i<16;i++)
	{
		if((gSysErr.ErrCom.all>>i)&0x0001)
		{
			return (300+i+1);
		}
	}
	return 0;
}
/*********************************************************************************************/


/*********************************************************************************************/
void gSysErrClear(u16 ErrId)
{
	u16 temp = 0;
	u16 no = 0;
	if( !gSysErr.ErrHd.all && !gSysErr.ErrCtl.all && !gSysErr.ErrEmg.all && !gSysErr.ErrCom.all){
		return;
	}
	temp = ErrId/100;
	no =  ErrId%100;
	if(no==0) return;
	no = no-1;
	if(temp == 0)
	{
		if((gSysErcEn.ErrHd.all & (0x01L<<no)) == 0)
		{
			gSysErr.ErrHd.all = gSysErr.ErrHd.all & (~(0x01L<<no));
		}
	}else if(temp == 1)
	{
		if((gSysErcEn.ErrCtl.all & (0x01L<<no)) == 0)
		{
			gSysErr.ErrCtl.all = gSysErr.ErrCtl.all & (~(0x01L<<no));
		}
	}else if(temp == 2)
	{
		if((gSysErcEn.ErrEmg.all & (0x01L<<no)) == 0)
		{
			gSysErr.ErrEmg.all = gSysErr.ErrEmg.all & (~(0x01L<<no));
		}
	}else{
		if((gSysErcEn.ErrCom.all & (0x01L<<no)) == 0)
		{
			gSysErr.ErrCom.all = gSysErr.ErrCom.all & (~(0x01L<<no));
		}
	}
}
/*********************************************************************************************/


/*********************************************************************************************/
void gSysErrClearAll(void)
{
	gSysErr.ErrHd.all &= gSysErcEn.ErrHd.all;
	gSysErr.ErrCtl.all &= gSysErcEn.ErrCtl.all;
	gSysErr.ErrEmg.all &= gSysErcEn.ErrEmg.all;
	gSysErr.ErrCom.all &= gSysErcEn.ErrCom.all;
}
/*********************************************************************************************/


