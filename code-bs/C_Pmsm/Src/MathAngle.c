/*
 * MathAngleNew.c
 *
 *  Created on: 2016413
 *      Author: admin
 */

#include "SysMain.h"

QEP_F gQepAngle;


/*************************************************************************/
void ANGLE_INIT_ALL(void)
{
	memset_16(&gQepAngle.ElecTheta,0,sizeof_16(QEP_F));

	
	gQepAngle.QepType = (MotoPar.ENCT%100);
	if(gQepAngle.QepType >= ENC_ABS_17BIT){
		gQepAngle.CalibratedAngle = AbsPar.CalibratedAngle ;
		

	}else{
	
		//MotoPar.EZRO = -150;
		MotoPar.POLES = 8;
		MotoPar.ENCL = 1024;
		gQepAngle.CalibratedAngle = MotoPar.EZRO;
		
	}
	
	gQepAngle.LineEncoder = MotoPar.ENCL;
	gQepAngle.ReferenceLine =  (MotoPar.ENCL*4);
	gQepAngle.CircleLine = ENCODER_CIRCLE*gQepAngle.ReferenceLine;
	gQepAngle.MechScaler = (1.0f/gQepAngle.ReferenceLine);	
	gQepAngle.PolePairs = MotoPar.POLES/2;
	
}
/*************************************************************************/


/*************************************************************************/
ATTR_RAMFUNC void ANGLE_MACRO_ALL(u32 AbsQep,s32 AbsQepInc)

{

	float Tempf = 0.0f;
	Uint32 TempI = 0;
	int32 temps = 0;
	if(gQepAngle.QepType >= ENC_ABS_17BIT)
	{
		gQepAngle.RawTheta = AbsQep + gQepAngle.CalibratedAngle;
		if(gQepAngle.RawTheta < 0)
			gQepAngle.RawTheta += gQepAngle.ReferenceLine;
		else if(gQepAngle.RawTheta > gQepAngle.ReferenceLine)
			gQepAngle.RawTheta -= gQepAngle.ReferenceLine;


		gQepAngle.MechTheta = gQepAngle.RawTheta;

		Tempf = gQepAngle.MechScaler * (float)gQepAngle.RawTheta;
		Tempf = (float)gQepAngle.PolePairs * Tempf;
		TempI = (Uint32)Tempf;

		gQepAngle.ElecTheta = Tempf - TempI;

	//------------------------------------------------------------------------
		temps = AbsQep + gQepAngle.CalibratedAngle + AbsQepInc;
		if(temps < 0)
			temps += gQepAngle.ReferenceLine;
		else if(temps > gQepAngle.ReferenceLine)
			temps -= gQepAngle.ReferenceLine;

		Tempf = gQepAngle.MechScaler * (float)temps;
		Tempf = (float)gQepAngle.PolePairs * Tempf;
		TempI = (Uint32)Tempf;

		gQepAngle.ElecThetaN = Tempf - TempI;
	//-----------------------------------------------------------------------------
		temps = AbsQep + gQepAngle.CalibratedAngle + 2*AbsQepInc;
		if(temps < 0)
			temps += gQepAngle.ReferenceLine;
		else if(temps > gQepAngle.ReferenceLine)
			temps -= gQepAngle.ReferenceLine;

		Tempf = gQepAngle.MechScaler * (float)temps;
		Tempf = (float)gQepAngle.PolePairs * Tempf;
		TempI = (Uint32)Tempf;

		gQepAngle.ElecThetaN1 = Tempf - TempI;
	//-------------------------------------------------------------------------------
		return;
	}
	
    
}
/*************************************************************************/

