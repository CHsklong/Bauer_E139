/*
 * MathFilterFuzzy.c
 *
 *  Created on: 2017Äê12ÔÂ28ÈÕ
 *      Author: admin
 */


#include "MathFilterFuzzy.h"


float LowPasFilterFuzzyMacr(float err,float par)
{
	float tmpf = 0.0f;
	float terr = fabs(err);
	if(terr<0.01f) tmpf = par;
	else if(terr<0.02f) tmpf = 0.5f*par;
	else if(terr<0.04f) tmpf = 0.2f*par;
	else if(terr<0.1f) tmpf = 0.1f*par;
	else tmpf = 0.0f;
	return tmpf;
}


