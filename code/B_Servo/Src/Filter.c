

#include "SysMain.h"


FILTER_TYPE_1 gFilter1;
FILTER_TYPE_1 gFilterId,gFilterIq, gFilterIU,gFilterIV;




void filter_K_cal(FILTER_TYPE_1 *dp)
{
	float	wpt;
	//
/*	
	if(SysParP3.NCF1 == 0)
	{
		dp->Km_1 = 0.0;
		dp->Kn = 1.0;

	}else{
}
*/	
	wpt = 0.2f*PI*SysParP5.CMDFT *T_PWM*SPEED_LOOP_PSR;
		dp->Kn = wpt/(2+wpt);
		dp->Km_1 = 1-dp->Kn-dp->Kn;


	
}


void filter_K_idiq(FILTER_TYPE_1 *dp)
{
	float	wpt;
	//


		wpt = 0.2f*PI*SysParP5.UVFT *T_PWM;
		dp->Kn = wpt/(2+wpt);
		dp->Km_1 = 1-dp->Kn-dp->Kn;


}

ATTR_RAMFUNC void filter_step1(FILTER_TYPE_1 *ptr)
{
	ptr->Yn = (ptr->Xn + ptr->Xn_1) * ptr->Kn + ptr->Yn * ptr->Km_1 ;
	ptr->Xn_1 = ptr->Xn ;
	ptr->Yn_1 = ptr->Yn ;
}


void filter_K_gan(FILTER_TYPE_1 *dp)
{



}
