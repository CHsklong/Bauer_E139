/*
 * MathSpeedNew.c
 *
 *  Created on: 2016年4月13日
 *      Author: admin
 */

#include "SysMain.h"

SPEED_MEAS_QEPF gSpeed;

#define SPEED_FILTER_LEN 4

#if 0
const float SPEED_FILTER_RATIO8_L1[SPEED_FILTER_LEN] = {0.25  ,0.25 ,0.25 ,0.25 };//小于60RPM     10K
const float SPEED_FILTER_RATIO8_L2[SPEED_FILTER_LEN] = {0.0   ,0.0  ,0.0  ,1.0  };//60-300		10K-50K
const float SPEED_FILTER_RATIO8_L3[SPEED_FILTER_LEN] = {0.0   ,0.0  ,0.0  ,1.0  };//300-1800		50K-300K
const float SPEED_FILTER_RATIO8_L4[SPEED_FILTER_LEN] = {0.0   ,0.0  ,0.0  ,1.0  };//1800			300K

#else

const float SPEED_FILTER_RATIO8_L1[SPEED_FILTER_LEN] = {0.25  ,0.25, 0.25, 0.25 };//小于60RPM     10K
const float SPEED_FILTER_RATIO8_L2[SPEED_FILTER_LEN] = {0.25  ,0.25 ,0.25 ,0.25 };//60-300		10K-50K
const float SPEED_FILTER_RATIO8_L3[SPEED_FILTER_LEN] = {0.25  ,0.25 ,0.25 ,0.25 };//300-1800		50K-300K
const float SPEED_FILTER_RATIO8_L4[SPEED_FILTER_LEN] = {0.25  ,0.25 ,0.25 ,0.25 };//1800			300K
#endif

void SPEED_INIT_ALL(void)
{
	memset_16(&gSpeed.Index,0,sizeof_16(SPEED_MEAS_QEPF));

	gSpeed.Ks = 1.0f/(MECH_FREQ * SPEED_LOOP_PSR * T_PWM);
	gSpeed.BaseRpm = BASE_SPD_RPM;
	gSpeed.FilterLen = SPEED_FILTER_LEN;
	if(SysParP8.ENCT < ENC_ABS_17BIT){	
		gSpeed.CircleLine = ENCODER_CIRCLE *(MotoPar.ENCL*4);	// 2500
	}else{
		gSpeed.CircleLine = (MotoPar.ENCL*4);			//17bit
	}
	gSpeed.MechScaler = (1.0/(MotoPar.ENCL*4));
	memcpy_16(&gSpeed.Kr_L1[0],SPEED_FILTER_RATIO8_L1,sizeof_16(SPEED_FILTER_RATIO8_L1));
	memcpy_16(&gSpeed.Kr_L2[0],SPEED_FILTER_RATIO8_L2,sizeof_16(SPEED_FILTER_RATIO8_L2));
	memcpy_16(&gSpeed.Kr_L3[0],SPEED_FILTER_RATIO8_L3,sizeof_16(SPEED_FILTER_RATIO8_L3));
	memcpy_16(&gSpeed.Kr_L4[0],SPEED_FILTER_RATIO8_L4,sizeof_16(SPEED_FILTER_RATIO8_L4));
}


