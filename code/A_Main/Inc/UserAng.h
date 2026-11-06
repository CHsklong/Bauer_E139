/*
 * UserAng.h
 *
 *  Created on: 2015313
 *      Author: admin
 */

#ifndef USERANG_H_
#define USERANG_H_

#include "mcu.h"


#define CUR_ZERO_OFFSET_MAX 2150
#define CUR_ZERO_OFFSET_MIN 1945

#if POW_VER_STS == POW_VER_220
#define BORD_INFO_SCALE1 	(4096*0.2/3.3)
#define BORD_INFO_SCALE2 	(4096*0.4/3.3)
#define BORD_INFO_SCALE3 	(4096*0.6/3.3)
#define BORD_INFO_SCALE4 	(4096*0.8/3.3)
#define BORD_INFO_SCALE5 	(4096*1.0/3.3)
#define BORD_INFO_SCALE6 	(4096*1.2/3.3)
#define BORD_INFO_SCALE7 	(4096*1.4/3.3)
#define BORD_INFO_SCALE8 	(4096*1.6/3.3)
#define BORD_INFO_SCALE9 	(4096*1.8/3.3)
#define BORD_INFO_SCALE10 	(4096*2.0/3.3)

#else
#define BORD_INFO_SCALE11 	(4096*1.6/3.3)
#define BORD_INFO_SCALE12 	(4096*1.8/3.3)
#define BORD_INFO_SCALE13 	(4096*2.0/3.3)
#define BORD_INFO_SCALE14 	(4096*2.2/3.3)
#define BORD_INFO_SCALE15 	(4096*2.4/3.3)
#define BORD_INFO_SCALE16 	(4096*2.6/3.3)
#define BORD_INFO_SCALE17 	(4096*2.8/3.3)
#define BORD_INFO_SCALE18 	(4096*3.0/3.3)
#define BORD_INFO_SCALE19 	(4096*3.2/3.3)
#define BORD_INFO_SCALE20 	(4096*3.3/3.3)
#endif

#if  POW_VER_STS == POW_VER_220
	#define VOLT_BUS_HD_GAIN 205		//
#elif  POW_VER_STS == POW_VER_24V
	#define VOLT_BUS_HD_GAIN 21.59		//
#else
	#define VOLT_BUS_HD_GAIN 355		//
#endif



#define EX_REF_ANG_ZERO_VOLT 1.65f	//1.65V
#define EX_REF_ANG_MAX_VOLT 1.5f	//1.5V
#define EX_REF_ANG_ZERO_SCALE (4096/3.3f)
#define EX_REF_ANG_ZERO_DIG (EX_REF_ANG_ZERO_VOLT*EX_REF_ANG_ZERO_SCALE)
typedef struct{
	s16 	iReg;
	s16 	tTemp;
	float 	Kn_1;
	float	Kn;
	float 	LpKn_1;
	float	LpKn;
	float 	Gain;
	s16 	Offset;
	float 	Out;
	float	OutLp;
	u16		ErrVolt;
}AdcChannelTypeDef;

extern AdcChannelTypeDef AngVar_Vbus;

extern float AngOffSet_U;
extern float AngOffSet_V;
extern s16 AngBordInfo;

#define ADC_CUR_MACRO(v) 				\
	v.tTemp =  v.iReg  - v.Offset;		\
	v.Out = v.tTemp * v.Gain;


void ADC_LP_MACRO(AdcChannelTypeDef *dp);
#if 0
#define ADC_LP_MACRO(v)								\
		v.tTemp =  v.iReg  - v.Offset;				\
		v.tTemp =  v.tTemp * v.Gain ;				\
		v.Out =  v.Out*v.Kn_1  +  v.tTemp*v.Kn;
#endif

void ADC_CHANNEL_INIT(void);
void ADC_CHANNEL_MACRO(AdcChannelTypeDef *dp);
void ADC_OFFSET_MACRO(AdcChannelTypeDef *dp);

#endif /* USERANG_H_ */
