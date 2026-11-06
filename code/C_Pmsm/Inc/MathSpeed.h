/*
 * MathSpeedNew.h
 *
 *  Created on: 2016413
 *      Author: admin
 */

#ifndef MATHSPEED_H_
#define MATHSPEED_H_

#include "Bsp.h"

#define SPD_FILTER_RATIO_LEN_MAX 4

typedef struct {
    	Uint16 	Index;
    	Uint16  Ticker;
    	Uint16  FilterLen;
        float   Kr_L1[SPD_FILTER_RATIO_LEN_MAX];	//
        float   Kr_L2[SPD_FILTER_RATIO_LEN_MAX];	//
        float   Kr_L3[SPD_FILTER_RATIO_LEN_MAX];	//
        float   Kr_L4[SPD_FILTER_RATIO_LEN_MAX];	//
        float   Kr_L5[SPD_FILTER_RATIO_LEN_MAX];	//
        float   Kr_L6[SPD_FILTER_RATIO_LEN_MAX];	//
        float   Kr_L7[SPD_FILTER_RATIO_LEN_MAX];	//
        float   Kr_L8[SPD_FILTER_RATIO_LEN_MAX];	//
        float  	SpdAry[SPD_FILTER_RATIO_LEN_MAX];	//
       Uint32 	DirectionQep; 			// 
       Uint32 	BaseRpm;      			// 
       int32 	MechTheta;  			// 
       int32	OldMechTheta;   		// 
       float	Speed;       			// -
       float 	Ks;         			// 
       float 	Tmp;					//
       int32	temp32;
       Uint16 	FirstIndexFlag;			//Z
       Uint16 	Flag;					//Z
       int32  	FirstIndexMechTheta;	//
       float 	SpeedRpm;      			//     RPM
       float    SpeedFiterSum;
       float 	SpeedFiterRpm;
       int32 	CircleLine;
       float	MechScaler;
	   
       } SPEED_MEAS_QEPF;   			//

extern SPEED_MEAS_QEPF gSpeed;
void SPEED_INIT_ALL(void);


#define SPEED_MACRO_ALL(v)																	\
																							\
	/************************************************************************************/	\
	if((v.FirstIndexFlag == 1) && (v.Flag == 0))											\
	{																						\
		v.OldMechTheta -= v.FirstIndexMechTheta;											\
		if(v.OldMechTheta > v.CircleLine)													\
			v.OldMechTheta -= v.CircleLine;												\
		else if(v.OldMechTheta < 0.0f)														\
			v.OldMechTheta += v.CircleLine;												\
		v.Flag = 1;																			\
	}																						\
	/************************************************************************************/	\
																							\
	v.temp32 = v.MechTheta - v.OldMechTheta;													\
																							\
	if(v.temp32 < -v.CircleLine/2)															\
		v.temp32 = v.temp32 + v.CircleLine;														\
	else if (v.temp32 > v.CircleLine/2)														\
		v.temp32 = v.temp32 - v.CircleLine;														\
																							\
	v.Tmp = v.Ks*v.temp32*v.MechScaler;																		\
																							\
	v.SpdAry[0] = v.SpdAry[1];																\
	v.SpdAry[1] = v.SpdAry[2];																\
	v.SpdAry[2] = v.SpdAry[3];																\
	v.SpdAry[3] = v.Tmp;																	\
																							\
	switch(v.Index)																			\
	{																						\
	case 0:																					\
		v.Tmp = v.SpdAry[0]*v.Kr_L1[0] + v.SpdAry[1]*v.Kr_L1[1] + v.SpdAry[2]*v.Kr_L1[2] + v.SpdAry[3]*v.Kr_L1[3];				\
		if(v.Tmp > 0.02f)																	\
		{																					\
			v.Index = 1;																	\
		}																					\
		break;																				\
	case 1:																					\
		v.Tmp = v.SpdAry[0]*v.Kr_L2[0] + v.SpdAry[1]*v.Kr_L2[1] +v.SpdAry[2]*v.Kr_L2[2] + v.SpdAry[3]*v.Kr_L2[3];	\
		if(v.Tmp < 0.016f)																	\
		{																					\
			v.Index = 0;																	\
		}else if(v.Tmp > 0.1f)																\
		{																					\
			v.Index = 2;																	\
		}																					\
		break;																				\
	case 2:																					\
		v.Tmp = v.SpdAry[0]*v.Kr_L3[0] + v.SpdAry[1]*v.Kr_L3[1] + v.SpdAry[2]*v.Kr_L3[2] + v.SpdAry[3]*v.Kr_L3[3];	\
		if(v.Tmp < 0.08f)																	\
		{																					\
			v.Index = 1;																	\
		}else if(v.Tmp > 0.6f)																\
		{																					\
			v.Index = 3;																	\
		}																					\
		break;																				\
	case 3:																					\
		v.Tmp = v.SpdAry[0]*v.Kr_L4[0] + v.SpdAry[1]*v.Kr_L4[1] + v.SpdAry[2]*v.Kr_L4[2] + v.SpdAry[3]*v.Kr_L4[3];	\
		if(v.Tmp < 0.5f)																		\
		{																					\
			v.Index = 2;																	\
		}																					\
		break;																				\
	default:																				\
		v.Tmp = v.SpdAry[0]*v.Kr_L1[0] + v.SpdAry[1]*v.Kr_L1[1] +	v.SpdAry[2]*v.Kr_L1[2] + v.SpdAry[3]*v.Kr_L1[3];	\
		v.Index = 0;																		\
		break;																				\
	}																						\
	if(v.Tmp > 2.1f) v.Tmp = 2.1f;															\
	else if(v.Tmp < -2.1f) v.Tmp = -2.1f;														\
	v.Ticker++;																				\
	v.SpeedFiterSum += v.Tmp;																\
	if(v.Ticker >= 1000)																	\
	{																						\
		v.SpeedFiterRpm = (v.SpeedFiterSum*0.001f)*v.BaseRpm;								\
		v.SpeedFiterSum = 0;																\
		v.Ticker = 0;																		\
	}																						\
																							\
	v.Speed = v.Tmp;																		\
	v.OldMechTheta = v.MechTheta;															\
	v.SpeedRpm = v.BaseRpm * v.Speed;														\








#endif /* MATHSPEEDNEW_H_ */
