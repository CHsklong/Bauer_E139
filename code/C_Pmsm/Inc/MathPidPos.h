/*
 * MathPidPos.h
 *
 *  Created on: 2015年3月18日
 *      Author: admin
 */

#ifndef MATHPIDPOS_H_
#define MATHPIDPOS_H_

#include "Bsp.h"


typedef struct {  	s32    Ref;   		// Input: Reference input
					s32    Fbk;   		// Input: Feedback input
					s32    CirLine;		//圈数
					float  Err;			// Variable: Error
					float  Kp;			// Parameter: Proportional gain
					float  Up;			// Variable: Proportional output
					float  OutPreSat; 	// Variable: Pre-saturated output
					float  Umax;		// Parameter: Maximum output
					float  Umin;	    // Parameter: Minimum output
					float  Out;   		// Output: PID output
					float  VfGain;		//位置速度前馈增益
					float  Inc;			//位置增量
					float  KVf;			//前馈增量，速度系数
					float  VfOut;		//前馈，速度增量输出
					float  SpdGn;		//前馈，速度给出
					float  VfTemp;		//前馈速度 变量
					u16    IndexFlag;
					float  PFlt;		//位置前馈时间平滑常数
					float  Tmp;
					float  QepDline;
				} PI_POS;

extern PI_POS gPidPos;



void PI_POS_INIT(void);


#define PI_POS_MACRO(v)																		\
	v.SpdGn = v.Inc * v.KVf;																\
	v.VfTemp =  v.SpdGn * v.VfGain;															\
																							\
	v.Tmp = v.VfTemp - v.VfOut;																\
																							\
	if(v.Tmp > v.PFlt) v.VfOut = v.VfOut + v.PFlt;											\
	else if(v.Tmp < -v.PFlt) v.VfOut = v.VfOut - v.PFlt;									\
	else v.VfOut = v.VfTemp;																\
																							\
	v.Err = (float)(v.Ref - v.Fbk)*v.QepDline;												\
	if(v.Err > ENCODER_CIRCLE/2) v.Err = v.Err - ENCODER_CIRCLE;							\
	else if(v.Err < -ENCODER_CIRCLE/2) v.Err = v.Err + ENCODER_CIRCLE;						\
																							\
 	v.Up = v.Kp * v.Err;																	\
 																							\
	v.OutPreSat = v.Up + v.VfOut;															\
																							\
	if(v.OutPreSat > v.Umax){																\
		v.Out = v.Umax;																		\
	}else if(v.OutPreSat < v.Umin){															\
		v.OutPreSat = v.Umin;																\
	}else{																					\
		v.Out = v.OutPreSat;																\
	}																						\



#endif /* MATHPIDPOS_H_ */
