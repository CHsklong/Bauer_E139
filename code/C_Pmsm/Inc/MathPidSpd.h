/*
 * MathPidSpd.h
 *
 *  Created on: 2015318
 *      Author: admin
 */

#ifndef MATHPIDSPD_H_
#define MATHPIDSPD_H_

#include "Bsp.h"



typedef struct {  		float  Ref;   			// Input: Reference input
						float  Fbk;   			// Input: Feedback input
						float  Err;				// Variable: Error
						float  Kp;				// Parameter: Proportional gain
						float  Up;				// Variable: Proportional output
						float  ui;				// Variable: Integral output
						float  OutPreSat; 		// Variable: Pre-saturated output
						float  Umax;		    // Parameter: Maximum output
						float  Umin;	    	// Parameter: Minimum output
						float  Out;   			// Output: PID output
						float  Ki;			    // Parameter: Integral gain
						float  uKi;
						float  temp;
						float  Kc;		     	// Parameter: Integral correction gain
						float  InterThd;		// 
						u16    TorLmtFlag;		//
					} PI_CONTROLLER_SPD;

extern PI_CONTROLLER_SPD gPidSpd;



void PI_SPD_INIT(void);


#define PI_SPD_MACRO(v)											\
																\
	v.Err = v.Ref - v.Fbk;										\
	 										\
	v.Up =  v.Kp * v.Err ;										\
	v.temp = fabs(v.Err);										\
	if(v.temp <= 0.003) v.uKi = 1.0;							\
	else if(v.temp <= 0.01) v.uKi = 0.8;						\
	else if(v.temp <= 0.05) v.uKi = 0.6;						\
	else if(v.temp <= 0.1) v.uKi = 0.3;						    \
	else if(v.temp < v.InterThd) v.uKi = 0.1;					\
	else{v.uKi = 0; v.ui = 0;}									\
	v.ui = v.ui +  v.uKi*v.Ki * v.Up;							\
	if(v.ui > v.Umax)											\
	{	v.ui = v.Umax ;}										\
	else if(v.ui < v.Umin)										\
	{	v.ui = v.Umin ;}										\
	 										\
	v.OutPreSat = v.Up + v.ui;									\
	if (v.OutPreSat > v.Umax)             						\
	{															\
		v.Out =  v.Umax;										\
		v.TorLmtFlag = TRUE;									\
	}															\
	else if (v.OutPreSat < v.Umin)								\
	{															\
		v.Out =  v.Umin; 										\
		v.TorLmtFlag = TRUE;									\
	} 															\
	else														\
	{															\
		v.Out = v.OutPreSat;  									\
		v.TorLmtFlag = FALSE;									\
	}                 					  						\


#endif /* MATHPIDSPD_H_ */
