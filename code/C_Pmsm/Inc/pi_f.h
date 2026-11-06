/*
 * pi_f.h
 *
 *  Created on: 2013-9-29
 *      Author: Administrator
 */

#ifndef PI_F_H_
#define PI_F_H_


	typedef struct {  	float  Ref;   			// Input: Reference input
						float  Fbk;   			// Input: Feedback input
						float  Err;				// Variable: Error
						float  Kp;				// Parameter: Proportional gain
						float  Up;				// Variable: Proportional output
						float  ui;				// Variable: Integral output
						float  OutPreSat; 		// Variable: Pre-saturated output
						float  Umax;		    // Parameter: Maximum output
						float  Umin;	    	// Parameter: Minimum output
						float  Out;   			// Output: PID output
						float  SatErr;			// Variable: Saturated difference
						float  Ki;			    // Parameter: Integral gain
						float  Kc;		     	// Parameter: Integral correction gain
						float  uKi;
						float  tmp;
						float  VbusRatio;
						float  Emf;
					} PI_CLA;



#define PI_CLA_MACRO(v)											\
																\
	/* Compute the error:E(k)=ref(k)-fdb(k)	*/					\
	v.Err = v.Ref - v.Fbk;										\
	v.tmp = v.Err;												\
	if(v.tmp < 0.0f) v.tmp = -v.tmp;								\
	/* integral term */ 										\
	v.Up = v.Kp*v.Err;											\
	if(v.tmp >= 1.0f) {v.uKi = 0.0f;}								\
	else{														\
		v.uKi = (1.0f - v.tmp)*(1.0f - v.tmp)*v.Ki;				\
		/*v.uKi = v.Ki;	*/										\
	}															\
	v.ui = v.ui + v.uKi*v.Up;/*+ 0*v.uKi*v.SatErr;*/			\
	/* control output */ 										\
	v.OutPreSat = v.Up + v.ui;									\
	v.OutPreSat += v.Emf;										\
	v.OutPreSat = v.OutPreSat*v.VbusRatio;						\
/* Compute the saturate difference:Usat(k)=Uout(k)-Upresat(k)*/	\
	if(v.OutPreSat > v.Umax){v.Out = v.Umax;}					\
	else if(v.OutPreSat < v.Umin){v.Out = v.Umin;}				\
	else {v.Out = v.OutPreSat;}									\
	if(v.ui > v.Umax){v.ui = v.Umax;}							\
	else if(v.ui < v.Umin){	v.ui = v.Umin;}						\
	/*v.Out=__mmaxf32(__mminf32(v.OutPreSat, v.Umax),v.Umin);*/	\
	/*v.ui=__mmaxf32(__mminf32(v.ui, v.Umax),v.Umin);*/			\
	v.SatErr = v.Out - v.OutPreSat;



#endif /* PI_F_H_ */
