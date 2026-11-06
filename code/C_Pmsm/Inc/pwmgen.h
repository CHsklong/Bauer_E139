/*
 * pwmgen.h
 *
 *  Created on: 2015Äê12ÔÂ7ÈÕ
 *      Author: admin
 */

#ifndef PWMGEN_H_
#define PWMGEN_H_

/****************************************************************************************************************************/
typedef struct {
	float HalfPerMax;     // Parameter: Half of PeriodMax
	float MfuncC1;        // Input: EPWM1 A&B Duty cycle ratio
	float MfuncC2;        // Input: EPWM2 A&B Duty cycle ratio
	float MfuncC3;        // Input: EPWM3 A&B Duty cycle ratio
	float CMP1;
	float CMP2;
	float CMP3;
	float DTIME;
}PWMGEN_CLA ;

#define  PWMGEN_CLA_MACRO(v)							\
														\
	v.CMP1 = v.MfuncC1 * v.HalfPerMax +  v.HalfPerMax;	\
	v.CMP2 = v.MfuncC2 * v.HalfPerMax +  v.HalfPerMax;	\
	v.CMP3 = v.MfuncC3 * v.HalfPerMax +  v.HalfPerMax;




#endif /* PWMGEN_H_ */
