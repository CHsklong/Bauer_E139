/*
 * dtc_f.h
 *
 *  Created on: 2016Äê6ÔÂ23ÈÕ
 *      Author: Aaron
 */

#ifndef DTC_F_H_
#define DTC_F_H_

typedef struct{
	float A;
	float B;
	float C;
	float alph;
	float beta;
	float SignA;
	float SignB;
	float SignC;
}DTC_F;

#define DTC_MACRO(v)	\
		v.A = v.alph;	\
		v.B = 0.8660254 * v.beta - 0.5*v.alph;		\
		v.C = 0.0-v.A -v.B;							\
		if(v.A > 0.0) v.SignA = 1;					\
		else if(v.A < -0.0)	v.SignA = -1;		\
		else	v.SignA = 0;						\
		if(v.B > 0.0) v.SignB = 1;					\
		else if(v.B < -0.0)	v.SignB = -1;		\
		else v.SignB = 0;							\
		if(v.C > 0.0) v.SignC = 1;					\
		else if(v.C < -0.0)	v.SignC = -1;		\
		else v.SignC = 0;

#endif /* DTC_F_H_ */
