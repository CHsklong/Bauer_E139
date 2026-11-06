/*
 * clarke_f.h
 *
 *  Created on: 2013-9-29
 *      Author: Administrator
 */

#ifndef CLARKE_F_H_
#define CLARKE_F_H_

// structure definition for DMC CLA blocks
typedef struct {
				  float  As;			// Input : a
				  float  Bs;		    // Input : b
				  float  alpha; 		// Output: alpha
				  float  beta;		    // Output: beta
				}CLARKE_CLA;

#define CLARKE_CLA_MACRO(v)														\
	v.alpha=v.As;																\
	v.beta =(v.As + v.Bs*2)*0.57735026918f;

#define CLARKE_CLA_INIT(v)														\
	v.alpha=0.0;																\
	v.beta =0.0;																\
	v.As=0.0;																	\
	v.Bs=0.0;


#endif /* CLARKE_F_H_ */
