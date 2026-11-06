/*
 * MathVoltCal.h
 *
 *  Created on: 2014-9-17
 *      Author: Administrator
 */

#ifndef MATHVOLTCAL_H_
#define MATHVOLTCAL_H_

/* =================================================================================
File name:        VOLT_CAL.H
===================================================================================*/


#ifndef __VOLT_CAL_H__
#define __VOLT_CAL_H__

#include "Bsp.h"

typedef struct 	{ 	float  DcBusVolt;		// Input: DC-bus voltage (pu)
					float  MfuncV1;  		// Input: Modulation voltage phase A (pu)
					float  MfuncV2;			// Input: Modulation voltage phase B (pu)
					float  MfuncV3;			// Input: Modulation voltage phase C (pu)
					Uint16  OutOfPhase; 	// Parameter: Out of Phase adjustment (0 or 1) (Q0) - independently with global Q
					float  VphaseA;			// Output: Phase voltage phase A (pu)
					float  VphaseB;			// Output: Phase voltage phase B (pu)
					float  VphaseC;			// Output: Phase voltage phase C (pu)
					float  Valpha;			// Output: Stationary d-axis phase voltage (pu)
					float  Vbeta;  			// Output: Stationary q-axis phase voltage (pu)
					float  temp;				// Variable: temp variable
		  	  	} PHASEVOLTAGE;

extern PHASEVOLTAGE PaseVolt;
void PaseVoltInit(void);
/*
OutOfPhase = 1 for the out of phase correction if
* MfuncV1 is out of phase with PWM1,
* MfuncV2 is out of phase with PWM3,
* MfuncV3 is out of phase with PWM5,
otherwise, set 0 if their phases are correct.
*/

/*-----------------------------------------------------------------------------
Default initalizer for the PHASEVOLTAGE object.
-----------------------------------------------------------------------------*/

#define PHASEVOLTAGE_DEFAULTS { 0, \
                          		0, \
                          		0, \
                          		0, \
                          		1, \
                          		0, \
                          		0, \
                          		0, \
															0, \
															0, \
															}

#define ONE_THIRD  0.33333333333333f
#define TWO_THIRD  0.66666666666667f
#define INV_SQRT3  0.57735026918963f
/*------------------------------------------------------------------------------
 	Phase Voltage Calculation Macro Definition
------------------------------------------------------------------------------*/


#define PHASEVOLT_MACRO(v)														\
																				\
																				\
/* Scale the incomming Modulation functions with the DC bus voltage value*/		\
/* and calculate the 3 Phase voltages */										\
  v.temp 	  = v.DcBusVolt*ONE_THIRD;											\
  v.VphaseA   = v.temp*(2*v.MfuncV1-v.MfuncV2-v.MfuncV3);						\
  v.VphaseB   = v.temp*(2*v.MfuncV2-v.MfuncV1-v.MfuncV3);						\
  v.VphaseC   = v.temp*(2*v.MfuncV3-v.MfuncV2-v.MfuncV1);						\
																				\
/* Voltage transformation (a,b,c)  ->  (Alpha,Beta)	*/							\
  v.Valpha = v.VphaseA;															\
  v.Vbeta = (v.VphaseA + 2 * v.VphaseB)*INV_SQRT3;


#endif // __VOLT_CAL_H__

// Phase C (if needed)
// v.VphaseC   = _IQmpy(v.temp,(_IQmpy2(v.MfuncV3)-v.MfuncV2-v.MfuncV1));	


























#endif /* MATHVOLTCAL_H_ */
