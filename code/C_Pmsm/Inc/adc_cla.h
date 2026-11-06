/*
 * adc_cla.h
 *
 *  Created on: 2016Äê4ÔÂ25ÈÕ
 *      Author: admin
 */

#ifndef ADC_CLA_H_
#define ADC_CLA_H_


typedef struct{
	float 	iReg;
	float 	tTemp;
	float 	Gain;
	float 	Offset;
	float 	Out;
}ADC_CLA;

#define ADC_CLA_MACRO(v) 				\
	v.tTemp =  v.iReg  - v.Offset;		\
	v.Out = v.tTemp * v.Gain;

#endif /* ADC_CLA_H_ */
