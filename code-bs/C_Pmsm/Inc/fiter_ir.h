/*
 * fiter_ir.h
 *
 *  Created on: 2016Äê7ÔÂ14ÈÕ
 *      Author: renbin
 */

#ifndef FITER_IR_H_
#define FITER_IR_H_

typedef struct {
	float data_0;
	float data_1;
	float data_2;
	float data_3;
	float data_4;
	float Kn_0;
	float Kn_1;
	float Kn_2;
	float Kn_3;
	float Kn_4;
	float Out;
}FlterIrTypedef;



#define FlterIr_CLA_MACRO(v)																		\
	v.Out = v.data_0 * v.Kn_0 + v.data_1 * v.Kn_1 + v.data_2 * v.Kn_2 + v.data_3 * v.Kn_3 + v.data_4 * v.Kn_4;	\
	v.data_4 = v.data_3;																			\
	v.data_3 = v.data_2;																			\
	v.data_2 = v.data_1;																			\
	v.data_1 = v.data_0;


#endif /* FITER_IR_H_ */
