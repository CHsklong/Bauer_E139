/*
 * MathOverLoad.h
 *
 *  Created on: 2016年5月16日
 *      Author: admin
 */

#ifndef MATHOVERLOAD_H_
#define MATHOVERLOAD_H_

#include "Bsp.h"

#define OVER_LOAD_THD_SCALE (2*2*2*2 *1000L * ISR_FREQUENCY)

/********************************************************************************************************************/
//过载有关
typedef struct
{
	Uint16 flag;
	Uint16 wait_cnt;
	float gQs;			//外部给定Q轴电流值
	float gCountQs;		//总过载值
	float gQsAdd;		//单次累加过载值
	float gRefQs;		//过载电流阀值
	float gRefCountQs;	//总过载值阀值
}MP_OverLoadTypeDef;

extern MP_OverLoadTypeDef MP_OverLoadVar;

void MP_OverLoadInit(void);
void MP_OverLoadMacro(MP_OverLoadTypeDef *dp);
/********************************************************************************************************************/


#endif /* MATHOVERLOAD_H_ */
