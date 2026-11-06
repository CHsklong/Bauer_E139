/*
 * ServoTask_Pos.h
 *
 *  Created on: 2016421
 *      Author: admin
 */

#ifndef SERVOTASK_POS_H_
#define SERVOTASK_POS_H_

#include "mcu.h"

extern Uint8 pos_trig;

void Moto1_Pos_Init(void);
void Moto1_Pos_Macro(void);
void Moto2_Pos_Init(void);
void Moto2_Pos_Macro(void);


#endif /* SERVOTASK_POS_H_ */
