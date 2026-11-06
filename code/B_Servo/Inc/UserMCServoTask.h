/*
 * UserMCServoTask.h
 *
 *  Created on: 2016414
 *      Author: admin
 */

#ifndef USERMCSERVOTASK_H_
#define USERMCSERVOTASK_H_

#include "mcu.h"

extern        int32_t runTime3 ;

extern void (*MOTO1_SERVO_TASK_Api_Ptr)(void);


void MC_SERVO_TASK(void);
void MOTO1_SERVO_TASK_SEL(void);
#endif /* USERMCSERVOTASK_H_ */
