/*
 * ServoTask_Tor.h
 *
 *  Created on: 2016421
 *      Author: admin
 */

#ifndef SERVOTASK_TOR_H_
#define SERVOTASK_TOR_H_

#include "mcu.h"

void Moto1_Tor_Init(void);
void Moto1_Tor_TASK(void);

void Moto2_Tor_Init(void);
void Moto2_Tor_TASK(void);

float TOR_CMD_SELECT(void);

#endif /* SERVOTASK_TOR_H_ */
