/**
  ******************************************************************************
  * @file    MT6816.h
  * @author  @robert
  * @version V0.0
  * @date    2020.02.11
  * @brief   MT6816 driver
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

#ifndef __MT6835_H
#define __MT6835_H

#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif
#define CAL_STATE_NO                (0)
#define CAL_STATE_RUNING            (1)
#define CAL_STATE_FAILED            (2)
#define CAL_STATE_SUCCESS           (3)


void MT6835_Init(void);
void MT6835_ABZ_Config(uint32_t lines);
uint32_t MT6835_Read_Angle(void);
void MT6835_auto_cal(uint8_t enb);
uint8_t MT6835_cal_state(void);


uint8_t Crc_Table(uint8_t *ptr, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif



