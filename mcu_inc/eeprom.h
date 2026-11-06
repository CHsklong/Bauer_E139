/******************** (C) COPYRIGHT 2023 HPMicro ********************
* File Name          : eeprom.h
* Author             : FAE Team
* Date First Issued  : 23/09/03 : EEProm Emulation Version 1.0
* Description        : This file provides the routines' prototypes and the variables 
*                      used in the eeprom.c source file.
********************************************************************************
* History:
* 23/09/03 : EEProm Emulation Version 1.0
********************************************************************************/
#ifndef __EEPROM_H
#define __EEPROM_H

#include "board.h"
#include "hpm_romapi.h"


/* Sector size */
#define SECTOR_SIZE	(uint32_t) (0x1000)	
#define PARA_SIZE       (SECTOR_SIZE * 2 -1)     
/* Sectors 0 and 1 base and end addresses */
#define Flash_base 0x80000000L
//#define SECTOR1_BASE_ADDRESS (Flash_base+0x3FE000)
#define SECTOR1_BASE_ADDRESS (Flash_base+0xFE000)//just for 5300 with 1mbyte
#define SECTOR1_END_ADDRESS  (SECTOR1_BASE_ADDRESS+SECTOR_SIZE*2-1)

#define SECTOR2_BASE_ADDRESS (Flash_base+0xFA000)//just for 5300 with 1mbyte
#define SECTOR2_END_ADDRESS  (SECTOR1_BASE_ADDRESS+SECTOR_SIZE*2-1)

/* Routines' prototypes */
void EEPROM_Write(uint16_t index, uint8_t data);
uint8_t EEPROM_Read(uint16_t index);
hpm_stat_t Initial_EEProm(void);
hpm_stat_t EEProm_Store(void);
void EEProm_Flush(void);
hpm_stat_t writeEEProm_withflush(uint16_t index, uint8_t *data, uint16_t size);
void EEProm_Bulkcopy(uint16_t index, uint8_t *data, uint16_t size);
void EEProm_Bulkupdate(uint16_t index, uint8_t *data, uint16_t size);

uint8_t *EEPROM_Read_Data_addr(void)  ;
ATTR_RAMFUNC hpm_stat_t writeEEProm_withflush2(uint16_t index, uint8_t *data, uint16_t size);
#endif

/************************ (c) 2005  ST Microelectronics ******* END OF FILE ***/
