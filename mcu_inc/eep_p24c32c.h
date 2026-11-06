#ifndef EEP_P24C32_H
#define EEP_P24C32_H
#include "board.h"


void P24CLXX_Init(void);

uint8_t P24CLXX_ReadOneByte(uint16_t ReadAddr);

void P24CLXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);


uint32_t P24CLXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);  

uint32_t P24CLXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);

#endif