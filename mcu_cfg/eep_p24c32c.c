
#include "eep_p24c32c.h"
#include "board.h"
#include "I2C.h"

#define DEVICE_ADDR (0xA0 >>1)
void P24CLXX_Init(void)
{
	 I2C_init();
}

uint8_t P24CLXX_ReadOneByte(uint16_t ReadAddr)
{	
	uint8_t temp=0;		  	
   	I2C_Read_data(DEVICE_ADDR,(uint8_t *)&ReadAddr ,2,&temp,1) ;	

	return temp;
}

void P24CLXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{
  	I2C_Write_data(DEVICE_ADDR,(uint8_t *)&WriteAddr,2,&DataToWrite,1) ;	

}



uint32_t P24CLXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	return I2C_Read_data(DEVICE_ADDR,(uint8_t *)&ReadAddr,2,pBuffer,NumToRead) ;

}  

uint32_t P24CLXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	return I2C_Write_data(DEVICE_ADDR,(uint8_t *)&WriteAddr,2,pBuffer,NumToWrite) ;
	
}