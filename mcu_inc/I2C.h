#ifndef I2C_H
#define I2C_H

#include "hpm_soc.h"
#include "hpm_i2c_drv.h"


void I2C_init(void);
uint32_t I2C_Write_data(uint16_t device_address,uint8_t *addr, uint32_t addr_size_in_byte,uint8_t *buf, const uint32_t size_in_byte);

uint32_t I2C_Read_data(uint16_t device_address,uint8_t *addr, uint32_t addr_size_in_byte,uint8_t *buf, const uint32_t size_in_byte);
#endif