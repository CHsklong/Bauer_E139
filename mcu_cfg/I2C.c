
#include "I2C.h"
#include "board.h"
#define TEST_TRANSFER_DATA_IN_BYTE  (128U)
uint8_t rx_buff[TEST_TRANSFER_DATA_IN_BYTE]={0,0};
uint8_t tx_buff[TEST_TRANSFER_DATA_IN_BYTE]={0x55,0x55};

void I2C_gpio_init(void)
{

    HPM_IOC->PAD[IOC_PAD_PA08].FUNC_CTL = IOC_PA08_FUNC_CTL_I2C2_SCL  | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_I2C2_SDA  | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;

    HPM_IOC->PAD[IOC_PAD_PB08].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    HPM_IOC->PAD[IOC_PAD_PB09].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
  
 
}

uint32_t I2C_Write_data(uint16_t device_address,uint8_t *addr, uint32_t addr_size_in_byte,uint8_t *buf, const uint32_t size_in_byte)
{
	return i2c_master_address_write(HPM_I2C2,device_address,addr,addr_size_in_byte,buf,size_in_byte);
}
uint32_t I2C_Read_data(uint16_t device_address,uint8_t *addr, uint32_t addr_size_in_byte,uint8_t *buf, const uint32_t size_in_byte)
{
	return i2c_master_address_read(HPM_I2C2,device_address,addr,addr_size_in_byte,buf,size_in_byte);
}
void I2C_init(void)
{

    
    hpm_stat_t stat;
    i2c_config_t config;
    uint32_t freq;
//    uint16_t WriteAddr =0 ;
    I2C_gpio_init() ;

    config.i2c_mode = i2c_mode_normal;
    config.is_10bit_addressing = false;
    freq = clock_get_frequency(clock_i2c2);
    stat = i2c_init_master(HPM_I2C2, freq, &config);
    if (stat != status_success) {
        
        return ;
    }
#if 0
    if (status_success != i2c_master_write(HPM_I2C2, 0XA0+(((WriteAddr>>8)&0x07)<<1), tx_buff, sizeof(tx_buff))) {
        printf("Master write failed");
         
         
    }

    /* wait for slave controller to be ready to send data */
    board_delay_ms(100);

    if (status_success != i2c_master_read(HPM_I2C2, 0XA0+(((WriteAddr>>8)&0x07)<<1), rx_buff, sizeof(rx_buff))) {
        printf("Master read failed\n");
        
        
    }
#endif
}

