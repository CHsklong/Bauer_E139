/******************** (C) COPYRIGHT 2023 HPMicro ********************
* File Name          : eeprom.h
* Author             : FAE Team
* Date First Issued  : 23/09/03 : EEProm Emulation Version 1.0
* Description        : This file provides all routines used in Eprom emulation.
********************************************************************************
* History:
* 23/09/03 : Eprom Emulation Version 1.0
********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"
#include "hpm_pmp_drv.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
xpi_nor_config_t s_xpi_nor_config;
uint32_t flash_size=0;//zero means invalid initialization
uint32_t sector_size;
uint32_t page_size;
ATTR_PLACE_AT_WITH_ALIGNMENT(".ahb_sram1",8) uint8_t EEPROM_data[SECTOR1_END_ADDRESS-SECTOR1_BASE_ADDRESS+1];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*The EEPROM_Write( ) function provides functionality for storing data into a temporary buffer.*/
inline void EEPROM_Write(uint16_t index, uint8_t data)
{
    EEPROM_data[index]=data;
}/* End EEPROM_Write */

/*The EEPROM_Read( ) function provides functionality for reading the data and storing it into a temporary buffer.*/
inline uint8_t EEPROM_Read(uint16_t index)  
{
  return EEPROM_data[index];
}/* End EEPROM_Read */
inline uint8_t *EEPROM_Read_Data_addr(void) 
{
  return EEPROM_data ;
}

/*******************************************************************************
* Routine:  close_EEprom_cache
* Purpose:  close cache for emulated flash as eeprom 
* Input  :  user entry index
* return :  none
*******************************************************************************/
//void close_EEprom_cache(uint32_t entry_user)
//{

//    uint32_t start_addr = SECTOR1_BASE_ADDRESS;
//    uint32_t end_addr = SECTOR1_END_ADDRESS;
//    uint32_t length = end_addr - start_addr+1;

//    if (length == 0) {
//        return;
//    }

//    /* Ensure the address and the length are power of 2 aligned */
//    assert((length & (length - 1U)) == 0U);
//    assert((start_addr & (length - 1U)) == 0U);

//    pmp_entry_t pmp_entry[1] = {0};

//    pmp_entry[0].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
//    pmp_entry[0].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
//    //pmp_entry[0].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
//    //pmp_entry[0].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
//    pmp_config_entry(&pmp_entry[0], entry_user+3);
//}
/*******************************************************************************
* Routine:  InitialEEProm
* Purpose:  initialize EEProm to get flash information
* Input  :  none
* Output :  flash struct and EEPROM buffer.
* return :  Common status code
*******************************************************************************/
#if defined(FLASH_XIP) && FLASH_XIP
ATTR_RAMFUNC hpm_stat_t Initial_EEProm(void)
#else
hpm_stat_t Initial_EEProm(void)
#endif
{      
    xpi_nor_config_option_t option;
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1; 
    //close_EEprom_cache(0);//close cache for eeprom  
    hpm_stat_t status = rom_xpi_nor_auto_config(HPM_XPI0, &s_xpi_nor_config, &option);
    if (status != status_success) {
        return status;
    }
    rom_xpi_nor_get_property(HPM_XPI0, &s_xpi_nor_config, xpi_nor_property_total_size,
                             &flash_size);
    rom_xpi_nor_get_property(HPM_XPI0, &s_xpi_nor_config, xpi_nor_property_sector_size,
                             &sector_size);
    rom_xpi_nor_get_property(HPM_XPI0, &s_xpi_nor_config, xpi_nor_property_page_size, &page_size);
    
/*    printf("Flash Size:%dMBytes\nFlash Sector Size:%dKBytes\nFlash Page Size:%dBytes\n",
           flash_size / 1024U / 1024U, sector_size / 1024U, page_size);
    */
    EEProm_Flush();

    return status ;
} /* End  Initial_EEProm */

/*******************************************************************************
* Routine:  EEPromFlush
* Purpose:  Refresh data from flash to buffer.
*******************************************************************************/
inline void EEProm_Flush(void)
{
    rom_xpi_nor_read(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                               (uint32_t *)EEPROM_data, SECTOR1_BASE_ADDRESS-Flash_base, SECTOR1_END_ADDRESS-SECTOR1_BASE_ADDRESS+1);
} /* End EEProm_Flush */

/*******************************************************************************
* Routine:  EEProm_Bulkcopy
* Purpose:  Refresh data from eeprom buffer to customer buffer.
*******************************************************************************/
inline void EEProm_Bulkcopy(uint16_t index, uint8_t *data, uint16_t size)
{
    memcpy((void*)data,(const void*)EEPROM_data,size);
} /* End EEProm_Bulkcopy */
/*******************************************************************************
* Routine:  EEProm_Bulkupdate
* Purpose:  Refresh data from customer buffer to eeprom  buffer.
*******************************************************************************/
inline void EEProm_Bulkupdate(uint16_t index, uint8_t *data, uint16_t size)
{
    memcpy((void*)EEPROM_data,(const void*)data,size);
} /* End EEProm_Bulkupdate */
static inline hpm_stat_t EEProm_earse(uint32_t start,uint32_t length)
{
    return rom_xpi_nor_erase(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                               start, length);
}
static inline hpm_stat_t EEProm_program( const uint32_t *src,uint32_t dst_addr,uint32_t length)
{
    return rom_xpi_nor_program(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                 (const uint32_t *)src, dst_addr, length);
}
/*******************************************************************************
* Routine:  EEProm_Store
* Purpose:  Writes full variable in EEPROM.
* Input  :  none
* Output:    None.
* Return:   Returns 0 
*******************************************************************************/
#if defined(FLASH_XIP) && FLASH_XIP
ATTR_RAMFUNC hpm_stat_t EEProm_Store(void)
#else
hpm_stat_t EEProm_Store(void)
#endif
{
    hpm_stat_t status;
    if(flash_size==0) return status_fail;
    status = rom_xpi_nor_erase(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                               SECTOR1_BASE_ADDRESS-Flash_base, SECTOR1_END_ADDRESS-SECTOR1_BASE_ADDRESS);
    if (status != status_success) {
        return status;
    }
    status = rom_xpi_nor_program(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                 (const uint32_t *)EEPROM_data, SECTOR1_BASE_ADDRESS-Flash_base, SECTOR1_END_ADDRESS-SECTOR1_BASE_ADDRESS);
    if (status != status_success) {
        return status;
    }
    return status_success ;
}  /* End EEProm_Store */

/*******************************************************************************
* Routine:  writeEEProm_withflush
* Purpose:  Writes variable to EEPROM and flush flash later.
* Input  :  none
* Output:    None.
* Return:   Returns 0 
*******************************************************************************/
#if defined(FLASH_XIP) && FLASH_XIP
ATTR_RAMFUNC hpm_stat_t writeEEProm_withflush(uint16_t index, uint8_t *data, uint16_t size)
#else
hpm_stat_t writeEEProm_withflush(uint16_t index, uint8_t *data, uint16_t size)
#endif
{
    hpm_stat_t status;
    if(flash_size==0) return status_fail;
    memcpy((void*)&EEPROM_data[index],(const void*)data,size);
    status = rom_xpi_nor_erase(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                               SECTOR1_BASE_ADDRESS-Flash_base, SECTOR1_END_ADDRESS-SECTOR1_BASE_ADDRESS);
    if (status != status_success) {
        return status;
    }
    status = rom_xpi_nor_program(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                 (const uint32_t *)EEPROM_data, SECTOR1_BASE_ADDRESS-Flash_base, SECTOR1_END_ADDRESS-SECTOR1_BASE_ADDRESS);
    if (status != status_success) {
        return status;
    }
    return status_success ;
}


#if defined(FLASH_XIP) && FLASH_XIP
ATTR_RAMFUNC hpm_stat_t writeEEProm_withflush2(uint16_t index, uint8_t *data, uint16_t size)
#else
hpm_stat_t writeEEProm_withflush2(uint16_t index, uint8_t *data, uint16_t size)
#endif
{
    hpm_stat_t status;
    if(flash_size==0) return status_fail;
    memcpy((void*)&EEPROM_data[index],(const void*)data,size);
    status = rom_xpi_nor_erase(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                               SECTOR2_BASE_ADDRESS-Flash_base, SECTOR2_END_ADDRESS-SECTOR2_BASE_ADDRESS);
    if (status != status_success) {
        return status;
    }
    status = rom_xpi_nor_program(HPM_XPI0, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                 (const uint32_t *)EEPROM_data, SECTOR2_BASE_ADDRESS-Flash_base, SECTOR2_END_ADDRESS-SECTOR2_BASE_ADDRESS);
    if (status != status_success) {
        return status;
    }
    return status_success ;
}
/************************ (c) 2006  ST Microelectronics ******* END OF FILE ***/