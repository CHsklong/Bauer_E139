#include <string.h>
#include <stdint.h>
#include "board.h"
#include "tick.h"
#include "servo_can.h"
#include "eeprom.h"
#include "hpm_platform.h"
#include "mcdrv.h"
#include "hal_for_upg.h"
#include "mcu.h"
#include "SysMain.h"

uint16_t upg_flag;

uint16_t dev_id;

uint16_t app_crc;

//static uint32_t *boot_flag = (uint32_t *)HPM_PGPR0;

uint32_t pflashBlockBase = 0;
uint32_t pflashTotalSize = 0;
uint32_t pflashSectorSize = 0;

extern uint16_t mot_id ; 

//##########################################################################################
#define SOC_ROM_FLAG_REG (*(volatile uint32_t *)(0xF40D4000UL + 8U))

static uint32_t *boot_flag = (uint32_t *)HPM_PGPR0;


void hpm_ota_soc_reset(void)
{
	disable_global_irq(CSR_MSTATUS_MIE_MASK);
	SOC_ROM_FLAG_REG = 0;                            // warning: this is rom reset flag,
	ppor_sw_reset(HPM_PPOR, 1000);
	while (1)
	{
	}
}

uint32_t get_bootflag_data(void)
{

	return (uint32_t) * boot_flag;

}

void set_bootflag_data(uint32_t flag)
{
	*boot_flag = flag;
}
int hal_flash_init(void)
{
  Initial_EEProm();
  int ret = hpm_platform_flash_init() ;

  upg_flag = *boot_flag ;
  dev_id = mot_id ;
  return ret ;
}

void hal_flash_unlock(void)
{
  
}

int hal_flash_erase_page(uint32_t Page_Address)
{
    return hpm_flash_erase_sector(Page_Address) ;
}

int hal_flash_program_halfword(uint32_t Address, uint16_t Data)
{
      // not support
    return 0;
}

int hal_flash_program(uint32_t Address, uint8_t* Data, uint32_t len)
{

   return  hpm_flash_write(Address,Data,len) ;
}

void hal_flash_lock(void)
{

}


//////////////////CAN command process ////////////////////////////////////
int hal_can_send(uint8_t* cmd, uint8_t len)
{
    return BOARD_CanSend(0x100 + dev_id, cmd, len);
}

int hal_led_trig(void)
{
    static uint32_t delay = 0;

    if (delay == 0 || delay++ > 30)
    {
        USER_LED_TOGGLE();
        delay = 1;
    }

    return 0;
}

int hal_flag_write(int flag, uint16_t data)
{
    if (flag == UPG_FLAGE)
    {
      upg_flag = data;
      *boot_flag = data ;
    }
    else if (flag == APP_CRC)
    {
       return hal_flash_program(CRC_ADDR,(uint8_t*)&data, sizeof(uint16_t)) ;
       
    }
    return 0;
}

uint16_t hal_flag_read(int flag)
{
    uint16_t data = 0;

    if (flag == UPG_FLAGE)
      data = upg_flag;
    else if (flag == APP_CRC)
      data = *((uint16_t*)CRC_ADDR);

    return data;
}

void hal_delayms(uint32_t time)
{
    uint32_t cur_time = getTicks();

    while (!expireTicks(cur_time, time));
}

void hal_set_dev_id(uint8_t id)
{
    if (id == 0xff)
      dev_id = 1;
    else
      dev_id = id;
}

uint16_t hal_get_dev_id()
{
    return dev_id;
}

uint8_t hal_upg_begin(uint8_t batch_flag)
{
    if (batch_flag == 1)
      hal_flag_write(UPG_FLAGE, CMD_UPGRADE_BATCH_START);
    else
      hal_flag_write(UPG_FLAGE, CMD_UPGRADE_START);
 //    set_bootflag_data(1);
 //    hpm_ota_soc_reset();
      return 0;
}

/****************************************************************
* Function:    Flash_EnableReadProtection
* Description: Enable the read protection of user flash area.
* Input:        NONE
* Output:        NONE
* Return:  NONE
*****************************************************************/
void hal_flash_enable_read_protection(void)
{


}

/****************************************************************
* Function:    Flash_DisableReadProtection
* Description: Disable the read protection of user flash area.
* Input:        NONE
* Output:        NONE
* Return:  NONE
*****************************************************************/
void hal_flash_disable_read_protection(void)
{

}
