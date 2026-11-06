#ifndef _HAL_FOR_UPG_H_
#define _HAL_FOR_UPG_H_
#include "eeprom.h"
#include "mcdrv.h"
#define CMD_UPGRADE_START 0x1
#define CMD_UPGRADE_BATCH_START 0x2

#define APP_ADDR        0x00020000       // 128K bytes
#define APP_DATA_ADDR   0x000f0000       // 832k bytes
#define BOOT_ADDR       0x00003000       // 12k bytes
#define CRC_ADDR        0X0001effc     
//#define PAGE_SIZE       (pflashSectorSize)
#define PAGE_SIZE    (SECTOR_SIZE)
#define UPG_FLAGE		 0x0	// 
#define APP_CRC	     0x1	//

#define PROG_BASE   APP_ADDR + 0x80000000



 
#if (BOARD_TYPE == BOARD_E139)
#define APP_VER 5602 //支持 60电机冲孔
#elif (BOARD_TYPE == BOARD_E914)
//#define APP_VER 4601 //支持 60电机冲孔
//#define APP_VER 4602 //修改ID设置多个闪问题
//#define APP_VER 4605 //跟随和运行参数分开
#define APP_VER 4607 //随机数重复和修改修改完ID 灯不闪烁问题 跟随误差设置0 出现的问题
#elif (BOARD_TYPE == BOARD_E145)
//#define APP_VER 6608 //步进电机版本 方向反
#define APP_VER 6609 //作为升降电机 修复过流不报警问题
#elif (BOARD_TYPE == BOARD_E884)
//#define APP_VER 8502
//#define APP_VER 8505 //增加自动排刀和外部控制的互锁功能
//#define APP_VER 8506 //增加左右限位功能
//#define APP_VER 8507 //加大间隙检测 
#define APP_VER 8508 //力矩模式 跟原先方向全 
#endif

//extern uint32_t pflashSectorSize;	

int hal_flash_init(void);
void hal_flash_unlock(void);
int  hal_flash_erase_page(uint32_t Page_Address);
int hal_flash_program(uint32_t Address, uint8_t *Data, uint32_t len);
int  hal_flash_program_halfword(uint32_t Address, uint16_t Data);
void hal_flash_lock(void);
void hal_flash_enable_read_protection(void);
void hal_flash_disable_read_protection(void);

int hal_can_send(uint8_t *cmd, uint8_t len);
int hal_led_trig(void);

uint16_t hal_flag_read(int flag);
int hal_flag_write(int flag, uint16_t data);

extern void hal_delayms(uint32_t time);

void hal_set_dev_id(uint8_t id);
uint16_t hal_get_dev_id(void);

uint8_t hal_upg_begin(uint8_t batch_flag);
void hpm_ota_soc_reset(void);
uint32_t get_bootflag_data(void);
void set_bootflag_data(uint32_t flag);
#endif
