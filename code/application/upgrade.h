
#ifndef __UPGRADE_H__
#define __UPGRADE_H__
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif


#define UPGRADE_REQUEST   (0xE1)
#define UPGRADE_PRG_ADDR  (0xE2)
#define UPGRADE_DATA_INT  (0xE3)
#define UPGRADE_DATA_WORD (0xE4)
#define UPGRADE_END       (0xE5)

enum
{
  NO_ERROR,
  FLASH_ERROR,
  PACK_ERROR,
  NO_UPGRADE = 6,
  CRC_ERROR,
  CRC_FLAG_ERROR,
  DEVICE_ERROR,
};
enum
{
  STAT_INIT,
  STAT_UPGRADE,
  STAT_ERROR,
  STAT_END
};

typedef struct 
{
	uint8_t cmd_type;
	uint8_t cmd_data;
	uint16_t arg1;
	uint16_t arg2;
	uint16_t arg3;
}T_CAN_WORD_CMD;

int cmdPush(uint8_t data[8]);

uint8_t upgrade_prepare(void);
uint8_t upgrade_upgrade_loop(void);
uint8_t upgrade_is_need(void);
uint8_t upgarde_stat(void);
void GoApp(void);
void boot_main(void);

#endif
