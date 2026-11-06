#include <stdio.h>
#include <string.h>
#include "board.h"
#include "upgrade.h"
#include "hal_for_upg.h"
#include "hpm_l1c_drv.h"
#include "hpm_gptmr_drv.h"
typedef void (*pFunction)(void);
pFunction Jump_To;
volatile uint32_t JumpAddress;
uint8_t m_stat = STAT_INIT;
uint8_t m_pack_idx = 0;
uint8_t m_batch_flag = 0; //
static int32_t flash_ptr = 0;
#define CRC_CHAR_FLAG (('C' << 0) | ('R' << 8) | ('C' << 16) | (0x10 << 24))

static int hal_can_read(uint8_t data[8]);

//#########################################################################

unsigned short have_crc_flag(uint32_t *arg)
{
	if (*arg == CRC_CHAR_FLAG)
	{
		return 1;
	}
	else
		return 0;
}

void GoApp(void)
{
#ifdef BOOT_DEF
	int i;
	for (i = 0; i < 2; i++)
	{
		if (*(volatile uint32_t *)(PROG_BASE + i * 4) != 0xFFFFFFFF)
		{
			break;
		}
		else
			continue;
	}
	if (i == 2) // app is empty
	{
		return;
	}
	gptmr_stop_counter(BOARD_CALLBACK_TIMER, BOARD_CALLBACK_TIMER_CH);
	gptmr_disable_irq(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_IRQ_MASK(BOARD_CALLBACK_TIMER_CH));
	disable_global_irq(CSR_MSTATUS_MIE_MASK);
	// disable_global_irq(CSR_MSTATUS_SIE_MASK);
	disable_global_irq(CSR_MSTATUS_UIE_MASK);
	l1c_dc_writeback_all();
	l1c_dc_disable();
	l1c_ic_disable();
	fencei();
	__asm("la a0, %0" ::"i"(PROG_BASE));
	__asm("jr a0");
#endif
}

uint8_t upgrade_prepare(void)
{
	uint8_t ret = 0;
	uint16_t app_pages = (APP_DATA_ADDR - APP_ADDR) / PAGE_SIZE;
	hal_flash_unlock();
	for (int i = 0; i < app_pages; i++)
	{
		ret = hal_flash_erase_page(APP_ADDR + i * PAGE_SIZE);
		if (ret != 0)
		{
			return ret;
		}
	}
	hal_flash_lock();
	return 0;
}

uint8_t *upgrade_get_Data_start()
{
	return (uint8_t *)PROG_BASE;
}

#define POLY16 0x1021
unsigned short CRC16(unsigned char *buf, unsigned long dlen, int poly, int CRCinit)
{
	unsigned char ch;
	int i;
	unsigned short CRC_loc = (unsigned short)CRCinit;

	while (dlen--)
	{
		ch = *buf++;
		CRC_loc ^= (((unsigned short)ch) << 8);
		for (i = 0; i < 8; i++)
		{
			if (CRC_loc & 0x8000)
				CRC_loc = (CRC_loc << 1) ^ poly;
			else
				CRC_loc <<= 1;
		}
	}
	return CRC_loc;
}

unsigned short check_crc_ok(unsigned char *buff, unsigned short CRCdata, unsigned long datelen, unsigned short *crcd)
{
	unsigned short crc_temp;
	crc_temp = CRC16((unsigned char *)buff, datelen, POLY16, 0);
	if (crcd)
		*crcd = crc_temp;
	if (crc_temp == CRCdata)
	{
		return 1;
	}
	else
		return 0;
}

uint8_t upgrade_upgrade_loop(void)
{
	uint8_t rData[8];
	uint8_t sData[8];
	uint32_t data;

	if (hal_can_read(rData) != 0 && (rData[1] == 0xff || rData[1] == hal_get_dev_id()))
	{
		T_CAN_WORD_CMD *pCmd = (T_CAN_WORD_CMD *)rData;
		memcpy(sData, rData, 8);

		if (rData[0] == UPGRADE_DATA_INT)
		{
			if (m_pack_idx == rData[6] || m_batch_flag == 1)
			{
				int status;
				hal_flash_unlock();
#if 0
				status = hal_flash_program_halfword(APP_ADDR + flash_ptr, pCmd->arg1);
				if (status != 0)
				{
					sData[2] = status;
				}
				flash_ptr += 2;
				status = hal_flash_program_halfword(APP_ADDR + flash_ptr, pCmd->arg2);
				if (status != 0)
				{
					sData[2] = status;
				}
				flash_ptr += 2;
#else
				data = (pCmd->arg1) | (pCmd->arg2 << 16);
				status = hal_flash_program(APP_ADDR + flash_ptr, (uint8_t *)&data, sizeof(uint32_t));
				if (status != 0)
				{
					sData[2] = status;
				}
				flash_ptr += 4;
#endif
				hal_flash_lock();
				sData[2] = NO_ERROR;
				sData[3] = rData[6] + 1;
				memset(&sData[4], 0, 4);
				m_pack_idx++;
			}
			else
			{
				sData[2] = PACK_ERROR;
				sData[3] = rData[6] + 1;
				memset(&sData[4], 0, 4);
			}
			sData[1] = hal_get_dev_id();
			if (m_batch_flag == 0)
				hal_can_send(sData, 8);
		}
		else if (rData[0] == UPGRADE_PRG_ADDR)
		{
			sData[1] = hal_get_dev_id();
			sData[2] = NO_ERROR;
			sData[3] = rData[6] + 1;
			memset(&sData[4], 0, 4);
			m_pack_idx++;
			if (m_batch_flag == 0)
				hal_can_send(sData, 8);
		}
		else if (rData[0] == UPGRADE_DATA_WORD)
		{
			if (m_pack_idx == rData[6])
			{
				int status;
				hal_flash_unlock();
#if 0
				status = hal_flash_program_halfword(APP_ADDR + flash_ptr, pCmd->arg1);
#else
				status = hal_flash_program(APP_ADDR + flash_ptr, (uint8_t *)&pCmd->arg1, sizeof(uint32_t));
#endif
				if (status != 0)
				{
					rData[2] = status;
				}
				flash_ptr += 2;
				hal_flash_lock();
				sData[2] = NO_ERROR;
				sData[3] = rData[6] + 1;
				memset(&sData[4], 0, 4);
				m_pack_idx++;
			}
			else
			{
				sData[2] = PACK_ERROR;
				sData[3] = rData[6] + 1;
				memset(&sData[4], 0, 4);
			}
			sData[1] = hal_get_dev_id();
			hal_can_send(sData, 8);
		}
		else if (rData[0] == UPGRADE_END)
		{
			unsigned short CRC_data_com =0 ;

			char crc_flg[6] = {0};
			hal_flash_unlock();
			memcpy(crc_flg, (uint8_t *)(PROG_BASE + flash_ptr - 6), 6);
			if (have_crc_flag((uint32_t *)(PROG_BASE + flash_ptr - 6)))
			{
				if (!check_crc_ok(upgrade_get_Data_start(), *((uint16_t *)(PROG_BASE + flash_ptr - 2)), flash_ptr - 6, &CRC_data_com))
				{
					sData[2] = CRC_ERROR;
				}
				else
				{
					sData[2] = NO_ERROR;
					hal_flag_write(APP_CRC, CRC_data_com);
				}
			}
			else
			{
				sData[2] = CRC_FLAG_ERROR;
			}
			hal_flash_lock();
			sData[1] = hal_get_dev_id();
			sData[3] = rData[6] + 1;
			sData[4] = crc_flg[4];
			sData[5] = crc_flg[5];
			sData[6] = CRC_data_com & 0xff;
			sData[7] = (CRC_data_com >> 8) & 0xff;
			if (m_batch_flag == 0)
				hal_can_send(sData, 8);
                        board_delay_ms(50) ;
			if (sData[2] > 0)
			{
				m_stat = STAT_ERROR;
				return 1;
			}
			else
			{

				m_stat = STAT_END;
				return 2;
			}
		}
		else
		{
			sData[2] = 0;
		}

		if (sData[2] > 0)
		{
			m_stat = STAT_ERROR;
			return 1;
		}
		hal_led_trig();
	}

	return 0;
}

uint8_t upgarde_stat(void)
{
	return m_stat;
}

uint8_t upgrade_is_need(void)
{
	int retry_cnt = 0;
	uint8_t rData[8];
	uint8_t sData[8];
	uint16_t upg_flag;

	upg_flag = hal_flag_read(UPG_FLAGE);
	if (upg_flag == CMD_UPGRADE_START)
	{
		uint8_t ret;
		flash_ptr = 0;
		m_batch_flag = 0;
		ret = upgrade_prepare();
		upg_flag = 0;
		hal_flag_write(UPG_FLAGE, upg_flag);
		sData[0] = UPGRADE_REQUEST;
		if (ret)
		{
			sData[1] = hal_get_dev_id();
			sData[2] = ret;
			sData[3] = 1;
			memset(&sData[4], 0, 4);
			hal_can_send(sData, 8);
			m_stat = STAT_ERROR;
			return 0;
		}
		else
		{
			sData[1] = hal_get_dev_id();
			sData[2] = NO_ERROR;
			sData[3] = 1;
			memset(&sData[4], 0, 4);
			hal_can_send(sData, 8);
			m_stat = STAT_UPGRADE;
			m_pack_idx++;
			return 1;
		}
	}
	else if (upg_flag == CMD_UPGRADE_BATCH_START)
	{
		uint8_t ret;
		flash_ptr = 0;
		m_batch_flag = 1;
		ret = upgrade_prepare();
		upg_flag = 0;
		hal_flag_write(UPG_FLAGE, upg_flag);
		sData[0] = UPGRADE_REQUEST;
		if (ret)
		{
			return 0;
		}
		else
		{
			m_stat = STAT_UPGRADE;
			m_pack_idx++;
			return 1;
		}
	}
	else
	{
		while (retry_cnt <= 200 /*|| hal_flag_read(APP_CRC) == 0xffff*/)
		{
			if (hal_can_read(rData) != 0)
			{
				flash_ptr = 0;
				memcpy(sData, rData, 8);
				if (rData[7] == 1)
					m_batch_flag = 1;
				else
					m_batch_flag = 0;
				if (rData[0] == UPGRADE_REQUEST)
				{
					uint8_t ret;

					hal_set_dev_id(rData[1]);
					ret = upgrade_prepare();
					if (ret)
					{
						if (m_batch_flag == 0) // 2????y??
						{
							sData[1] = hal_get_dev_id();
							sData[2] = ret;
							sData[3] = rData[6] + 1;
							memset(&sData[4], 0, 4);
							hal_can_send(sData, 8);
						}
						m_stat = STAT_ERROR;
						return 0;
					}
					else
					{
						if (m_batch_flag == 0) // 2????y??
						{
							sData[1] = hal_get_dev_id();
							sData[2] = NO_ERROR;
							sData[3] = rData[6] + 1;
							memset(&sData[4], 0, 4);
							hal_can_send(sData, 8);
						}
						m_stat = STAT_UPGRADE;
						m_pack_idx++;
						return 1;
					}
				}
			}
			retry_cnt++;
			hal_delayms(1);
		}
	}
	return 2;
}

#define CMD_BUF_LEN (50)
static uint8_t cmd_buffer[CMD_BUF_LEN][8];
static int32_t cmd_rd = 0;
static int32_t cmd_wr = 0;
#ifdef BOOT_DEF
int easyCanPush(uint8_t data[8])
{
	int32_t len;

	len = cmd_wr - cmd_rd;
	if (len < 0)
		len += CMD_BUF_LEN;
	if (len < CMD_BUF_LEN - 1) // not full
	{
		memcpy(cmd_buffer[cmd_wr], data, 8);
		cmd_wr++;
		cmd_wr %= CMD_BUF_LEN;

		return 0;
	}
	return -1;
}
#endif
static int hal_can_read(uint8_t data[8])
{
	if (cmd_wr != cmd_rd) // not empty
	{
		memcpy(data, cmd_buffer[cmd_rd], 8);
		cmd_rd++;
		cmd_rd %= CMD_BUF_LEN;

		return 1;
	}

	return 0;
}

void boot_main(void)
{

#if 1
	while (1)
	{
		uint8_t ret = upgrade_is_need();
		if (ret == 1)
		{
			hal_led_trig();
			while (1)
			{
				uint8_t ret1 = upgrade_upgrade_loop();
				if (ret1 == 2)
				{
					GoApp();
				}
				else if (ret1 == 1)
					break;
			}
		}
		else if (ret == 2)
		{

			GoApp();
		}
		else
		{
		}
	}
#endif
	// hal_delayms(500);
}
