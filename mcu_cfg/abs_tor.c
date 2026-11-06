#include "SysMain.h"


MT_ABS_TYPEDEF mt_abs;

//----------------------------------------------------
void mt_abs_var_init(void)
{
	memset_16(&mt_abs.crc_abs, 0, sizeof_16(MT_ABS_TYPEDEF));
	mt_abs.cir_lines = 1 << MT_ABS_BIT ;
	mt_abs.max = mt_abs.cir_lines << 1 ;
	mt_abs.min = - mt_abs.cir_lines;
	mt_abs.half = mt_abs.cir_lines >> 1 ;
}

//------------------------------------------------------------------------------------------------------------------


ATTR_RAMFUNC void spi_abs_tor_dma_tr_rx(void)
{
// int16_t  addr ;
	hw_SPI3_ReceiveMessage(0xA003, 4);
}

void spi_abs_tor_write_add0(void)
{
	hw_SPI3_SendMessage(0x6000, wbuff, 1);

}

void spi_abs_tor_read_add0(void)
{
	hw_SPI3_ReceiveMessage(0x3000, 1);
}


//------------------------------------------
ATTR_RAMFUNC void spi_abs_tor_read_angle(void)
{
	rbuff[0] = SPI3_receive_ahb[4];
	rbuff[1] = SPI3_receive_ahb[5];
	rbuff[2] = SPI3_receive_ahb[6];
	rbuff[3] = SPI3_receive_ahb[7];
	mt_abs.crc_abs =  Crc_Table((uint8_t *)rbuff, 3);

	if (mt_abs.crc_abs == rbuff[3])
	{
		mt_abs.cur = ((rbuff[0] << 13) | (rbuff[1] << 5) | (rbuff[2] >> 3)) >> MT_SHIFT ; //>> 5;
		mt_abs.status  = rbuff[2] & 0x07 ;

		if (mt_abs.first )
		{
			mt_abs.inc = mt_abs.cur - mt_abs.old ;

			mt_abs.old = mt_abs.cur ;
			if (mt_abs.inc > mt_abs.half) {
				mt_abs.inc -= mt_abs.cir_lines ;
				
			} else if (mt_abs.inc < -mt_abs.half) {
				mt_abs.inc += mt_abs.cir_lines ;
				
			}
			mt_abs.abs_angle += mt_abs.inc ;
			mt_abs.delt = mt_abs.inc ;

		} else {
			mt_abs.old = mt_abs.cur ;
			mt_abs.abs_angle = mt_abs.cur ;

			mt_abs.inc = 0 ;

			mt_abs.first = 1 ;

		}
	}
}

