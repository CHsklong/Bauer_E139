#ifndef	_ABS_TOR_H_
#define	_ABS_TOR_H_



typedef  struct {

	uint8_t crc_abs ;
	uint8_t status ;
	uint8_t first ;
	uint8_t i ;
	int32_t abs_angle ;
	int32_t inc;
	int32_t cur;
	int32_t old;
	int32_t delt;

	int32_t cir_lines;
	int32_t max;
	int32_t min;
	int32_t half;


} MT_ABS_TYPEDEF;

extern MT_ABS_TYPEDEF mt_abs;



void mt_abs_var_init(void);


void spi_abs_tor_dma_tr_rx(void);

void spi_abs_tor_write_add0(void);
void spi_abs_tor_read_add0(void);


void spi_abs_tor_read_angle(void);











#endif
