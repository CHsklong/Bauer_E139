#include "SysMain.h"


uint32_t spi_clcok;
uint8_t wbuff[10] = {0xa0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8, 0xa9};
uint8_t rbuff[10] = {0};

uint8_t write[8] = { 0x55, 0xAA};

spi_timing_config_t timing_config = {0};
spi_format_config_t format_config = {0};
spi_control_config_t control_config = {0};



//-----------------------------------------------------------------------------------------------------------------------

ATTR_PLACE_AT(".ahb_sram1") volatile uint8_t SPI1_send_ahb[16] , SPI2_send_ahb[16];
ATTR_PLACE_AT(".ahb_sram1") volatile uint8_t SPI1_receive_ahb[16] , SPI2_receive_ahb[16];;


//-----------------------------------------------------------------------------------------------------------------------

/*
 * @brief initialize SPI2 as master and DMA
 */
void hw_SPI2_Init(void)
{
// spi_timing_config_t timing_config = {0};
// spi_format_config_t format_config = {0};



	clock_add_to_group(clock_spi2, 0);
	//      clock_set_source_divider(clock_spi1, clk_src_pll0_clk0, 100U); /* 400/100 = 4MHz */
	clock_set_source_divider(clock_spi2, clk_src_pll1_clk0, 10U); /* 80MHz */
	spi_clcok = clock_get_frequency(clock_spi2);

	/* set SPI sclk frequency for master */
	spi_master_get_default_timing_config(&timing_config);
	timing_config.master_config.clk_src_freq_in_hz = spi_clcok;
	timing_config.master_config.sclk_freq_in_hz = APP_SPI_SCLK_FREQ;
	spi_master_timing_init(HPM_SPI2, &timing_config);

	/* set SPI format config for master */
	spi_master_get_default_format_config(&format_config);
	format_config.common_config.data_len_in_bits = 8; /*8 bits data*/
	format_config.common_config.mode = spi_master_mode;
	format_config.common_config.cpol = spi_sclk_high_idle;
	format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
	format_config.master_config.addr_len_in_bytes = 2;
	spi_format_init(HPM_SPI2, &format_config);
	/* set DMAMUX for SPI master */
	//dmamux_config(TEST_SPI_DMAMUX, TEST_SPI_RX_DMAMUX_CH, TEST_SPI_RX_DMA_REQ, true);
	dmamux_config(HW_APP_DMAMUX, HW_SPI_TX_DMAMUX_CH, HW_SPI_TX_DMA, true);
	dmamux_config(HW_APP_DMAMUX, HW_SPI_RX_DMAMUX_CH, HW_SPI_RX_DMA, true);
}


/*
 * @brief initialize SPI2 as master and DMA
 */
void hw_SPI1_Init(void)
{
// spi_timing_config_t timing_config = {0};
// spi_format_config_t format_config = {0};



	clock_add_to_group(clock_spi1, 0);
	//      clock_set_source_divider(clock_spi1, clk_src_pll0_clk0, 100U); /* 400/100 = 4MHz */
	clock_set_source_divider(clock_spi1, clk_src_pll1_clk0, 10U); /* 80MHz */
	spi_clcok = clock_get_frequency(clock_spi1);

	/* set SPI sclk frequency for master */
	spi_master_get_default_timing_config(&timing_config);
	timing_config.master_config.clk_src_freq_in_hz = spi_clcok;
	timing_config.master_config.sclk_freq_in_hz = APP_SPI_SCLK_FREQ;
	spi_master_timing_init(HPM_SPI1, &timing_config);

	/* set SPI format config for master */
	spi_master_get_default_format_config(&format_config);
	format_config.common_config.data_len_in_bits = 8; /*8 bits data*/
	format_config.common_config.mode = spi_master_mode;
	format_config.common_config.cpol = spi_sclk_high_idle;
	format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
	format_config.master_config.addr_len_in_bytes = 2;
	spi_format_init(HPM_SPI1, &format_config);
	/* set DMAMUX for SPI master */
	//dmamux_config(TEST_SPI_DMAMUX, TEST_SPI_RX_DMAMUX_CH, TEST_SPI_RX_DMA_REQ, true);
	dmamux_config(HW_APP_DMAMUX, HW_SPI_TX_DMAMUX_CH, HW_SPI_TX_DMA, true);
	dmamux_config(HW_APP_DMAMUX, HW_SPI_RX_DMAMUX_CH, HW_SPI_RX_DMA, true);
}

/*
 * @brief trigger dma sending through spi
 */
hpm_stat_t spi_tx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, SPI_Type *spi_ptr, uint32_t src, uint8_t data_width, uint32_t size)
{
	dma_handshake_config_t config = {0};
	config.ch_index = ch_num;
	config.dst = (uint32_t)&spi_ptr->DATA;
	config.dst_fixed = true;
	config.src = src;
	config.src_fixed = false;
	config.data_width = data_width;
	config.size_in_byte = size;

	return dma_setup_handshake(dma_ptr, &config, true);
}

/*
 * @brief trigger dma receiving through spi
 */
ATTR_RAMFUNC hpm_stat_t spi_rx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, SPI_Type *spi_ptr, uint32_t dst, uint8_t data_width, uint32_t size)
{
	dma_handshake_config_t config = {0};
	config.ch_index = ch_num;
	config.dst = dst;
	config.dst_fixed = false;
	config.src = (uint32_t)&spi_ptr->DATA;
	config.src_fixed = true;
	config.data_width = data_width;
	config.size_in_byte = size;

	return dma_setup_handshake(dma_ptr, &config, true);
}
/**
 * @brief SPI get active status.
 *
 * @param ptr SPI base address.
 * @retval bool true for active, false for inactive
 */
//static inline bool spi_is_active(SPI_Type *ptr)
//{
//    return ((ptr->STATUS & SPI_STATUS_SPIACTIVE_MASK) == SPI_STATUS_SPIACTIVE_MASK) ? true : false;
//}
/*
 * @brief copy Message to from app data to ahb
 */
static inline void hw_SPI1_send_copy(uint16_t addr, volatile uint8_t *pData, uint16_t Size)
{
	uint16_t *addr_prt;
	volatile uint8_t *data_prt;
	addr_prt = (uint16_t *)SPI1_send_ahb;
	*addr_prt = addr;
	data_prt = &SPI1_send_ahb[4];
	uint16_t i;
	for (i = 0; i < Size; i++)
		*data_prt++ = *pData++;
}
void hw_SPI2_send_copy(uint16_t addr, volatile uint8_t *pData, uint16_t Size)
{
	uint16_t *addr_prt;
	volatile uint8_t *data_prt;
	addr_prt = (uint16_t *)SPI2_send_ahb;
	*addr_prt = addr;
	data_prt = &SPI2_send_ahb[4];
	uint16_t i;
	for (i = 0; i < Size; i++)
		*data_prt++ = *pData++;
}
/*
 * @brief Send Message to spi through dma
 */
int hw_SPI1_SendMessage(uint16_t addr, uint8_t *pData, uint16_t Size)
{
	uint32_t ret;
	spi_control_config_t control_config = {0};
	if (spi_is_active(HPM_SPI1)) {
		//  printf("SPI is busy: -3\n");
		return -3;
	}
	/* set SPI control config for master */
	spi_master_get_default_control_config(&control_config);
	control_config.master_config.cmd_enable = false;
	control_config.master_config.addr_enable = true;
	control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
	control_config.common_config.tx_dma_enable = true;
	//control_config.common_config.rx_dma_enable = true;
	control_config.common_config.trans_mode = spi_trans_write_only;//spi_trans_write_dummy_read;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.dummy_cnt = spi_dummy_count_1;
	hw_SPI1_send_copy(addr, pData, Size);
	ret = spi_setup_dma_transfer(HPM_SPI1,
	                             &control_config,
	                             NULL, (uint32_t*)SPI1_send_ahb,
	                             Size, 1);
	if (status_success != ret)
	{
		//  printf("Send data error: %d\n", ret);
		return -1;
	}

	ret = spi_tx_trigger_dma(HW_APP_HDMA,
	                         HW_SPI_TX_DMA_CH,
	                         HPM_SPI1,
	                         core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&SPI1_send_ahb[4]),
	                         DMA_TRANSFER_WIDTH_BYTE,
	                         Size);
	if (ret != status_success) {
		//  printf("spi tx trigger dma failed\n");
	}
	return ret;
}
/*
 * @brief receive Message to spi through dma
 */
ATTR_RAMFUNC int hw_SPI1_ReceiveMessage(uint16_t addr, uint16_t Size)
{
	uint32_t ret;
	spi_control_config_t control_config = {0};
	if (spi_is_active(HPM_SPI1)) {
		//   printf("SPI is busy: -3\n");
		return -3;
	}
	uint32_t *addr_prt;
	addr_prt = (uint32_t *)SPI1_receive_ahb;
	*addr_prt = addr;
	/* set SPI control config for master */
	spi_master_get_default_control_config(&control_config);
	control_config.master_config.cmd_enable = false;
	control_config.master_config.addr_enable = true;
	control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
	//control_config.common_config.tx_dma_enable = true;
	control_config.common_config.rx_dma_enable = true;
	control_config.common_config.trans_mode = spi_trans_read_only;//spi_trans_write_dummy_read;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.dummy_cnt = spi_dummy_count_1;
	control_config.slave_config.slave_data_only = false;

	ret = spi_setup_dma_transfer(HPM_SPI1,
	                             &control_config,
	                             NULL, addr_prt,
	                             1, Size);
	if (status_success != ret)
	{
		//    printf("Receive data error: %d\n", ret);
		return -1;
	}
	ret = spi_rx_trigger_dma(HW_APP_HDMA,
	                         HW_SPI_RX_DMA_CH,
	                         HPM_SPI1,
	                         core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&SPI1_receive_ahb[4]),
	                         DMA_TRANSFER_WIDTH_BYTE,
	                         Size);
	if (ret != status_success) {
		//   printf("spi rx trigger dma failed\n");
	}
	return ret;
}

/*
 * @brief invalidate cache for reading Message
 */
void hw_SPI1_ReadMessage(volatile uint8_t *pRxData, uint16_t Size)
{
	uint8_t *data_prt;
	data_prt = (uint8_t *)&SPI1_receive_ahb[4];
	uint16_t i;
	for (i = 0; i < Size; i++)
		*pRxData++  = *data_prt++;
}
void hw_SPI1_CS_LOW(void)
{
	// no need to set CS.
	return ;
}

void hw_SPI1_CS_HIGH(void)
{
	// no need to set CS.
	return ;
}


int hw_SPI2_SendMessage(uint16_t addr, uint8_t *pData, uint16_t Size)
{
	uint32_t ret;
	spi_control_config_t control_config = {0};
	if (spi_is_active(HPM_SPI2)) {
		//  printf("SPI is busy: -3\n");
		return -3;
	}
	/* set SPI control config for master */
	spi_master_get_default_control_config(&control_config);
	control_config.master_config.cmd_enable = false;
	control_config.master_config.addr_enable = true;
	control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
	control_config.common_config.tx_dma_enable = true;
	//control_config.common_config.rx_dma_enable = true;
	control_config.common_config.trans_mode = spi_trans_write_only;//spi_trans_write_dummy_read;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.dummy_cnt = spi_dummy_count_1;
	hw_SPI2_send_copy(addr, pData, Size);
	ret = spi_setup_dma_transfer(HPM_SPI2,
	                             &control_config,
	                             NULL, (uint32_t*)SPI2_send_ahb,
	                             Size, 1);
	if (status_success != ret)
	{
		//  printf("Send data error: %d\n", ret);
		return -1;
	}

	ret = spi_tx_trigger_dma(HW_APP_HDMA,
	                         HW_SPI_TX_DMA_CH,
	                         HPM_SPI2,
	                         core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&SPI2_send_ahb[4]),
	                         DMA_TRANSFER_WIDTH_BYTE,
	                         Size);
	if (ret != status_success) {
		//  printf("spi tx trigger dma failed\n");
	}
	return ret;
}
/*
 * @brief receive Message to spi through dma
 */
ATTR_RAMFUNC int hw_SPI2_ReceiveMessage(uint16_t addr, uint16_t Size)
{
	uint32_t ret;
	spi_control_config_t control_config = {0};
	if (spi_is_active(HPM_SPI2)) {
		//   printf("SPI is busy: -3\n");
		return -3;
	}
	uint32_t *addr_prt;
	addr_prt = (uint32_t *)SPI2_receive_ahb;
	*addr_prt = addr;
	/* set SPI control config for master */
	spi_master_get_default_control_config(&control_config);
	control_config.master_config.cmd_enable = false;
	control_config.master_config.addr_enable = true;
	control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
	//control_config.common_config.tx_dma_enable = true;
	control_config.common_config.rx_dma_enable = true;
	control_config.common_config.trans_mode = spi_trans_read_only;//spi_trans_write_dummy_read;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.dummy_cnt = spi_dummy_count_1;
        control_config.common_config.cs_index = spi_cs_2 ;
	control_config.slave_config.slave_data_only = false;
        
	ret = spi_setup_dma_transfer(HPM_SPI2,
	                             &control_config,
	                             NULL, addr_prt,
	                             1, Size);
	if (status_success != ret)
	{
		//    printf("Receive data error: %d\n", ret);
		return -1;
	}
	ret = spi_rx_trigger_dma(HW_APP_HDMA,
	                         HW_SPI_RX_DMA_CH,
	                         HPM_SPI2,
	                         core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&SPI2_receive_ahb[4]),
	                         DMA_TRANSFER_WIDTH_BYTE,
	                         Size);
	if (ret != status_success) {
		//   printf("spi rx trigger dma failed\n");
	}
	return ret;
}

/*
 * @brief invalidate cache for reading Message
 */
void hw_SPI2_ReadMessage(volatile uint8_t *pRxData, uint16_t Size)
{
	uint8_t *data_prt;
	data_prt = (uint8_t *)&SPI2_receive_ahb[4];
	uint16_t i;
	for (i = 0; i < Size; i++)
		*pRxData++  = *data_prt++;
}

#if 0

//------------------------###############################--------------------------
//------------------------###############################--------------------------
//  spi3 config
//
//

void hw_SPI3_Init(void)
{
// spi_timing_config_t timing_config = {0};
// spi_format_config_t format_config = {0};

	spi_gpio_init();

	clock_add_to_group(clock_spi3, 0);
	//      clock_set_source_divider(clock_spi1, clk_src_pll0_clk0, 100U); /* 400/100 = 4MHz */
	//clock_set_source_divider(clock_spi1, clk_src_pll0_clk0, 5U); /* 80MHz */
	clock_set_source_divider(clock_spi3, clk_src_pll1_clk0, 10U); /* 80MHz */
	spi_clcok = clock_get_frequency(clock_spi3);

	/* set SPI sclk frequency for master */
	spi_master_get_default_timing_config(&timing_config);
	timing_config.master_config.clk_src_freq_in_hz = spi_clcok;
	timing_config.master_config.sclk_freq_in_hz = APP_SPI_SCLK_FREQ;
	spi_master_timing_init(HPM_SPI3, &timing_config);

	/* set SPI format config for master */
	spi_master_get_default_format_config(&format_config);
	format_config.common_config.data_len_in_bits = 8; /*8 bits data*/
	format_config.common_config.mode = spi_master_mode;
	format_config.common_config.cpol = spi_sclk_high_idle;
	format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
	format_config.master_config.addr_len_in_bytes = 2;
	spi_format_init(HPM_SPI3, &format_config);
	/* set DMAMUX for SPI master */
	//dmamux_config(TEST_SPI_DMAMUX, TEST_SPI_RX_DMAMUX_CH, TEST_SPI_RX_DMA_REQ, true);
	dmamux_config(HW_APP_DMAMUX, HW_SPI3_TX_DMAMUX_CH, HPM_DMA_SRC_SPI3_TX, true);
	dmamux_config(HW_APP_DMAMUX, HW_SPI3_RX_DMAMUX_CH, HPM_DMA_SRC_SPI3_RX, true);
}
/*
 * @brief copy Message to from app data to ahb
 */
void hw_SPI3_send_copy(uint16_t addr, volatile uint8_t *pData, uint16_t Size)
{
	uint16_t *addr_prt;
	volatile uint8_t *data_prt;
	addr_prt = (uint16_t *)SPI3_send_ahb;
	*addr_prt = addr;
	data_prt = &SPI3_send_ahb[4];
	uint16_t i;
	for (i = 0; i < Size; i++)
		*data_prt++ = *pData++;
}
/*
 * @brief Send Message to spi through dma
 */
SPI_Type SPI_copy[1];
int hw_SPI3_SendMessage(uint16_t addr, uint8_t *pData, uint16_t Size)
{
	uint32_t ret;
	spi_control_config_t control_config = {0};
	if (spi_is_active(HPM_SPI3)) {
		//printf("SPI_3 is busy: -3\n");
		memcpy((void*)SPI_copy, (const void*)HPM_SPI3, 0x28);
		return -3;
	}
	/* set SPI control config for master */
	spi_master_get_default_control_config(&control_config);
	control_config.master_config.cmd_enable = false;
	control_config.master_config.addr_enable = true;
	control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
	control_config.common_config.tx_dma_enable = true;
	//control_config.common_config.rx_dma_enable = true;
	control_config.common_config.trans_mode = spi_trans_write_only;//spi_trans_write_dummy_read;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.dummy_cnt = spi_dummy_count_1;
	hw_SPI3_send_copy(addr, pData, Size);
	ret = spi_setup_dma_transfer(HPM_SPI3,
	                             &control_config,
	                             NULL, (uint32_t*)SPI3_send_ahb,
	                             Size, 1);
	if (status_success != ret)
	{
		//  printf("Send data error: %d\n", ret);
		return -1;
	}

	ret = spi_tx_trigger_dma(HW_APP_HDMA,
	                         HW_SPI3_TX_DMA_CH,
	                         HPM_SPI3,
	                         core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&SPI3_send_ahb[4]),
	                         DMA_TRANSFER_WIDTH_BYTE,
	                         Size);
	if (ret != status_success) {
		//  printf("spi tx trigger dma failed\n");
	}
	return ret;
}

/*
 * @brief receive Message to spi through dma
 */
int hw_SPI3_ReceiveMessage(uint16_t addr, uint16_t Size)
{
	uint32_t ret;
	spi_control_config_t control_config = {0};
	if (spi_is_active(HPM_SPI3)) {
		//   printf("SPI is busy: -3\n");
		return -3;
	}
	uint32_t *addr_prt;
	addr_prt = (uint32_t *)SPI3_receive_ahb;
	*addr_prt = addr;
	/* set SPI control config for master */
	spi_master_get_default_control_config(&control_config);
	control_config.master_config.cmd_enable = false;
	control_config.master_config.addr_enable = true;
	control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
	//control_config.common_config.tx_dma_enable = true;
	control_config.common_config.rx_dma_enable = true;
	control_config.common_config.trans_mode = spi_trans_read_only;//spi_trans_write_dummy_read;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.dummy_cnt = spi_dummy_count_1;
	control_config.slave_config.slave_data_only = false;

	ret = spi_setup_dma_transfer(HPM_SPI3,
	                             &control_config,
	                             NULL, addr_prt,
	                             1, Size);
	if (status_success != ret)
	{
		//    printf("Receive data error: %d\n", ret);
		return -1;
	}
	ret = spi_rx_trigger_dma(HW_APP_HDMA,
	                         HW_SPI3_RX_DMA_CH,
	                         HPM_SPI3,
	                         core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&SPI3_receive_ahb[4]),
	                         DMA_TRANSFER_WIDTH_BYTE,
	                         Size);
	if (ret != status_success) {
		//   printf("spi rx trigger dma failed\n");
	}
	return ret;
}

#endif

