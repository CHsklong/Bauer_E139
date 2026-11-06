#ifndef SPI_H_
#define SPI_H_

typedef enum {
    spi_op_write = 0,
    spi_op_read,
    spi_op_no_data
} spi_op_t;

#define  APP_SPI_SCLK_FREQ         (10000000UL)

#if (BOARD_TYPE == BOARD_E145)
/* spi section */
#define APP_SPI_BASE              HPM_SPI2
#define APP_SPI_CLK_NAME          clock_spi2
#define APP_SPI_IRQ               IRQn_SPI2

#define APP_SPI_ADDR_LEN_IN_BYTES (1U)
#define APP_SPI_DATA_LEN_IN_BITS  (8U)
#define APP_SPI_RX_DMA HPM_DMA_SRC_SPI2_RX
#define APP_SPI_RX_DMAMUX_CH DMAMUX_MUXCFG_HDMA_MUX0
#define APP_SPI_TX_DMA HPM_DMA_SRC_SPI2_TX
#define APP_SPI_TX_DMAMUX_CH DMAMUX_MUXCFG_HDMA_MUX1
#define SPI_CS_GPIO_CTRL           HPM_GPIO0
#define SPI_CS_PIN                 IOC_PAD_PB02
#define SPI_CS_ACTIVE_LEVEL        (0U)





#define HW_APP_HDMA HPM_HDMA
#define HW_SPI_RX_DMA HPM_DMA_SRC_SPI2_RX
#define HW_SPI_TX_DMA HPM_DMA_SRC_SPI2_TX
#else
/* spi section */
#define APP_SPI_BASE              HPM_SPI1
#define APP_SPI_CLK_NAME          clock_spi1
#define APP_SPI_IRQ               IRQn_SPI1

#define APP_SPI_ADDR_LEN_IN_BYTES (1U)
#define APP_SPI_DATA_LEN_IN_BITS  (8U)
#define APP_SPI_RX_DMA HPM_DMA_SRC_SPI1_RX
#define APP_SPI_RX_DMAMUX_CH DMAMUX_MUXCFG_HDMA_MUX0
#define APP_SPI_TX_DMA HPM_DMA_SRC_SPI1_TX
#define APP_SPI_TX_DMAMUX_CH DMAMUX_MUXCFG_HDMA_MUX1
#define SPI_CS_GPIO_CTRL           HPM_GPIO0
#define SPI_CS_PIN                 IOC_PAD_PB02
#define SPI_CS_ACTIVE_LEVEL        (0U)





#define HW_APP_HDMA HPM_HDMA
#define HW_SPI_RX_DMA HPM_DMA_SRC_SPI1_RX
#define HW_SPI_TX_DMA HPM_DMA_SRC_SPI1_TX
#endif
#define HW_APP_DMAMUX HPM_DMAMUX
#define HW_SPI_RX_DMA_CH     0
#define HW_SPI_TX_DMA_CH     1
#define HW_SPI_RX_DMAMUX_CH     DMA_SOC_CHN_TO_DMAMUX_CHN(HW_APP_HDMA, 0)
#define HW_SPI_TX_DMAMUX_CH     DMA_SOC_CHN_TO_DMAMUX_CHN(HW_APP_HDMA, 1)

#define HW_SPI2_SCLK_FREQ     (2000000UL)

#define HW_SPI3_RX_DMA_CH     6
#define HW_SPI3_TX_DMA_CH     7
#define HW_SPI3_RX_DMAMUX_CH     DMA_SOC_CHN_TO_DMAMUX_CHN(HW_APP_HDMA, 6)
#define HW_SPI3_TX_DMAMUX_CH     DMA_SOC_CHN_TO_DMAMUX_CHN(HW_APP_HDMA, 7)






extern uint32_t spi_clcok;
extern uint8_t wbuff[10] ;
extern uint8_t rbuff[10] ;

extern uint8_t write[8];
extern spi_control_config_t control_config ;



extern ATTR_PLACE_AT(".ahb_sram") volatile uint8_t SPI1_send_ahb[16]  , SPI2_send_ahb[16];
extern ATTR_PLACE_AT(".ahb_sram") volatile uint8_t SPI1_receive_ahb[16] , SPI2_receive_ahb[16];;


void hw_SPI1_Init(void);
int hw_SPI1_SendMessage(uint16_t addr, uint8_t *pData, uint16_t Size);
int hw_SPI1_ReceiveMessage(uint16_t addr, uint16_t Size) ;

//-----------------------
//
//
void hw_SPI2_Init(void);
void hw_SPI2_send_copy(uint16_t addr, volatile uint8_t *pData, uint16_t Size);
int hw_SPI2_SendMessage(uint16_t addr, uint8_t *pData, uint16_t Size);
int hw_SPI2_ReceiveMessage(uint16_t addr, uint16_t Size);



#endif