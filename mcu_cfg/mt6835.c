/**
  ******************************************************************************
  * @file    MT6816.h
  * @author  @mengxiang
  * @version V0.0
  * @date    2020.07.12
  * @brief   MT6816 driver
  ******************************************************************************
  * @attention	IIF-type: MT6816B E1000
	*Incremental Interface A/B mode.
	*12bit mode, one count per 0.088?angle step.
	*Absolute count enabled.
	*Autocalibration mode 1 enabled.
	*Prediction disabled.
	*Hysteresis set to 0.703?
	*IFA/IFB/IFC pins set to push-pull output.
	*SSC interface's DATA pin set to push-pull output.
	*IFA/IFB/IFC pins set to strong driver, DATA pin set to strong driver, fast edge.
	*Voltage spike filter on input pads disabled.
  *  
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//#include "board.h"
#include "mt6835.h"
//#include "mathlib.h"

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE (10)

//4bit command + 12bit address.
#define READ_CMD                (0x30)
#define WRITE_CMD               (0x60)
#define PROG_CMD                (0xC0)
#define AUTO_SET_ZERO_CMD       (0x50)
#define BUST_READ_CMD           (0xA0)

/*MT6823 register define*/
#define READ_ANGLE_VALUE1        (0x003)
#define READ_ANGLE_VALUE2        (0x004)
#define READ_ANGLE_VALUE3        (0x005)

#define ADDR_ABZ_RES1            (0x007)
#define ADDR_ABZ_RES2            (0x008)
#define ADDR_CAL_STATE           (0x113)

#define ABZ_OFF     (0)
#define ABZ_SWAP    (0)

/* Private variables ---------------------------------------------------------*/
    
 uint8_t txData[BUFFER_SIZE];
 uint8_t rxData[BUFFER_SIZE];
 uint16_t cur_lines;

/*!
* @brief   void MT6816_Init(void)
*           - Initialization of the SPI peripheral for MT6816
*             pre-driver
*           - SPI configuration for ???????? MT6816
*
* @param   void
*
* @return  none
*/
void MT6835_Init(void)
{
    //ABZ 输出 250线
    MT6835_ABZ_Config(250);
}

void MT6835_ABZ_Config(uint32_t lines)
{
    lines --;

    //ABZ 输出 lines线
    txData[0] = WRITE_CMD + ((ADDR_ABZ_RES1>>8)&0xf);
    txData[1] = (ADDR_ABZ_RES1&0xff);
    txData[2] = (lines>>6)&0xf;
    txData[3] = 0;
//    BOARD_SPI_WrRd(0, txData, &rxData[0], 4); 

    txData[0] = WRITE_CMD + ((ADDR_ABZ_RES2>>8)&0xf);
    txData[1] = (ADDR_ABZ_RES2&0xff);
    txData[2] = ((lines&0xff)<<2) + (ABZ_OFF<<1) + ABZ_SWAP;
    txData[3] = 0;
  //  BOARD_SPI_WrRd(0, txData, &rxData[0], 4); 

	
    txData[0] = READ_CMD + ((ADDR_ABZ_RES1>>8)&0xf);
    txData[1] = (ADDR_ABZ_RES1&0xff);
    txData[2] = 0;
    txData[3] = 0;
 //   BOARD_SPI_WrRd(0, txData, &rxData[0], 4);
    cur_lines = (rxData[2]<<6);

    txData[0] = READ_CMD + ((ADDR_ABZ_RES2>>8)&0xf);
    txData[1] = (ADDR_ABZ_RES2&0xff);
    txData[2] = 0;
    txData[3] = 0;
//    BOARD_SPI_WrRd(0, txData, &rxData[0], 4);

    cur_lines += (rxData[2]>>2);

}

// 32bit TODO: crc
uint32_t MT6835_Read_Angle(void)
{
    uint32_t ang = 0;
    uint8_t crc;
    static uint32_t old_ang = 0;

    txData[0] = BUST_READ_CMD + ((READ_ANGLE_VALUE1>>8)&0xf);
    txData[1] = (READ_ANGLE_VALUE1&0xff);
    txData[2] = 0;
    txData[3] = 0;
    BOARD_SPI_WrRd(0, txData, &rxData[0], 6);

#ifdef FEATURE_ENC_POS_32BIT
    //21bit -> 32bit
    ang = ((rxData[2]<<13) | (rxData[3]<<5) | (rxData[4] >> 3)) << 11;
#else 
    //21bit -> 16bit
    ang = ((rxData[2]<<13) | (rxData[3]<<5) | (rxData[4] >> 3)) >> 5;
#endif

//    crc = crc_8_poly(&rxData[2], 3, 0x7, 0);

    if(crc == rxData[5])
    {
        old_ang = ang;
        return ang;
    }

    return old_ang;
}

void MT6835_auto_cal(uint8_t enb)
{
//    if(enb)
//        GPIO_PortSet(BOARD_INITPINS_ENC_CAL_PERIPHERAL, 1U<<BOARD_INITPINS_ENC_CAL_CHANNEL);
//   else
//      GPIO_PortClear(BOARD_INITPINS_ENC_CAL_PERIPHERAL, 1U<<BOARD_INITPINS_ENC_CAL_CHANNEL);
}

uint8_t MT6835_cal_state(void)
{
    txData[0] = READ_CMD + ((ADDR_CAL_STATE>>8)&0xf);
    txData[1] = (ADDR_CAL_STATE&0xff);
    txData[2] = 0;
    txData[3] = 0;
    BOARD_SPI_WrRd(0, txData, &rxData[0], 4);
    
    return (rxData[2]>>6) & 0x3;
}





