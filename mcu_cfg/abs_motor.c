#include "SysMain.h"
#include "motor.h"
int8_t read_state = 0;

CALIBRATION_TYPEDEF cali_abs =
    {
        .enable = 0,
        .send_flag = 0,
};

/* CRC校验查表，公式：X8+X2+X+1 */
static uint8_t Crc8Table[256] =
    {
        0x00,
        0x07,
        0x0e,
        0x09,
        0x1c,
        0x1b,
        0x12,
        0x15,
        0x38,
        0x3f,
        0x36,
        0x31,
        0x24,
        0x23,
        0x2a,
        0x2d,
        0x70,
        0x77,
        0x7e,
        0x79,
        0x6c,
        0x6b,
        0x62,
        0x65,
        0x48,
        0x4f,
        0x46,
        0x41,
        0x54,
        0x53,
        0x5a,
        0x5d,
        0xe0,
        0xe7,
        0xee,
        0xe9,
        0xfc,
        0xfb,
        0xf2,
        0xf5,
        0xd8,
        0xdf,
        0xd6,
        0xd1,
        0xc4,
        0xc3,
        0xca,
        0xcd,
        0x90,
        0x97,
        0x9e,
        0x99,
        0x8c,
        0x8b,
        0x82,
        0x85,
        0xa8,
        0xaf,
        0xa6,
        0xa1,
        0xb4,
        0xb3,
        0xba,
        0xbd,
        0xc7,
        0xc0,
        0xc9,
        0xce,
        0xdb,
        0xdc,
        0xd5,
        0xd2,
        0xff,
        0xf8,
        0xf1,
        0xf6,
        0xe3,
        0xe4,
        0xed,
        0xea,
        0xb7,
        0xb0,
        0xb9,
        0xbe,
        0xab,
        0xac,
        0xa5,
        0xa2,
        0x8f,
        0x88,
        0x81,
        0x86,
        0x93,
        0x94,
        0x9d,
        0x9a,
        0x27,
        0x20,
        0x29,
        0x2e,
        0x3b,
        0x3c,
        0x35,
        0x32,
        0x1f,
        0x18,
        0x11,
        0x16,
        0x03,
        0x04,
        0x0d,
        0x0a,
        0x57,
        0x50,
        0x59,
        0x5e,
        0x4b,
        0x4c,
        0x45,
        0x42,
        0x6f,
        0x68,
        0x61,
        0x66,
        0x73,
        0x74,
        0x7d,
        0x7a,
        0x89,
        0x8e,
        0x87,
        0x80,
        0x95,
        0x92,
        0x9b,
        0x9c,
        0xb1,
        0xb6,
        0xbf,
        0xb8,
        0xad,
        0xaa,
        0xa3,
        0xa4,
        0xf9,
        0xfe,
        0xf7,
        0xf0,
        0xe5,
        0xe2,
        0xeb,
        0xec,
        0xc1,
        0xc6,
        0xcf,
        0xc8,
        0xdd,
        0xda,
        0xd3,
        0xd4,
        0x69,
        0x6e,
        0x67,
        0x60,
        0x75,
        0x72,
        0x7b,
        0x7c,
        0x51,
        0x56,
        0x5f,
        0x58,
        0x4d,
        0x4a,
        0x43,
        0x44,
        0x19,
        0x1e,
        0x17,
        0x10,
        0x05,
        0x02,
        0x0b,
        0x0c,
        0x21,
        0x26,
        0x2f,
        0x28,
        0x3d,
        0x3a,
        0x33,
        0x34,
        0x4e,
        0x49,
        0x40,
        0x47,
        0x52,
        0x55,
        0x5c,
        0x5b,
        0x76,
        0x71,
        0x78,
        0x7f,
        0x6a,
        0x6d,
        0x64,
        0x63,
        0x3e,
        0x39,
        0x30,
        0x37,
        0x22,
        0x25,
        0x2c,
        0x2b,
        0x06,
        0x01,
        0x08,
        0x0f,
        0x1a,
        0x1d,
        0x14,
        0x13,
        0xae,
        0xa9,
        0xa0,
        0xa7,
        0xb2,
        0xb5,
        0xbc,
        0xbb,
        0x96,
        0x91,
        0x98,
        0x9f,
        0x8a,
        0x8d,
        0x84,
        0x83,
        0xde,
        0xd9,
        0xd0,
        0xd7,
        0xc2,
        0xc5,
        0xcc,
        0xcb,
        0xe6,
        0xe1,
        0xe8,
        0xef,
        0xfa,
        0xfd,
        0xf4,
        0xf3,
};

/*!
 * @brief  CRC校验：X8+X2+X+1
 * @return CRC校验值
 */
uint8_t Crc_Table(uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0x00; // 计算的初始crc值，也存放结果
    while (len--)
    {
        crc ^= *ptr++; // 每次先与需要计算的数据异或,计算完指向下一数据
        crc = Crc8Table[crc];
    }
    return (crc);
}

uint8_t ABS_MOTOR_Get_MT6835_Clib_Send_flag(void)
{
    return cali_abs.send_flag;
}

void ABS_MOTOR_Clear_MT6835_Clib_Send_flag(void)
{
    cali_abs.send_flag = 0;
}
void ABS_MOTOR_Init(void)
{
    spi_gpio_init();
    #if (BOARD_TYPE == BOARD_E139 || BOARD_TYPE == BOARD_E884)
        hw_SPI1_Init();
    #elif (BOARD_TYPE == BOARD_E914)
        hw_SPI1_Init();
    #elif (BOARD_TYPE == BOARD_E145)
        hw_SPI2_Init();
    #endif
}
ATTR_RAMFUNC void ABS_MOTOR_MT6835_clib_begin(void)
{
    cali_abs.enable = 1;
    ABS_CAL_START();
    enableMotor(0);
}
uint8_t ABS_MOTOR_Get_MT6835_Clib_Result(void)
{
    return cali_abs.ret;
}

ATTR_RAMFUNC void ABS_MOTOR_MT6835_clibing(void)
{
    switch (cali_abs.state)
    {
    case CLIB_STATE_IDLE:
        if (cali_abs.enable == 1)
        {
            if (cali_abs.cnt > 10)
                cali_abs.state = CLIB_STATE_READ_CLIB_RESULT;
            else
                cali_abs.cnt++;
        }
        break;
    case CLIB_STATE_READ_CLIB_RESULT:
        cali_abs.cnt = 0;
#if (BOARD_TYPE == BOARD_E139 || BOARD_TYPE == BOARD_E884)
        hw_SPI1_ReceiveMessage(0x3113, 1);
#elif (BOARD_TYPE == BOARD_E914)
        hw_SPI1_ReceiveMessage(0x3113, 1);
#elif (BOARD_TYPE == BOARD_E145)
        hw_SPI2_ReceiveMessage(0x3113, 1);
#endif

        cali_abs.state = CLIB_STATE_RESV_CLIB_RESULT;
        break;
    case CLIB_STATE_RESV_CLIB_RESULT:
#if (BOARD_TYPE == BOARD_E139|| BOARD_TYPE == BOARD_E884)
        cali_abs.status113 = (SPI1_receive_ahb[4] >> 6) & 0x03; // bit6 bit7
#elif (BOARD_TYPE == BOARD_E914)
        cali_abs.status113 = (SPI1_receive_ahb[4] >> 6) & 0x03; // bit6 bit7
#elif (BOARD_TYPE == BOARD_E145)
        cali_abs.status113 = (SPI2_receive_ahb[4] >> 6) & 0x03; // bit6 bit7
#endif

        if (cali_abs.status113 == 3) // 校准成功
        {
            ABS_CAL_FINISH();
            if (cali_abs.cnt > 6000 * 20) // 6s
                cali_abs.state = CLIB_STATE_READ_SAVE_EPP_RESULT;
            else
                cali_abs.cnt++;
        }
        else if (cali_abs.status113 == 0) // 未开始校准
        {
            cali_abs.state = CLIB_STAT_CLIB_FAIL;
        }
        else if (cali_abs.status113 == 1) // 正在校准
        {
            cali_abs.state = CLIB_STATE_READ_CLIB_RESULT;
        }
        else if (cali_abs.status113 == 2) // 校准失败
        {
            cali_abs.state = CLIB_STAT_CLIB_FAIL;
        }

        break;
    case CLIB_STATE_READ_SAVE_EPP_RESULT:
#if (BOARD_TYPE == BOARD_E139|| BOARD_TYPE == BOARD_E884)
        hw_SPI1_ReceiveMessage(0x3112, 1);
#elif (BOARD_TYPE == BOARD_E914)
        hw_SPI1_ReceiveMessage(0x3112, 1);
#elif (BOARD_TYPE == BOARD_E145)
        hw_SPI2_ReceiveMessage(0x3112, 1);
#endif
        cali_abs.state = CLIB_STATE_RESV_SAVE_EPP_RESULT;
        break;
    case CLIB_STATE_RESV_SAVE_EPP_RESULT:
#if (BOARD_TYPE == BOARD_E139|| BOARD_TYPE == BOARD_E884)
        cali_abs.status112 = (SPI1_receive_ahb[4] >> 5) & 0x01; // bit5
#elif (BOARD_TYPE == BOARD_E914)
        cali_abs.status112 = (SPI1_receive_ahb[4] >> 5) & 0x01; // bit5
#elif (BOARD_TYPE == BOARD_E145)
        cali_abs.status112 = (SPI2_receive_ahb[4] >> 5) & 0x01; // bit5
#endif

        if (cali_abs.status112 == 1)
        {
            cali_abs.state = CLIB_STATE_CLIB_SUCCESS;
        }
        break;
    case CLIB_STAT_CLIB_FAIL:
        cali_abs.ret = 2;
        cali_abs.enable = 0;
        cali_abs.state = CLIB_STATE_IDLE;
        cali_abs.send_flag = 1;
        ABS_CAL_FINISH();
        // cali_abs.status112 =0 ;
        // cali_abs.status113 =0 ;
        break;
    case CLIB_STATE_CLIB_SUCCESS:
        cali_abs.ret = 1;
        cali_abs.enable = 0;
        cali_abs.state = CLIB_STATE_IDLE;
        cali_abs.send_flag = 1;
        // cali_abs.status112 =0 ;
        // cali_abs.status113 =0 ;
        break;
    }
}

ATTR_RAMFUNC void ABS_MOTOR_DAM_TX_START(void)
{
    if (cali_abs.enable == 0) // 无校准时读角度值
    {
#if (BOARD_TYPE == BOARD_E139|| BOARD_TYPE == BOARD_E884)
        hw_SPI1_ReceiveMessage(0xA003, 4);
#elif (BOARD_TYPE == BOARD_E914)
        hw_SPI1_ReceiveMessage(0xA003, 4);
#elif (BOARD_TYPE == BOARD_E145)
        hw_SPI2_ReceiveMessage(0xA003, 4);
#endif
    }
    else
        ABS_MOTOR_MT6835_clibing();
}
uint32_t resv_error =0 ;
ATTR_RAMFUNC uint32_t ABS_MOTOR_READ_ANGLE(void)
{
    static uint32_t abs_ret = 0;

    uint8_t RcvBuf[4] = {0};
    uint8_t crc_abs = 0;
  
    if (cali_abs.enable != 0)
        return abs_ret;
#if (BOARD_TYPE == BOARD_E139|| BOARD_TYPE == BOARD_E884)
    RcvBuf[0] = SPI1_receive_ahb[4];
    RcvBuf[1] = SPI1_receive_ahb[5];
    RcvBuf[2] = SPI1_receive_ahb[6];
    RcvBuf[3] = SPI1_receive_ahb[7];
#elif (BOARD_TYPE == BOARD_E914)
    RcvBuf[0] = SPI1_receive_ahb[4];
    RcvBuf[1] = SPI1_receive_ahb[5];
    RcvBuf[2] = SPI1_receive_ahb[6];
    RcvBuf[3] = SPI1_receive_ahb[7];
#elif (BOARD_TYPE == BOARD_E145)
    RcvBuf[0] = SPI2_receive_ahb[4];
    RcvBuf[1] = SPI2_receive_ahb[5];
    RcvBuf[2] = SPI2_receive_ahb[6];
    RcvBuf[3] = SPI2_receive_ahb[7];
#endif

    crc_abs = Crc_Table((uint8_t *)RcvBuf, 3);

    // 数据处理  0.5us

    if (crc_abs == RcvBuf[3])
    {
        uint32_t spiread = ((RcvBuf[0] << 13) | (RcvBuf[1] << 5) | (RcvBuf[2] >> 3));

        abs_ret = spiread >> (21 - 17);
    }
    else
    {
        resv_error ++ ;
    }
    return abs_ret;

    //    ABS_EEP_READ_START();
}

//----------------------------------------------------------------------------------------
/*
void spi_master_data_dump(spi_op_t op, uint32_t datalen, uint8_t *buff,  uint32_t size)
{
    uint32_t i;

    if (op == spi_op_no_data) {
        printf("SPI-Master no data transfer.\n");
        return;
    }

    if (buff == NULL || size == 0) {
        return;
    }

    printf("SPI-Master %s", op == spi_op_write ? "write data: " : "read  data: ");

    for (i = 0; i < size; i++) {
        if (datalen <= 8) {
            printf("0x%02x ", *(uint8_t *)buff);
            buff += 1;
        } else if (datalen <= 16) {
            printf("0x%02x ", *(uint16_t *)buff);
            buff += 2;
        } else {
            printf("0x%02x ", *(uint32_t *)buff);
            buff += 4;
        }
    }
    printf("\n");
}


void SPI_READ_MT6835(void)
{
    wbuff[0] = 0xA0;
    wbuff[1] = 0x03 ;
    spi_transfer(HPM_SPI1,
                 &control_config,
                 NULL, NULL,
                 (uint8_t *)wbuff, 2, (uint8_t *)rbuff, 4);


    mt_abs.crc_abs =  Crc_Table((uint8_t *)rbuff, 3);

    if (mt_abs.crc_abs == rbuff[3])
    {
        mt_abs.abs_angle = ((rbuff[0] << 13) | (rbuff[1] << 5) | (rbuff[2] >> 3)) ; //>> 5;
        mt_abs.status  = rbuff[2] & 0x07 ;
    }
}
*/
void SPI_TEST_MT6835(void)
{
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;                /* cmd phase control for master */
    control_config.master_config.addr_enable = false;               /* address phase control for master */
    control_config.common_config.trans_mode = spi_trans_write_read; // spi_trans_write_read_together;

    wbuff[0] = 0x30;
    wbuff[1] = 0x13;
    spi_transfer(HPM_SPI1,
                 &control_config,
                 NULL, NULL,
                 (uint8_t *)wbuff, 2, (uint8_t *)rbuff, 1);
}

void SPI_WIRTE_TEST_MT6835(void)

{
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;                /* cmd phase control for master */
    control_config.master_config.addr_enable = false;               /* address phase control for master */
    control_config.common_config.trans_mode = spi_trans_write_only; // spi_trans_write_read ; //spi_trans_write_read_together;

    wbuff[0] = 0x60;
    wbuff[1] = 0x13;
    wbuff[1] = 0x55;
    spi_transfer(HPM_SPI1,
                 &control_config,
                 NULL, NULL,
                 (uint8_t *)wbuff, 3, (uint8_t *)rbuff, 0);
}
//----------------------------------------------------------------------------------
