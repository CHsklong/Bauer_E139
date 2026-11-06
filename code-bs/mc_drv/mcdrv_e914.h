/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _MCDRV_FRDM_KV31F512_H_
#define _MCDRV_FRDM_KV31F512_H_

#include "mcdrv_pwm3ph_ftm.h"
#include "mcdrv_adc_adc12.h"
#include "mcdrv_ftm_enc.h"
#include "mcdrv_ftm_hall.h"
#include "m1_pmsm_appconfig.h"
#include "mt6835.h"
#include "mid_def.h"
#include "m1_sm_ref_sol.h"
#include "abs_motor.h"
#include "UserAbs.h"
#include "abs_motor.h"
#include "gpio.h"
#include "param.h"
#include "mcdrv.h"
#include "user_app_fun.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Version info */
#define MCRSP_VER       "v1.0"        /* motor control package version */
//#define TEST_SYNC 1
#define ENCODER_SYNC 1
//#define SPI_WR_BLOCKING  1

#if (BOARD_TYPE == BOARD_E139)
       #define DEVICE_ID 0x4054
#elif (BOARD_TYPE == BOARD_E884)
       #define DEVICE_ID 0x4884
#elif (BOARD_TYPE == BOARD_E914)
       #define DEVICE_ID 0x4914
#elif (BOARD_TYPE == BOARD_E145)
/*#ifdef JACKING_MOTOR_FUN
      #define DEVICE_ID 0x4054
#else
       #define DEVICE_ID 0x4145
#endif*/
#define DEVICE_ID 0x4054
#endif


/* Application info */
typedef struct _app_ver
{
    char    cBoardID[15];
    char    cMotorType[4];
    char    cMotorID[15];
    char    cAppVer[7];
}app_ver_t;
   
/* Structure used during clocks and modulo calculations */
typedef struct _clock_setup
{
    uint32_t ui32SystemClock;
    uint32_t ui32BusClock;
    uint16_t ui16PwmFreq;
    uint16_t ui16PwmModulo;
    uint16_t ui16SpeedLoopFreq;
    uint16_t ui16SpeedLoopModulo;
} clock_setup_t;
#ifndef PI
#define PI (3.14159265358979F)
#endif
/******************************************************************************
 * Clock & PWM definition
 ******************************************************************************/
#define M1_USE_MCU_DEADTIME         (1)
//#define M1_PWM_FREQ (10000)         /* PWM frequency - 10kHz */
#define M1_PWM_FREQ (20000)         /* PWM frequency - 20kHz */
#define M1_FOC_FREQ_VS_PWM_FREQ (1) /* FOC calculation is called every n-th PWM reload */
//#define M1_SPEED_LOOP_FREQ (10000)   /* Speed loop frequency */  //10000
//#define M1_SPEED_LOOP_FREQ (10000)   /* Speed loop frequency */  //10000
#define M1_SPEED_LOOP_FREQ (20000) 
//#define M1_SPEED_LOOP_FREQ (5000)
#define M1_FAST_LOOP_TS ((float_t)1.0/(float_t)(M1_PWM_FREQ / M1_FOC_FREQ_VS_PWM_FREQ))
#define M1_SLOW_LOOP_TS ((float_t)1.0/(float_t)(M1_SPEED_LOOP_FREQ))
#define M1_TIME_ONESEC_COUNT (M1_PWM_FREQ / M1_FOC_FREQ_VS_PWM_FREQ)

/* Assignment of FTM channels to motor phases
 * 0 - FTM channels 0&1
 * 2 - FTM channels 2&3
 * 4 - FTM channels 4&5
 */
#define M1_PWM_PAIR_PHA (0)
#define M1_PWM_PAIR_PHB (2)
#define M1_PWM_PAIR_PHC (4)

/* Braking resistor macros */
#define M1_BRAKE_SET()
#define M1_BRAKE_CLEAR()

/* Over Current Fault detection */
#define M1_FAULT_NUM (2)

/******************************************************************************
 * MC driver macro definition and check - do not change this part
 ******************************************************************************/
#define ADC_OFFSET_WINDOW (3)
/* Phase current A assingned to ADC0 only */
#define M1_ADC0_PH_A                   (0)
#define M1_ADC1_PH_A                   (MCDRV_CHAN_OFF)
/* Phase current A assingned to ADC1 only */
#define M1_ADC0_PH_B                   (1)
#define M1_ADC1_PH_B                   (MCDRV_CHAN_OFF)
/* Phase current A assingned to ADC0 and ADC1 */
#define M1_ADC0_PH_C                   (MCDRV_CHAN_OFF)
#define M1_ADC1_PH_C                   (0)
/* Udc bus voltage is assigned to ADC0 only */
#define M1_ADC0_UDCB                   (MCDRV_CHAN_OFF)
#define M1_ADC1_UDCB                   (1)
/* Aux channel is assigned to ADC1 only */
#define M1_ADC0_AUX                    (MCDRV_CHAN_OFF)
#define M1_ADC1_AUX                    (MCDRV_CHAN_OFF)
#define USER_LED_ON()                    SYS_LED_LOW()
#define USER_LED_OFF()                   SYS_LED_HIGH()
#define USER_LED_TOGGLE()                SYS_LED_TOG()


#define DISABLE_ALL_IRQ(par)        {par =  disable_global_irq(CSR_MSTATUS_MIE_MASK);disable_global_irq(CSR_MSTATUS_UIE_MASK);} 

#define ENABLE_ALL_IRQ(par)   restore_global_irq(par)
/******************************************************************************
 * Define motor 1 ADC control functions
 ******************************************************************************/
#define M1_MCDRV_ADC_PERIPH_INIT() (InitADC12())
#define M1_MCDRV_ADC_GET(par) \
    MCDRV_Curr3Ph2ShGet(par); \
    MCDRV_VoltDcBusGet(par);  \
    MCDRV_AuxValGet(par);
#define M1_MCDRV_CURR_3PH_CHAN_ASSIGN(par) (MCDRV_Curr3Ph2ShChanAssign(par))
#define M1_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
#define M1_MCDRV_CURR_3PH_CALIB(par) (MCDRV_Curr3Ph2ShCalib(par))
#define M1_MCDRV_CURR_3PH_CALIB_SET(par) (MCDRV_Curr3Ph2ShCalibSet(par))

/******************************************************************************
 * Define motor 1 3-ph PWM control functions
 ******************************************************************************/
#define M1_MCDRV_PWM_PERIPH_INIT() (InitFTM0())
#define M1_MCDRV_PWM3PH_SET(par) (MCDRV_FtmPwm3PhSet(par))
#define M1_MCDRV_PWM3PH_EN(par) (MCDRV_FtmPwm3PhOutEn(par))
#define M1_MCDRV_PWM3PH_DIS(par) (MCDRV_FtmPwm3PhOutDis(par))
#define M1_MCDRV_PWM3PH_FLT_GET(par) (MCDRV_FtmPwm3PhFltGet(par))

/******************************************************************************
 * Define motor 1 slow control loop timer
 ******************************************************************************/
#define M1_MCDRV_TMR_SLOWLOOP_INIT() InitFTM3()

/******************************************************************************
 * Define motor 1 ADC trigger PDB
 ******************************************************************************/
#define M1_MCDRV_PDB_INIT() InitPDB()

/******************************************************************************
 * Define motor 1 CMP for overcurrent detection                               
 ******************************************************************************/
#define M1_MCDRV_CMP_INIT() InitCMP()
    
/******************************************************************************
 * Define motor 1 FTM sensor                                                  
 ******************************************************************************/    
#define M1_MCDRV_TMR_SENSOR_INIT() InitFTM1()

/******************************************************************************
 * Define speed sensor
 ******************************************************************************/ 
#define M1_MCDRV_ENC_GET(par)                (MCDRV_FtmEncGet(par))
#define M1_MCDRV_ENC_CLEAR(par)              (MCDRV_FtmEncClear(par))
#define M1_MCDRV_ENC_SET_POSME(par,par2)     (MCDRV_FtmEncSetPosMe(par,par2))
#define M1_MCDRV_ENC_GET_CNT(par)            (par=MCDRV_FtmEncGetCnt(&g_sM1Enc))

#ifndef SPI_WR_BLOCKING
#define M1_MCDRV_ENC_GET_ABS(par)            {par =(ABS_MOTOR_READ_ANGLE() >>3) ;}
#define M1_MCDRV_ENC_READ_TRANS_NO_BLOCKING()              (ABS_MOTOR_DAM_TX_START())
#else
#define M1_MCDRV_ENC_GET_ABS(par)            (par=(MT6816_Read_Angle()))
#endif
#define M1_MCDRV_WRITE_ALIGN_DATA()            (writeAlignData())
#define M1_MCDRV_ENC_ENB_Z_INT()             (MCDRV_EnableEncZInt())
#define M1_MCDRV_ENC_OUT_CFG(lINES)           (MT6816_ABZ_Config((lINES)))
#define M1_MCDRV_ENC_AUTO_CAL(eNB)           
#define M1_MCDRV_ENC_AUTO_CAL_STATE(sTATE)    

#define M1_MCDRV_FTM_HALL_GET_SPEED_POS(par)  
#define M1_MCDRV_FTM_HALL_SET_INIT_POS(par)   
#define M1_MCDRV_FTM_HALL_CLEAR(par)          

#define M1_MCDRV_GET_SENSOR_TYPE              kEncoder
      

/******************************************************************************
 * Global variable definitions
 ******************************************************************************/
extern mcdrv_ftm_pwm3ph_t g_sM1Pwm3ph;
extern mcdrv_adc12_t g_sM1AdcSensor;
extern clock_setup_t g_sClockSetup;
extern mcdrv_ftm_hs_t g_sM1HsTmr;
extern mcdrv_ftm_enc_t g_sM1Enc;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

void MCDRV_Init_M1(void);
void MCDRV_Init_gpio(void);
void InitFTM0(void);
void InitFTM1(void);
void InitFTM3(void);
void InitPDB(void);
void InitADC12(void);
void InitCMP(void);
uint32_t InitClock(void);
void InitRelay(void);
void BoardTest(uint32_t ui32PrimaskReg);
uint32_t InitEncConfigData(void);
void MCDRV_EnableEncZInt(void);
void ADCA_IRQHandler(void);
void motEncCalibrate(void);
bool BOARD_CanSend(uint32_t id, uint8_t *cmd, uint8_t len);
uint8_t BOARD_Input(uint8_t type);
int32_t board_get_jr_motor_encoder(void) ;
void board_ms_loop(void);
uint32_t board_get_ms_tick(void);
void BOARD_Test_init(void);
void BOARD_Output(uint8_t value) ;
uint32_t  BOARD_GetUID(uint8_t uid[12]) ;
bool BOARD_Can1Send(uint32_t id, uint8_t *cmd, uint8_t len);
#ifdef __cplusplus
}
#endif
#endif /* _MCDRV_FRDM_KV31F512_H_  */
