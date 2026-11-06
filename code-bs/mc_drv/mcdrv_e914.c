/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#include "board.h"
#include "m1_pmsm_align.h"
#include "mcdrv.h"
#include "mt6835.h"
// #include "tick.h"
#include "adc.h"
#include "can.h"
#include "encode.h"
#include "m1_sm_ref_sol.h"
#include "machine.h"
#include "motor.h"
#include "pwm.h"
#include "work_io.h"
#include "hal_can_encrypt.h"
#include "abs_motor.h"
#include "gpio.h"
#include "ladrc.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* CPU load measurement using Systick*/
#define OTP_CHIP_UUID_IDX_START (88U)
#define OTP_CHIP_UUID_IDX_END (91U)
uint32_t g_ui32NumberOfCycles = 0;
uint32_t g_ui32MaxNumberOfCycles = 0;
uint32_t board_ms_cnt = 0;
int32_t g_encoder = 0;
uint16_t u_av_adc, v_av_adc, w_av_adc;
u16 MainTaskTicker1ms = 0;
/* Configuration structure for 3-phase PWM driver */
mcdrv_ftm_pwm3ph_init_t g_sM1Pwm3phInit;

/* Structure for 3-phase PWM mc driver */
mcdrv_ftm_pwm3ph_t g_sM1Pwm3ph;

/* Structure for current and voltage measurement */
mcdrv_adc12_t g_sM1AdcSensor;

/* Structure for Hall sensors driver */
mcdrv_ftm_hs_t g_sM1HsTmr;

/* Structure for Encoder driver */
mcdrv_ftm_enc_t g_sM1Enc = {.encAlignData.sAbcAlignData.s16AlignPos[0] = ALIGN_TABLE_DATA};

/* Clock setup structure */
clock_setup_t g_sClockSetup;

/* 80MHz, 1/80MHz * 96 = 1.2us */
#define M1_DEADTIME (8)
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief   void MCDRV_Init_M1(void)
 *           - Motor control driver main initialization
 *           - Calls initialization functions of peripherals required for motor
 *             control functionality
 *
 * @param   void
 *
 * @return  none
 */
void MCDRV_Init_M1(void)
{
  /* Init application clock dependent variables */
  // InitClock();

  /* Init ADC */
  M1_MCDRV_ADC_PERIPH_INIT();

  /* Init PDB */
  //  M1_MCDRV_PDB_INIT();

  /* Init FTM3 (slow loop counter) */
  //    M1_MCDRV_TMR_SLOWLOOP_INIT();

  /*init tle5012 encoder ic*/

  ABS_MOTOR_Init();
  /* Init FTM1 */
  M1_MCDRV_TMR_SENSOR_INIT();

  /* 6-channel PWM peripheral init */
  M1_MCDRV_PWM_PERIPH_INIT();

  /* Comparator CMP1 */
  M1_MCDRV_CMP_INIT();
  over_current_flag = 0;
  TD_Init(&td_set_spd, 0.000165, 880, 15000); // BANDWIDTH=20Hz
}

void MCDRV_Init_gpio(void)
{
  led_gpio_init();
  io_gpio_init();
  can_gpio_init();
}
/*!
 * @brief      Core, bus, flash clock setup
 *
 * @param      void
 *
 * @return     uint32_t
 */
uint32_t InitClock(void)
{
  return clock_get_frequency(clock_mot0);
}

/*!
 * @brief   void InitFTM0(void)
 *           - Initialization of the ftm0 peripheral for motor M1
 *           - 3-phase center-aligned PWM
 *
 * @param   void
 *
 * @return  none
 */
void InitFTM0(void)
{

  g_sM1Pwm3phInit.ui16FaultFixNum = M1_FAULT_NUM; /* FTM Fault number for over current fault detection */
  pwm_init();
  // pwm_pins_init(HPM_PWM1);
  /* Pass initialization structure to the mc driver */
  MCDRV_FtmPwm3PhInit(&g_sM1Pwm3ph, &g_sM1Pwm3phInit); /* MC driver initialization */
}

/*!
 * @brief      Initialization of the PDB for current and voltage sensing
 *
 * @param      void
 *
 * @return     none
 */
void InitPDB(void)
{
}

/*!
 * @brief   void InitFTM3(void)
 *           - Initialization of the FTM3 peripheral
 *           - performs slow control loop counter
 *
 * @param   void
 *
 * @return  none
 */
void InitFTM3(void)
{
}

/***********************************************************************************************************************
 * ENC0 initialization code
 **********************************************************************************************************************/

void ENC0_init(void)
{
  /* ENC initialization */
}
void InitFTM1(void)
{

  //  ENC0_init();
  // FTM1_init() ;
  InitEncConfigData();
}

uint32_t InitEncConfigData(void)
{
  /* Pass initialization data into encoder driver structure */
  /* encoder position and speed measurement */
  //  g_sM1Enc.pui32FtmBase = (FTM_Type *) FTM1;
  g_sM1Enc.sTo.fltPGain = M1_POSPE_TO_KP_GAIN;
  g_sM1Enc.sTo.fltIGain = M1_POSPE_TO_KI_GAIN;
  g_sM1Enc.sTo.fltThGain = M1_POSPE_TO_INTEG_GAIN;
  g_sM1Enc.a32PosMeGain = M1_POSPE_MECH_POS_GAIN;
  g_sM1Enc.f16PosMe = 0;
  g_sM1Enc.f16PosMeEst = 0;
  g_sM1Enc.fltSpdMeEst = 0;
  g_sM1Enc.sTo.f32Theta = 0;
  g_sM1Enc.sTo.fltSpeed = 0;
  g_sM1Enc.sTo.fltI_1 = 0;

  g_sM1Enc.bNeedAlign = 0;
  #if (BOARD_TYPE == BOARD_E145)
  g_sM1Enc.i8EncDir = (motor.dir > 0) ? 1 : -1;
  #else
  #if (BOARD_TYPE == BOARD_E914)
  g_sM1Enc.i8EncDir =(M1_POSPE_ENC_DIR > 0) ? 1 : -1;
  #else
  g_sM1Enc.i8EncDir = (motor.dir > 0) ? 1 : -1;
  #endif

  #endif  
  g_sM1Enc.i16ZCnt = 0;
  g_sM1Enc.encType = ENC_TYPE_ABS;
#if (BOARD_TYPE == BOARD_E145)
  for (int i = 0; i < ALIGN_TABLE_SIZE; i++)
  {
    g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] = g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][(i + 1) % ALIGN_TABLE_SIZE] - g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i];
    if (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] < -8192)
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] += 16384;
    if (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] > 8192)
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] -= 16384;
  }
#else
  for (int i = 0; i < ALIGN_TABLE_SIZE; i++)
  {
    g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] = g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][(i + 1) % ALIGN_TABLE_SIZE] - g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i];
    if (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] < -8192)
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] += 16384;
    if (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] > 8192)
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] -= 16384;
  }
#endif
  /*	for (int i = 0; i < ALIGN_TABLE_SIZE; i++)
{

                   if(g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[i] >0)
                                 g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[i] -= 16384;


} */
  g_sM1Enc.encAlignData.sAbcAlignData.s16Zero = motor_run.zero_offset;
  // g_sM1Enc.encAlignData.sAbcAlignData.s16Zero = motor_run.zero_offset ;
  // g_sM1Enc.encAlignData.sAbcAlignData.s16Zero = 0;
  return 0;
}

/*!
 * @brief   void InitADC12(void)
 *           - Initialization of the ADC12 peripheral
 *           - Initialization of the A/D converter for current and voltage sensing
 *
 * @param   void
 *
 * @return  none
 */
void InitADC12(void)
{
  adc_pins_init_analog();
  hw_ADC0_squ_Init();
  hw_ADC1_squ_Init();

  // g_sM1AdcSensor.bIDir = false;

  /* Assign channels and init all pointers */
  MCDRV_Curr3Ph2ShChanAssignInit(&g_sM1AdcSensor);
}

/*!
@brief   void InitCMP(void)
          - Initialization of the comparator 1 module for dc-bus over current
            detection to generate FTM0 fault

@param   void

@return  none
*/
void InitCMP(void)
{
}

/*!
 *@brief      Set Inrush relay on HVP
 *
 *@param      none
 *
 *@return     none
 */
void InitRelay(void)
{
}

void MCDRV_EnableEncZInt(void)
{
}

void BOARD_loop_isr(void) // 100us
{
#if 0
#ifndef ENCODER_SYNC
#ifndef _BOOT
  machine_sync_test();
#endif
#else
#ifdef TEST_SYNC
  if (machine_get_oc()) {
    // m_Jcur_encoder += m_Jv / 600000.0f * CIRCLE_ENCODER ; // RPM *CIRCLE_ENCODER / 60000 * ftime= encoder
    m_Jcur_encoder += m_Jv / (60000.0f * 10) * CIRCLE_ENCODER; // RPM *CIRCLE_ENCODER / 60000 * ftime= encoder
    if (m_Jcur_encoder > CIRCLE_ENCODER)
      m_Jcur_encoder = m_Jcur_encoder - CIRCLE_ENCODER;
    machine_sync_deal(m_Jcur_encoder);
  }
#else
  g_encoder = qei_cnt();
  if (machine_get_acc_dec_flag() == 1) {
    machine_acc_dec_sync_deal(g_encoder);
  } else {
#if 0
  static uint8_t cnt =1 ;
  if(cnt==0)
  {
    cnt =1 ;
    machine_sync_deal(g_encoder) ;
  }
  else
  {
    cnt -- ;
  }
#else
    machine_sync_deal(g_encoder);
#endif
  }
#endif
#endif
#endif

  work_input_scan();
  servo_Auto_SR_Slave_ctrl() ;
  servo_ext_pos_ctrl_loop() ;
}

void MainTask1ms(void)
{
  if (MainTaskTicker1ms < 20)
    return;
  MainTaskTicker1ms = 0;

  mot_task_ms_loop();
  //----------------------------------------------------

  //----------------------------------------------------

  //----------------------------------------------------
  //
  // MonitorDataLoad();
  //----------------------------------------------------
}
void ADCA_IRQHandler(void)
{
  static uint8_t cnt = 1;
#if (BOARD_TYPE == BOARD_E145)
  u_av_adc = ((adc0_buff[0] & 0xffff) + (adc0_buff[1] & 0xffff) + (adc0_buff[2] & 0xffff) + (adc0_buff[3] & 0xffff)) >> 2;
  v_av_adc = ((adc1_buff[0] & 0xffff) + (adc1_buff[1] & 0xffff) + (adc1_buff[2] & 0xffff) + (adc1_buff[3] & 0xffff)) >> 2;
#elif (BOARD_TYPE == BOARD_E139 ||BOARD_TYPE == BOARD_E884 )
  v_av_adc = ((adc1_buff[0] & 0xffff) + (adc1_buff[1] & 0xffff) + (adc1_buff[2] & 0xffff) + (adc1_buff[3] & 0xffff)) >> 2;
  u_av_adc = ((adc0_buff[0] & 0xffff) + (adc0_buff[1] & 0xffff) + (adc0_buff[2] & 0xffff) + (adc0_buff[3] & 0xffff)) >> 2;
#elif (BOARD_TYPE == BOARD_E914)
  u_av_adc = ((adc1_buff[0] & 0xffff) + (adc1_buff[1] & 0xffff) + (adc1_buff[2] & 0xffff) + (adc1_buff[3] & 0xffff)) >> 2;
  w_av_adc = ((adc0_buff[0] & 0xffff) + (adc0_buff[1] & 0xffff) + (adc0_buff[2] & 0xffff) + (adc0_buff[3] & 0xffff)) >> 2;
#endif
  SM_StateMachineFast(&g_sM1Ctrl);
  SM_StateMachineSlow(&g_sM1Ctrl);
  MainTaskTicker1ms++;
#if 1
  if (cnt == 0)
  {
    BOARD_loop_isr();
    cnt = 1;
  }
  else
  {
    cnt--;
  }
#endif

  M1_MCDRV_ENC_READ_TRANS_NO_BLOCKING();
}
bool BOARD_Can1Send(uint32_t id, uint8_t *cmd, uint8_t len) {
  can_send_msg(HPM_MCAN1, id, cmd, len);
  return 0;
}
bool BOARD_CanSend(uint32_t id, uint8_t *cmd, uint8_t len)
{
  if (CANEncrypt(cmd))
    len = 8;
  can_send_msg(HPM_MCAN0, id, cmd, len);
  return 0;
}
uint8_t BOARD_Input(uint8_t type)
{
  uint8_t ret = 0xff;
#if (BOARD_TYPE == BOARD_E145)
  switch(type)
  {
      case 0:
        ret = gpio_read_pin(HPM_GPIO0,GPIO_DI_GPIOA,5) ;
      break;
      case 1:
        ret = gpio_read_pin(HPM_GPIO0,GPIO_DI_GPIOA,6) ;
      break;
      case 2:
        ret = gpio_read_pin(HPM_GPIO0,GPIO_DI_GPIOA,7) ;
      break;
  }
#else
  switch (type)
  {
  case 0:
    ret = gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOA, 10);
    break;
  case 1:
    ret = gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOA, 11);
    break;
  case 2:
    ret = gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOA, 12);
    break;
  }
#endif
  return ret;
}
void BOARD_Output(uint8_t value)
{

  gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 13, value);
}
int32_t board_get_jr_motor_encoder(void)
{
  return g_encoder;
}
void board_ms_loop(void)
{
  board_ms_cnt++;
}
uint32_t board_get_ms_tick(void)
{
  return board_ms_cnt;
}

uint32_t  BOARD_GetUID(uint8_t uid[12])
{
  uint32_t data[4] = {1234, 5678, 1234, 5678};
  uint32_t word_idx = 0;
  uint32_t ret =0 ;
  for (uint32_t i = OTP_CHIP_UUID_IDX_START; i <= OTP_CHIP_UUID_IDX_END; i++)
  {
    data[word_idx++] = ROM_API_TABLE_ROOT->otp_driver_if->read_from_shadow(i);
  }

  memcpy(&uid[0], ((uint32_t *)&data[1]), 4);
  memcpy(&uid[4], ((uint32_t *)&data[2]), 4);
  memcpy(&uid[8], ((uint32_t *)&data[3]), 4);
  memcpy(&ret,uid,4) ;
  return ret ;
}
/*void motEncCalibrate(void)
{
       //g_sMID.sMIDEnc.fltIdReqOpenLoop =  M1_I_PH_NOM*0.3f;
        g_sMID.sMIDEnc.fltIdReqOpenLoop =  M1_I_PH_NOM*0.3f;
        g_sMID.sMIDEnc.ui8Pp = M1_MOTOR_PP;
        g_sMID.eMeasurementType = kMID_EncSensors;
        g_sMID.sMIDAlignment.ui16AlignDuration = 1;
        g_sMID.sMIDAlignment.fltCurrentAlign = 0.0f;
        g_sMID.ui16EnableMeasurement = 1;
        g_sMID.sMIDEnc.clib_ext = 0 ;
        M1_SetAppSwitch(true);
}*/
