#include <float.h>
#include <stdint.h>
#include "board.h"
#include "param.h"
#include "motor.h"
#include "hal_for_upg.h"
#include "m1_pmsm_appconfig.h"
#include "servo_can.h"
#include "mcdrv.h"
#include "eeprom.h"
#include "jacking_motor.h"
#include "work_io.h"
#include "m1_pmsm_align.h"
#include "mcdrv_ftm_enc.h"
#include "eep_p24c32c.h"
#include "wire_feed_motor.h"
#include "gpio.h"
#include "user_app_fun.h"
#include "motor.h"
#define PARA_HEADER (0x55ccaabb)

#define PROP_TYPE_MASK (0x7 << 0)
#define PROP_TYPE_U4 (0x0 << 0)
#define PROP_TYPE_I4 (0x1 << 0)
#define PROP_TYPE_F4 (0x3 << 0)
#define PROP_TYPE_U2 (0x4 << 0)
#define PROP_TYPE_I2 (0x5 << 0)
#define PROP_TYPE_U1 (0x6 << 0)
#define PROP_TYPE_I1 (0x7 << 0)

#define PROP_WR_MASK (0x1 << 3)
#define PROP_READ (0x0 << 3)
#define PROP_WRITE (0x1 << 3)

#define PROP_SAVE_MASK (0x1 << 4)
#define PROP_NOSAVE (0x0 << 4)
#define PROP_SAVE (0x1 << 4)

#define PROP_LIMIT_MASK (0x1 << 5)
#define PROP_NOLIMIT (0x0 << 5)
#define PROP_LIMIT (0x1 << 5)

typedef struct Mot_Param_Tag
{
  uint16_t paraNo;
  uint8_t prop;
  Param_Data minVal;
  Param_Data maxVal;
  Param_Data defVal;
  Param_Data *value;
} Mot_Param;

extern IO_Fun_Tag io_fun_st;

//////////////////local varible define////////////////////////////////
#ifndef BOOT_DEF
#if (BOARD_TYPE == BOARD_E139)
static Mot_Param motParam[PARNO_MAX] =
    {
        {PARNO_MOT_ID, PROP_TYPE_U2 | PROP_WRITE | PROP_SAVE, {.u2_data = 1}, {.u2_data = 255}, {.u2_data = 1}, (Param_Data *)&mot_id},
        {PARNO_RUN_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 1}, (Param_Data *)&mot_run_state.runDir},
        {PARNO_ZERO_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 1}, (Param_Data *)&mot_run_state.zeroDir},
        {PARNO_ZERO_STOP, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroStPos},
        {PARNO_ZERO_POS, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroPos},
        {PARNO_NUME, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.nume},
        {PARNO_DENO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.deno},
        {PARNO_LEFT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = -384}, (Param_Data *)&mot_run_state.leftLmt},
        {PARNO_RIGHT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 16000}, (Param_Data *)&mot_run_state.rightLmt},
        {PARNO_MAX_SYNC_OFF, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 50.0f}, (Param_Data *)&mot_run_state.maxSyncOff},
        {PARNO_MOTOR_TYPE, PROP_TYPE_U1 | PROP_WRITE | PROP_SAVE, {.u1_data = 1}, {.u1_data = 255}, {.u1_data = 2}, (Param_Data *)&mot_run_state.motor_type},
        {PARNO_DIR_PROTOCOL, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 1}, {.u1_data = 5}, {.u2_data = DIR_PROTOCOL_ZHISHAN}, (Param_Data *)&mot_run_state.dir_protocol},
        {PARNO_ENC_DIR, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = -1}, {.i1_data = 1}, {.i1_data = -1}, (Param_Data *)&mot_run_state.encDir},
        {PARNO_SYNC_OFF_RATIO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.ratioSyncOff},
        {PARNO_CAN_BAUND, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = 0}, {.i1_data = 3}, {.i1_data = 0}, (Param_Data *)&mot_run_state.can_baund},
        {15, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_MOTOR_PP}, (Param_Data *)&motor.pp},
        {16, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Rs}, (Param_Data *)&motor.rs},
        {17, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Ld}, (Param_Data *)&motor.ld},
        {18, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Lq}, (Param_Data *)&motor.lq},
        {19, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_BACK_EMF}, (Param_Data *)&motor.ke},
        {20, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_I_PH_NOM}, (Param_Data *)&motor.Iph_nom},
        {21, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_ph}, (Param_Data *)&motor.Uph_nom},
        {22, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_req}, (Param_Data *)&motor.N_nom},
        {23, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_I_MAX}, (Param_Data *)&motor.I_max},
        {24, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_MAX}, (Param_Data *)&motor.UDCB_max},
        {25, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_TRIP}, (Param_Data *)&motor.UDCB_trip},
        {26, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_UNDERVOLTAGE}, (Param_Data *)&motor.UDCB_under},
        {27, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_OVERVOLTAGE}, (Param_Data *)&motor.UDCB_over},
        {28, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_over}, (Param_Data *)&motor.N_over},
        {29, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_min}, (Param_Data *)&motor.N_min},
        {30, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_max}, (Param_Data *)&motor.N_max},
        {31, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_E_BLOCK_TRH}, (Param_Data *)&motor.E_block},
        {32, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_E_BLOCK_PER}, (Param_Data *)&motor.E_block_pre},
        {33, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_RAMP_UP}, (Param_Data *)&motor_run.posAcc},
        {34, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_RAMP_DOWN}, (Param_Data *)&motor_run.posDec},
        {35, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_PI_PROP_GAIN}, (Param_Data *)&motor_run.posPip},
        {36, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_DRIVER_INERTIA}, (Param_Data *)&motor_run.inertia},
        {37, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_PROP_GAIN}, (Param_Data *)&motor_run.spdPidP},
        {38, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_INTEG_GAIN * 1000}, (Param_Data *)&motor_run.spdPidI},
        {39, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_LOOP_HIGH_LIMIT}, (Param_Data *)&motor_run.spdILmt},
        {40, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 50}, (Param_Data *)&motor_run.IBW},
        {41, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 80}, (Param_Data *)&motor_run.ILmt},
        {42, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1024}, (Param_Data *)&motor_run.zero_offset},
        {43, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1}, (Param_Data *)&motor.dir},
        {44, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1}, (Param_Data *)&motor.gear_ratio},
        {45, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&motor.torque_over_speed},
        {46, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 100}, (Param_Data *)&motor.max_over_load_scale},
        {47, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1000}, (Param_Data *)&motor.over_load_time[0]},
        {48, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 500}, (Param_Data *)&motor.over_load_time[1]},
        {49, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 250}, (Param_Data *)&motor.over_load_time[2]},
        {50, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 125}, (Param_Data *)&motor.over_load_time[3]},
        {51, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 60}, (Param_Data *)&motor.over_load_time[4]},
        {52, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 10}, (Param_Data *)&motor.speed_loop_rpm01},
        {53, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_PROP_GAIN}, (Param_Data *)&motor_run.spdPidP1},
        {54, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_INTEG_GAIN * 1000}, (Param_Data *)&motor_run.spdPidI1},
        {55, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_PI_PROP_GAIN}, (Param_Data *)&motor_run.posPip1},
        {56, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&mot_run_state.io_fun_set},
        {57, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = 3000}, {.f4_data = 1000}, (Param_Data *)&mot_run_state.punching_speed},
        {58, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = 3}, {.f4_data = 1}, (Param_Data *)&motor_run.ramp_type},
        {59, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 100}, (Param_Data *)&mot_run_state.ext_move_fast_speed},
        {60, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 50}, (Param_Data *)&mot_ext_pos_ctrl_st.encoder_diameter_mm},
        {61, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 500}, (Param_Data *)&&mot_ext_pos_ctrl_st.motor_encoder_10mm},
        {62, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 10}, (Param_Data *)&mot_run_state.ext_move_slow_speed},
        {63, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&mot_run_state.pos_buf_clib_scale},	
	
};
#elif (BOARD_TYPE == BOARD_E884)
static Mot_Param motParam[PARNO_MAX] =
    {
        {PARNO_MOT_ID, PROP_TYPE_U2 | PROP_WRITE | PROP_SAVE, {.u2_data = 1}, {.u2_data = 255}, {.u2_data = 1}, (Param_Data *)&mot_id},
        {PARNO_RUN_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 1}, (Param_Data *)&mot_run_state.runDir},
        {PARNO_ZERO_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 1}, (Param_Data *)&mot_run_state.zeroDir},
        {PARNO_ZERO_STOP, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroStPos},
        {PARNO_ZERO_POS, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroPos},
        {PARNO_NUME, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.nume},
        {PARNO_DENO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.deno},
        {PARNO_LEFT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = -384}, (Param_Data *)&mot_run_state.leftLmt},
        {PARNO_RIGHT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 16000}, (Param_Data *)&mot_run_state.rightLmt},
        {PARNO_MAX_SYNC_OFF, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 50.0f}, (Param_Data *)&mot_run_state.maxSyncOff},
        {PARNO_MOTOR_TYPE, PROP_TYPE_U1 | PROP_WRITE | PROP_SAVE, {.u1_data = 1}, {.u1_data = 255}, {.u1_data = 2}, (Param_Data *)&mot_run_state.motor_type},
        {PARNO_DIR_PROTOCOL, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 1}, {.u1_data = 5}, {.u2_data = DIR_PROTOCOL_ZHISHAN}, (Param_Data *)&mot_run_state.dir_protocol},
        {PARNO_ENC_DIR, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = -1}, {.i1_data = 1}, {.i1_data = -1}, (Param_Data *)&mot_run_state.encDir},
        {PARNO_SYNC_OFF_RATIO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.ratioSyncOff},
        {PARNO_CAN_BAUND, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = 0}, {.i1_data = 3}, {.i1_data = 0}, (Param_Data *)&mot_run_state.can_baund},
        {15, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_MOTOR_PP}, (Param_Data *)&motor.pp},
        {16, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Rs}, (Param_Data *)&motor.rs},
        {17, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Ld}, (Param_Data *)&motor.ld},
        {18, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Lq}, (Param_Data *)&motor.lq},
        {19, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_BACK_EMF}, (Param_Data *)&motor.ke},
        {20, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_I_PH_NOM}, (Param_Data *)&motor.Iph_nom},
        {21, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_ph}, (Param_Data *)&motor.Uph_nom},
        {22, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_req}, (Param_Data *)&motor.N_nom},
        {23, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_I_MAX}, (Param_Data *)&motor.I_max},
        {24, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_MAX}, (Param_Data *)&motor.UDCB_max},
        {25, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_TRIP}, (Param_Data *)&motor.UDCB_trip},
        {26, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_UNDERVOLTAGE}, (Param_Data *)&motor.UDCB_under},
        {27, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_OVERVOLTAGE}, (Param_Data *)&motor.UDCB_over},
        {28, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_over}, (Param_Data *)&motor.N_over},
        {29, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_min}, (Param_Data *)&motor.N_min},
        {30, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_max}, (Param_Data *)&motor.N_max},
        {31, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_E_BLOCK_TRH}, (Param_Data *)&motor.E_block},
        {32, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_E_BLOCK_PER}, (Param_Data *)&motor.E_block_pre},
        {33, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_RAMP_UP}, (Param_Data *)&motor_run.posAcc},
        {34, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_RAMP_DOWN}, (Param_Data *)&motor_run.posDec},
        {35, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_PI_PROP_GAIN}, (Param_Data *)&motor_run.posPip},
        {36, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_DRIVER_INERTIA}, (Param_Data *)&motor_run.inertia},
        {37, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_PROP_GAIN}, (Param_Data *)&motor_run.spdPidP},
        {38, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_INTEG_GAIN * 1000}, (Param_Data *)&motor_run.spdPidI},
        {39, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_LOOP_HIGH_LIMIT}, (Param_Data *)&motor_run.spdILmt},
        {40, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 50}, (Param_Data *)&motor_run.IBW},
        {41, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 80}, (Param_Data *)&motor_run.ILmt},
        {42, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1024}, (Param_Data *)&motor_run.zero_offset},
        {43, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1}, (Param_Data *)&motor.dir},
        {44, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1}, (Param_Data *)&motor.gear_ratio},
        {45, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&motor.torque_over_speed},
        {46, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 100}, (Param_Data *)&motor.max_over_load_scale},
        {47, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1000}, (Param_Data *)&motor.over_load_time[0]},
        {48, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 500}, (Param_Data *)&motor.over_load_time[1]},
        {49, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 250}, (Param_Data *)&motor.over_load_time[2]},
        {50, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 125}, (Param_Data *)&motor.over_load_time[3]},
        {51, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 60}, (Param_Data *)&motor.over_load_time[4]},
        {52, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 10}, (Param_Data *)&motor.speed_loop_rpm01},
        {53, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_PROP_GAIN}, (Param_Data *)&motor_run.spdPidP1},
        {54, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_INTEG_GAIN * 1000}, (Param_Data *)&motor_run.spdPidI1},
        {55, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_PI_PROP_GAIN}, (Param_Data *)&motor_run.posPip1},
        {56, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&mot_run_state.io_fun_set},
        {57, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = 3000}, {.f4_data = 1000}, (Param_Data *)&mot_run_state.punching_speed},
        {58, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = 3}, {.f4_data = 1}, (Param_Data *)&motor_run.ramp_type},
        {59, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 100}, (Param_Data *)&mot_run_state.ext_move_fast_speed},
        {60, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 50}, (Param_Data *)&mot_ext_pos_ctrl_st.encoder_diameter_mm},
        {61, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 500}, (Param_Data *)&mot_ext_pos_ctrl_st.motor_encoder_10mm},
        {62, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 10}, (Param_Data *)&mot_run_state.ext_move_slow_speed},
        {63, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&mot_run_state.pos_buf_clib_scale},
};
#elif (BOARD_TYPE == BOARD_E914)
static Mot_Param motParam[PARNO_MAX] =
    {
        {PARNO_MOT_ID, PROP_TYPE_U2 | PROP_WRITE | PROP_SAVE, {.u2_data = 1}, {.u2_data = 255}, {.u2_data = 1}, (Param_Data *)&mot_id},
        {PARNO_RUN_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 1}, (Param_Data *)&mot_run_state.runDir},
        {PARNO_ZERO_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 1}, (Param_Data *)&mot_run_state.zeroDir},
        {PARNO_ZERO_STOP, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroStPos},
        {PARNO_ZERO_POS, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroPos},
        {PARNO_NUME, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.nume},
        {PARNO_DENO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.deno},
        {PARNO_LEFT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = -384}, (Param_Data *)&mot_run_state.leftLmt},
        {PARNO_RIGHT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 16000}, (Param_Data *)&mot_run_state.rightLmt},
        {PARNO_MAX_SYNC_OFF, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 50.0f}, (Param_Data *)&mot_run_state.maxSyncOff},
        {PARNO_MOTOR_TYPE, PROP_TYPE_U1 | PROP_WRITE | PROP_SAVE, {.u1_data = 1}, {.u1_data = 255}, {.u1_data = 2}, (Param_Data *)&mot_run_state.motor_type},
        {PARNO_DIR_PROTOCOL, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 1}, {.u1_data = 5}, {.u2_data = DIR_PROTOCOL_ZHISHAN}, (Param_Data *)&mot_run_state.dir_protocol},
        {PARNO_ENC_DIR, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = -1}, {.i1_data = 1}, {.i1_data = -1}, (Param_Data *)&mot_run_state.encDir},
        {PARNO_SYNC_OFF_RATIO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.ratioSyncOff},
        {PARNO_CAN_BAUND, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = 0}, {.i1_data = 3}, {.i1_data = 0}, (Param_Data *)&mot_run_state.can_baund},
        {15, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_MOTOR_PP}, (Param_Data *)&motor.pp},
        {16, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Rs}, (Param_Data *)&motor.rs},
        {17, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Ld}, (Param_Data *)&motor.ld},
        {18, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Lq}, (Param_Data *)&motor.lq},
        {19, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_BACK_EMF}, (Param_Data *)&motor.ke},
        {20, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_I_PH_NOM}, (Param_Data *)&motor.Iph_nom},
        {21, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_ph}, (Param_Data *)&motor.Uph_nom},
        {22, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_req}, (Param_Data *)&motor.N_nom},
        {23, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_I_MAX}, (Param_Data *)&motor.I_max},
        {24, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_MAX}, (Param_Data *)&motor.UDCB_max},
        {25, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_TRIP}, (Param_Data *)&motor.UDCB_trip},
        {26, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_UNDERVOLTAGE}, (Param_Data *)&motor.UDCB_under},
        {27, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_OVERVOLTAGE}, (Param_Data *)&motor.UDCB_over},
        {28, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_over}, (Param_Data *)&motor.N_over},
        {29, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_min}, (Param_Data *)&motor.N_min},
        {30, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_max}, (Param_Data *)&motor.N_max},
        {31, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_E_BLOCK_TRH}, (Param_Data *)&motor.E_block},
        {32, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_E_BLOCK_PER}, (Param_Data *)&motor.E_block_pre},
        {33, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_RAMP_UP}, (Param_Data *)&motor_run.posAcc},
        {34, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_RAMP_DOWN}, (Param_Data *)&motor_run.posDec},
        {35, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_PI_PROP_GAIN}, (Param_Data *)&motor_run.posPip},
        {36, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_DRIVER_INERTIA}, (Param_Data *)&motor_run.inertia},
        {37, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_PROP_GAIN}, (Param_Data *)&motor_run.spdPidP},
        {38, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_INTEG_GAIN * 1000}, (Param_Data *)&motor_run.spdPidI},
        {39, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_LOOP_HIGH_LIMIT}, (Param_Data *)&motor_run.spdILmt},
        {40, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 50}, (Param_Data *)&motor_run.IBW},
        {41, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 80}, (Param_Data *)&motor_run.ILmt},
        {42, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1024}, (Param_Data *)&motor_run.zero_offset},
        {43, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1}, (Param_Data *)&motor.dir},
        {44, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1}, (Param_Data *)&motor.gear_ratio},
        {45, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&motor.torque_over_speed},
        {46, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 100}, (Param_Data *)&motor.max_over_load_scale},
        {47, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1000}, (Param_Data *)&motor.over_load_time[0]},
        {48, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 500}, (Param_Data *)&motor.over_load_time[1]},
        {49, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 250}, (Param_Data *)&motor.over_load_time[2]},
        {50, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 125}, (Param_Data *)&motor.over_load_time[3]},
        {51, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 60}, (Param_Data *)&motor.over_load_time[4]},
        {52, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 10}, (Param_Data *)&motor.speed_loop_rpm01},
        {53, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_PROP_GAIN}, (Param_Data *)&motor_run.spdPidP1},
        {54, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_INTEG_GAIN * 1000}, (Param_Data *)&motor_run.spdPidI1},
        {55, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_PI_PROP_GAIN}, (Param_Data *)&motor_run.posPip1},
        {56, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&mot_run_state.io_fun_set},
        {57, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = 3000}, {.f4_data = 1000}, (Param_Data *)&mot_run_state.punching_speed},
        {58, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 100}, (Param_Data *)&motor.max_over_load_scale1},

};
#elif (BOARD_TYPE == BOARD_E145)
static Mot_Param motParam[PARNO_MAX] =
    {
        {PARNO_MOT_ID, PROP_TYPE_U2 | PROP_WRITE | PROP_SAVE, {.u2_data = 1}, {.u2_data = 255}, {.u2_data = 1}, (Param_Data *)&mot_id},
        {PARNO_RUN_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 1}, (Param_Data *)&mot_run_state.runDir},   // 0
        {PARNO_ZERO_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 0}, (Param_Data *)&mot_run_state.zeroDir}, // 1
        {PARNO_ZERO_STOP, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroStPos},
        {PARNO_ZERO_POS, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroPos},
        {PARNO_NUME, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.nume},
        {PARNO_DENO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.deno},
        {PARNO_LEFT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = -400}, (Param_Data *)&mot_run_state.leftLmt},
        {PARNO_RIGHT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 400}, (Param_Data *)&mot_run_state.rightLmt},
        {PARNO_MAX_SYNC_OFF, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 50.0f}, (Param_Data *)&mot_run_state.maxSyncOff},
        {PARNO_MOTOR_TYPE, PROP_TYPE_U1 | PROP_WRITE | PROP_SAVE, {.u1_data = 1}, {.u1_data = 255}, {.u1_data = 2}, (Param_Data *)&mot_run_state.motor_type},
        {PARNO_DIR_PROTOCOL, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 1}, {.u1_data = 5}, {.u2_data = DIR_PROTOCOL_HQ}, (Param_Data *)&mot_run_state.dir_protocol},
        {PARNO_ENC_DIR, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = -1}, {.i1_data = 1}, {.i1_data = -1}, (Param_Data *)&mot_run_state.encDir},
        {PARNO_SYNC_OFF_RATIO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.ratioSyncOff},
        {PARNO_CAN_BAUND, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = 0}, {.i1_data = 3}, {.i1_data = 0}, (Param_Data *)&mot_run_state.can_baund},
        {15, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_MOTOR_PP}, (Param_Data *)&motor.pp},
        {16, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Rs}, (Param_Data *)&motor.rs},
        {17, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Ld}, (Param_Data *)&motor.ld},
        {18, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_Lq}, (Param_Data *)&motor.lq},
        {19, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_BACK_EMF}, (Param_Data *)&motor.ke},
        {20, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_I_PH_NOM}, (Param_Data *)&motor.Iph_nom},
        {21, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_ph}, (Param_Data *)&motor.Uph_nom},
        {22, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_req}, (Param_Data *)&motor.N_nom},
        {23, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_I_MAX}, (Param_Data *)&motor.I_max},
        {24, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_MAX}, (Param_Data *)&motor.UDCB_max},
        {25, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_TRIP}, (Param_Data *)&motor.UDCB_trip},
        {26, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_UNDERVOLTAGE}, (Param_Data *)&motor.UDCB_under},
        {27, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_U_DCB_OVERVOLTAGE}, (Param_Data *)&motor.UDCB_over},
        {28, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_over}, (Param_Data *)&motor.N_over},
        {29, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_min}, (Param_Data *)&motor.N_min},
        {30, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_N_max}, (Param_Data *)&motor.N_max},
        {31, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_E_BLOCK_TRH}, (Param_Data *)&motor.E_block},
        {32, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_E_BLOCK_PER}, (Param_Data *)&motor.E_block_pre},
        {33, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_RAMP_UP}, (Param_Data *)&motor_run.posAcc},
        {34, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_RAMP_DOWN}, (Param_Data *)&motor_run.posDec},
        {35, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_PI_PROP_GAIN}, (Param_Data *)&motor_run.posPip},
        {36, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_DRIVER_INERTIA}, (Param_Data *)&motor_run.inertia},
        {37, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_PROP_GAIN}, (Param_Data *)&motor_run.spdPidP},
        {38, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_INTEG_GAIN * 1000}, (Param_Data *)&motor_run.spdPidI},
        {39, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_LOOP_HIGH_LIMIT}, (Param_Data *)&motor_run.spdILmt},
        {40, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 50}, (Param_Data *)&motor_run.IBW},
        {41, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 80}, (Param_Data *)&motor_run.ILmt},
        {42, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1024}, (Param_Data *)&motor_run.zero_offset},
        {43, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1}, (Param_Data *)&motor.dir},
        {44, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1}, (Param_Data *)&motor.gear_ratio},
        {45, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 0}, (Param_Data *)&motor.torque_over_speed},
        {46, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 100}, (Param_Data *)&motor.max_over_load_scale},
        {47, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 1000}, (Param_Data *)&motor.over_load_time[0]},
        {48, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 500}, (Param_Data *)&motor.over_load_time[1]},
        {49, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 250}, (Param_Data *)&motor.over_load_time[2]},
        {50, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 125}, (Param_Data *)&motor.over_load_time[3]},
        {51, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 60}, (Param_Data *)&motor.over_load_time[4]},
        {52, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = 10}, (Param_Data *)&motor.speed_loop_rpm01},
        {53, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_PROP_GAIN}, (Param_Data *)&motor_run.spdPidP1},
        {54, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_SPEED_PI_INTEG_GAIN * 1000}, (Param_Data *)&motor_run.spdPidI1},
        {55, PROP_TYPE_F4 | PROP_WRITE | PROP_SAVE, {.f4_data = 0}, {.f4_data = FLOAT_PLUS_MAX}, {.f4_data = M1_POS_PI_PROP_GAIN}, (Param_Data *)&motor_run.posPip1},
};
#endif

#else
uint16_t mot_id = 1;
Mot_Run_State mot_run_state;

static Mot_Param motParam[PARNO_MAX] =
    {
        {PARNO_MOT_ID, PROP_TYPE_U2 | PROP_WRITE | PROP_SAVE, {.u2_data = 1}, {.u2_data = 255}, {.u2_data = 1}, (Param_Data *)&mot_id},
        {PARNO_RUN_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 1}, (Param_Data *)&mot_run_state.runDir},
        {PARNO_ZERO_DIR, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 0}, {.u1_data = 1}, {.u1_data = 0}, (Param_Data *)&mot_run_state.zeroDir},
        {PARNO_ZERO_STOP, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroStPos},
        {PARNO_ZERO_POS, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 0.0f}, (Param_Data *)&mot_run_state.zeroPos},
        {PARNO_NUME, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.nume},
        {PARNO_DENO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.deno},
        {PARNO_LEFT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = -FLT_MAX}, (Param_Data *)&mot_run_state.leftLmt},
        {PARNO_RIGHT_LMT, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = FLT_MAX}, (Param_Data *)&mot_run_state.rightLmt},
        {PARNO_MAX_SYNC_OFF, PROP_TYPE_F4 | PROP_WRITE | PROP_NOSAVE, {.f4_data = -FLT_MAX}, {.f4_data = FLT_MAX}, {.f4_data = 50.0f}, (Param_Data *)&mot_run_state.maxSyncOff},
        {PARNO_MOTOR_TYPE, PROP_TYPE_U1 | PROP_WRITE | PROP_SAVE, {.u1_data = 1}, {.u1_data = 255}, {.u1_data = 2}, (Param_Data *)&mot_run_state.motor_type},
        {PARNO_DIR_PROTOCOL, PROP_TYPE_U1 | PROP_WRITE | PROP_NOSAVE, {.u1_data = 1}, {.u1_data = 5}, {.u2_data = DIR_PROTOCOL_ZHISHAN}, (Param_Data *)&mot_run_state.dir_protocol},
        {PARNO_ENC_DIR, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = -1}, {.i1_data = 1}, {.i1_data = -1}, (Param_Data *)&mot_run_state.encDir},
        {PARNO_SYNC_OFF_RATIO, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1}, (Param_Data *)&mot_run_state.ratioSyncOff},
        {PARNO_CAN_BAUND, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = 0}, {.i1_data = 3}, {.i1_data = 0}, (Param_Data *)&mot_run_state.can_baund},
};
#endif

#ifdef JACKING_MOTOR_FUN
uint32_t jacking_motor_para_max = JACKING_MOTOR_PARA_MAX;
extern JACKING_MOTOR_INIT_ST jm_init_st;
extern JACKING_MOTOR_RESET_ST jm_reset_st;

static Mot_Param Jacking_motor_param[JACKING_MOTOR_PARA_MAX] =
    {
        {JACKING_MOTOR_BEING + 0, PROP_TYPE_I4 | PROP_READ | PROP_NOSAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&jacking_motor_para_max},
        {JACKING_MOTOR_BEING + 1, PROP_TYPE_I4 | PROP_WRITE | PROP_NOSAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&jm_init_st.dossal_pos},
        {JACKING_MOTOR_BEING + 2, PROP_TYPE_I4 | PROP_WRITE | PROP_NOSAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&jm_init_st.safe_signal_pos},
        {JACKING_MOTOR_BEING + 3, PROP_TYPE_I4 | PROP_WRITE | PROP_NOSAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&jm_init_st.cast_off_pos},
        {JACKING_MOTOR_BEING + 4, PROP_TYPE_I4 | PROP_WRITE | PROP_NOSAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&jm_init_st.safe_door_pos},
        {JACKING_MOTOR_BEING + 5, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&jm_init_st.dossal_st.dossal_safe_dec_pos},
        {JACKING_MOTOR_BEING + 6, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.zero_to_safe_door_speed},
        {JACKING_MOTOR_BEING + 7, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.safe_door_to_cast_off_speed},
        {JACKING_MOTOR_BEING + 8, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.cast_off_to_safe_speed},
        {JACKING_MOTOR_BEING + 9, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.safe_to_dossal_speed},
        {JACKING_MOTOR_BEING + 10, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.signal_enable_speed},
        {JACKING_MOTOR_BEING + 11, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.debug_speed},
        {JACKING_MOTOR_BEING + 12, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&jm_reset_st.reset_limit_pos},
        {JACKING_MOTOR_BEING + 13, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&jm_init_st.dossal_st.dossal_time_out_max},
        {JACKING_MOTOR_BEING + 14, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.safe_signal_st.safe_max_offset_pos},
        {JACKING_MOTOR_BEING + 15, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.safe_signal_st.safe_signal_check_pos},
        {JACKING_MOTOR_BEING + 16, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.safe_signal_st.safe_torque},
        {JACKING_MOTOR_BEING + 17, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.pdc_st.pdc_big_torque},
        {JACKING_MOTOR_BEING + 18, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.pdc_st.pdc_torque_time},
        {JACKING_MOTOR_BEING + 19, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.pdc_st.pdc_big_torque_time},
        {JACKING_MOTOR_BEING + 20, PROP_TYPE_I1 | PROP_WRITE | PROP_SAVE, {.i1_data = INT8_MIN}, {.i1_data = INT8_MAX}, {.i1_data = 0}, (Param_Data *)&jm_init_st.pdc_st.pdc_change_torque_enable},
        {JACKING_MOTOR_BEING + 21, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_reset_st.reset_signal_max_pos},
        {JACKING_MOTOR_BEING + 22, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.pdc_st.pdc_pull_max_pos},
        {JACKING_MOTOR_BEING + 23, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&jm_init_st.safe_signal_st.safe_move_offset_pos},
        {JACKING_MOTOR_BEING + 24, PROP_TYPE_I2 | PROP_WRITE | PROP_NOSAVE, {.i2_data = 0}, {.i2_data = 1}, {.i2_data = 0}, (Param_Data *)&mot_run_state.motor_dir_rev},
        //  {JACKING_MOTOR_BEING+25,PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE,{.i2_data = -1},{.i2_data = 15000},{.i2_data = -1},(Param_Data*)&jm_init_st.safe_to_dossal_torque}
};

#endif

#ifdef WIRE_MOTOR_FUN // 穿丝功能打开
WIRE_PARAM_HEAD wf_param_head_st = {
    .param_head_flag = 0x55aa,
    .param_version = 1,
    .param_groups = 1,
    .param_cur_group = 1,
    .prarm_cur_idx = 0,
};
uint8_t save_buf[8] = {0};
uint8_t wf_save_group = 0;
uint8_t wf_active_group = 0;
uint32_t wf_motor_para_max = WF_MOTOR_PARA_MAX;

extern WF_INIT_ST wf_init_st;
extern WF_RESET_ST wf_reset_st;
uint8_t *wf_motor_group_ptr;
WIRE_PARAM_HEAD *wf_param_head_ptr;
static Mot_Param wf_motor_param[WF_MOTOR_PARA_MAX] =
{
        {PARNO_MAX, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = WF_MOTOR_PARA_MAX}, (Param_Data *)&wf_motor_para_max},
        {PARNO_MAX + 1, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 20}, (Param_Data *)&wf_init_st.move_speed},
        {PARNO_MAX + 2, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 50}, (Param_Data *)&wf_init_st.feed_speed},
        {PARNO_MAX + 3, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 50}, (Param_Data *)&wf_init_st.collection_speed},
        {PARNO_MAX + 4, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.end_pos},
        {PARNO_MAX + 5, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1000}, (Param_Data *)&wf_init_st.move_speed_torque},
        {PARNO_MAX + 6, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 10}, (Param_Data *)&wf_init_st.signal_enable_speed},
        {PARNO_MAX + 7, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 20476}, (Param_Data *)&wf_reset_st.reset_limit_pos},
        {PARNO_MAX + 8, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 1000}, (Param_Data *)&wf_reset_st.reset_signal_max_pos},
        {PARNO_MAX + 9, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.wf_work_st.work_back_pos},
        {PARNO_MAX + 10, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 3}, (Param_Data *)&wf_init_st.wf_work_st.work_feed_block_retry_num},
        {PARNO_MAX + 11, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1000}, (Param_Data *)&wf_init_st.wf_work_st.work_feed_block_torque},
        {PARNO_MAX + 12, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 1000}, (Param_Data *)&wf_init_st.wf_work_st.work_collect_block_torque},
        {PARNO_MAX + 13, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.feed_dec_pos},
        {PARNO_MAX + 14, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.end_allowance_pos},
        {PARNO_MAX + 15, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.zero_signal_allow_pos},
        {PARNO_MAX + 16, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 20}, (Param_Data *)&wf_init_st.feed_dec_speed},
        {PARNO_MAX + 17, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 20}, (Param_Data *)&wf_init_st.wf_check_st.check_speed},
        {PARNO_MAX + 18, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 1000}, (Param_Data *)&wf_init_st.wire_broken_check_pos},
        {PARNO_MAX + 19, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 2}, (Param_Data *)&wf_init_st.wire_broken_check_type},
        {PARNO_MAX + 20, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 58}, (Param_Data *)&wf_init_st.wire_end_back_pos},
        {PARNO_MAX + 21, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 0}, (Param_Data *)&wf_init_st.wire_signal_def_status},
        {PARNO_MAX + 22, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 2}, (Param_Data *)&wf_init_st.wire_motor_signal_set_type},     // 厂家类型，2为杰泰方式设置零位和保存零位
        {PARNO_MAX + 23, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = -600}, (Param_Data *)&wf_init_st.wire_signal_save_zero_jt},    // 零位传感器设置保存值
        {PARNO_MAX + 24, PROP_TYPE_I2 | PROP_WRITE | PROP_SAVE, {.i2_data = INT16_MIN}, {.i2_data = INT16_MAX}, {.i2_data = 2}, (Param_Data *)&wf_init_st.wire_broken_retry_factory_type}, // 断丝回退厂家类型
        {PARNO_MAX + 25, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&mot_run_state.motor_dir_rev},               // 电机是否反向
        {PARNO_MAX + 26, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.feed_zero_dec_pos},              // 零位减速点
        {PARNO_MAX + 27, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.reserve[5]},
        {PARNO_MAX + 28, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.reserve[6]},
        {PARNO_MAX + 29, PROP_TYPE_I4 | PROP_WRITE | PROP_SAVE, {.i4_data = INT32_MIN}, {.i4_data = INT32_MAX}, {.i4_data = 0}, (Param_Data *)&wf_init_st.reserve[7]},

};
#endif

/////////////////////local function///////////////////////////////////
static void restoreParameter(void);
static void storeParameter(void);

//--------------------parameter manage--------------------------------
static void setParamValue(Param_Data *pDstVal, Param_Data *pSrcVal, uint8_t prop)
{
  if ((prop & PROP_TYPE_MASK) == PROP_TYPE_F4)
    pDstVal->f4_data = pSrcVal->f4_data;
  else if ((prop & PROP_TYPE_MASK) == PROP_TYPE_U4)
    pDstVal->u4_data = pSrcVal->u4_data;
  else if ((prop & PROP_TYPE_MASK) == PROP_TYPE_I4)
    pDstVal->i4_data = pSrcVal->i4_data;
  else if ((prop & PROP_TYPE_MASK) == PROP_TYPE_U2)
    pDstVal->u2_data = pSrcVal->u2_data;
  else if ((prop & PROP_TYPE_MASK) == PROP_TYPE_I2)
    pDstVal->i2_data = pSrcVal->i2_data;
  else if ((prop & PROP_TYPE_MASK) == PROP_TYPE_U1)
    pDstVal->u1_data = pSrcVal->u1_data;
  else if ((prop & PROP_TYPE_MASK) == PROP_TYPE_I1)
    pDstVal->i1_data = pSrcVal->i1_data;
}

void initParameter(void)
{
  // set default value
  for (int i = 0; i < PARNO_MAX; i++)
  {
    setParamValue(motParam[i].value, &motParam[i].defVal, motParam[i].prop);
  }
  restoreParameter();
 
#ifndef BOOT_DEF
  if (motor.max_over_load_scale >= 100 && motor.max_over_load_scale <= 300) //?
  {
    motor_run.over_load_scale = motor.max_over_load_scale / 100.0f;
  }
  else
    motor_run.over_load_scale = 1.0f;

  if (motor.max_over_load_scale1 >= 100 && motor.max_over_load_scale1 <= 300) //?
  {
    motor_run.over_load_scale1 = motor.max_over_load_scale1 / 100.0f;
  }
  else
    motor_run.over_load_scale1 = 1.0f;
#endif
  // for test
  // storeParameter();
}

static void storeParameter(void)
{

  uint32_t addr = 0;
  uint32_t para_header = PARA_HEADER;
  uint32_t ui32PrimaskReg;
  DISABLE_ALL_IRQ(ui32PrimaskReg);
  uint8_t *pData = EEPROM_Read_Data_addr();
  
  // write header

  memcpy(pData + addr, (uint8_t *)&para_header, sizeof(uint32_t));
  addr += sizeof(uint32_t);

#if (BOARD_TYPE == BOARD_E145)
  // write align data
  for (int j = 0; j < 2; j++)
  {
    for (int i = 0; i < MAX_ALIGN_CNT; i++)
    {
#ifndef BOOT_DEF
      memcpy(pData + addr, (uint8_t *)&g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[j][i], sizeof(uint32_t));
//  hal_flash_program(addr, (uint8_t *)&g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[i], sizeof(uint32_t));
#endif
      addr += sizeof(uint32_t);
    }
  }
#else
  // write align data
  for (int i = 0; i < MAX_ALIGN_CNT; i++)
  {
#ifndef BOOT_DEF
    memcpy(pData + addr, (uint8_t *)&g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i], sizeof(uint32_t));
    //  hal_flash_program(addr, (uint8_t *)&g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[i], sizeof(uint32_t));
#endif
    addr += sizeof(uint32_t);
  }
#endif
  // write parameter
  for (int i = 0; i < PARNO_MAX; i++)
  {
    if ((motParam[i].prop & PROP_SAVE_MASK) == PROP_SAVE)
    {
      memcpy(pData + addr, (uint8_t *)&motParam[i].paraNo, sizeof(uint32_t));
      //  hal_flash_program(addr, (uint8_t *)&motParam[i].paraNo, sizeof(uint32_t));
      addr += sizeof(uint32_t);
      memcpy(pData + addr, (uint8_t *)motParam[i].value, sizeof(Param_Data));
      // hal_flash_program(addr, (uint8_t *)motParam[i].value, sizeof(Param_Data));
      addr += sizeof(Param_Data);
    }
  }
#ifdef JACKING_MOTOR_FUN // 升降功能打开
  for (int i = 0; i < JACKING_MOTOR_PARA_MAX; i++)
  {

    if ((Jacking_motor_param[i].prop & PROP_SAVE_MASK) == PROP_SAVE)
    {
      memcpy(pData + addr, (uint8_t *)&Jacking_motor_param[i].paraNo, sizeof(uint32_t));

      addr += sizeof(uint32_t);
      memcpy(pData + addr, (uint8_t *)Jacking_motor_param[i].value, sizeof(Param_Data));

      addr += sizeof(Param_Data);
    }
  }
#endif

#ifdef WIRE_MOTOR_FUN // 穿丝功能打开
  for (int i = 0; i < WF_MOTOR_PARA_MAX; i++)
  {

    if ((wf_motor_param[i].prop & PROP_SAVE_MASK) == PROP_SAVE)
    {
      memcpy(pData + addr, (uint8_t *)&wf_motor_param[i].paraNo, sizeof(uint32_t));

      addr += sizeof(uint32_t);
      memcpy(pData + addr, (uint8_t *)wf_motor_param[i].value, sizeof(Param_Data));

      addr += sizeof(Param_Data);
    }
  }

#endif

  EEProm_Store();

  ENABLE_ALL_IRQ(ui32PrimaskReg);
}

static void restoreParameter(void)
{
  uint8_t *pData;
  uint8_t *pDataBegin;
  uint32_t para_header;
  uint16_t no;
  Param_Data val;
  uint32_t len = 0;
  Initial_EEProm();
  pData = EEPROM_Read_Data_addr();
  pDataBegin = EEPROM_Read_Data_addr();
  // read parameter value from flash.
  para_header = *((uint32_t *)pData);
  pData += sizeof(uint32_t);
  len += sizeof(uint32_t);
  if (para_header == PARA_HEADER)
  {
#if (BOARD_TYPE == BOARD_E145)
    // read align data
    for (int j = 0; j < 2; j++)
    {
      for (int i = 0; i < MAX_ALIGN_CNT; i++)
      {
#ifndef BOOT_DEF
        g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[j][i] = *((int16_t *)pData);
#endif
        pData += sizeof(uint32_t);
        len += sizeof(uint32_t);
      }
    }
#else
    // read align data
    for (int i = 0; i < MAX_ALIGN_CNT; i++)
    {
#ifndef BOOT_DEF
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] = *((int16_t *)pData);
#endif
      pData += sizeof(uint32_t);
      len += sizeof(uint32_t);
    }
#endif

#ifdef JACKING_MOTOR_FUN // 升降功能打开
    // read parameter
    while (len < (PAGE_SIZE - (sizeof(uint32_t) + sizeof(Param_Data))))
    {
      no = *((uint16_t *)pData);
      pData += sizeof(uint32_t);
      val = *((Param_Data *)pData);
      pData += sizeof(Param_Data);
      len += sizeof(uint32_t) + sizeof(Param_Data);
      for (int i = 0; i < PARNO_MAX; i++)
      {
        if (motParam[i].paraNo == no)
        {
          if (no == 0)
          {
            if (val.u2_data < 255 && val.u2_data > 0)
              setParamValue(motParam[i].value, &val, motParam[i].prop);
          }
          else
          {
            setParamValue(motParam[i].value, &val, motParam[i].prop);
          }
        }
      }
      for (int i = 0; i < JACKING_MOTOR_PARA_MAX; i++)
      {
        if (Jacking_motor_param[i].paraNo == no)
        {
          setParamValue(Jacking_motor_param[i].value, &val, Jacking_motor_param[i].prop);
        }
      }
      if (no == JACKING_MOTOR_BEING + JACKING_MOTOR_PARA_MAX - 1)
      {
        return;
      }
    }; // while
#else
#ifdef WIRE_MOTOR_FUN // 穿丝功能打开

    while (len < (SECTOR_SIZE - (sizeof(uint32_t) + sizeof(Param_Data))))
    {
      no = *((uint16_t *)pData);
      pData += sizeof(uint32_t);
      val = *((Param_Data *)pData);
      pData += sizeof(Param_Data);
      len += sizeof(uint32_t) + sizeof(Param_Data);

      // find parameter
      if (no < PARNO_MAX + WF_MOTOR_PARA_MAX)
      {
        for (int i = 0; i < PARNO_MAX + WF_MOTOR_PARA_MAX; i++)
        {
          if (i < PARNO_MAX)
          {
            if (motParam[i].paraNo == no)
            {
              if (no == 0)
              {
                if (val.u2_data < 255 && val.u2_data > 0)
                  setParamValue(motParam[i].value, &val, motParam[i].prop);
              }
              else
              {
                setParamValue(motParam[i].value, &val, motParam[i].prop);
              }
            }
          }
          else
          {
            if (wf_motor_param[i - PARNO_MAX].paraNo == no)
            {
              setParamValue(wf_motor_param[i - PARNO_MAX].value, &val, wf_motor_param[i - PARNO_MAX].prop);
            }
          }
          if (no == PARNO_MAX + WF_MOTOR_PARA_MAX - 1)
          {
            if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
            {
              wf_motor_group_ptr = pDataBegin + SECTOR_SIZE;
              wf_param_head_ptr = (WIRE_PARAM_HEAD *)(pDataBegin + SECTOR_SIZE);
            }
            return;
          }
        }
      }
    }; // while
    if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
    {
      wf_motor_group_ptr = pDataBegin + SECTOR_SIZE;
      wf_param_head_ptr = (WIRE_PARAM_HEAD *)(pDataBegin + SECTOR_SIZE);
    }

#else
    while (len < (PAGE_SIZE - (sizeof(uint32_t) + sizeof(Param_Data))))
    {
      no = *((uint16_t *)pData);
      pData += sizeof(uint32_t);
      val = *((Param_Data *)pData);
      pData += sizeof(Param_Data);
      len += sizeof(uint32_t) + sizeof(Param_Data);

      // find parameter
      if (no < PARNO_MAX)
      {
        for (int i = 0; i < PARNO_MAX; i++)
        {
          if (motParam[i].paraNo == no)
          {
            if (no == 0)
            {
              if (val.u2_data < 255 && val.u2_data > 0)
                setParamValue(motParam[i].value, &val, motParam[i].prop);
            }
            else
              setParamValue(motParam[i].value, &val, motParam[i].prop);
            if (no == PARNO_MAX - 1)
            {
              return;
            }
          }
        }
      }
    }; // while
#endif
#endif
    // read parameter
  }
}

void writeParameter(uint16_t paraNo, Param_Data data)
{
  if (paraNo < PARNO_MAX)
  {
    if (motParam[paraNo].prop & PROP_WR_MASK)
    {
      setParamValue(motParam[paraNo].value, &data, motParam[paraNo].prop);
      if (motParam[paraNo].prop & PROP_SAVE_MASK)
      { // save parameter to flash
        storeParameter();
      }
    }
  }
}
void writeAlignData()
{
  uint8_t data[8] = {0};
#if (BOARD_TYPE == BOARD_E145)
  for (int i = 0; i < ALIGN_TABLE_SIZE; i++)
  {

    if (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] > 0 && g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[1][i] < 0)
    {
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] = g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] - 16383;
    }
    else if (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] < 0 && g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[1][i] > 0)
    {
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] = g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] + 16383;
    }
    g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] = (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i] + g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[1][i]) / 2;
  }

  for (int i = 0; i < ALIGN_TABLE_SIZE; i++)
  {
    g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] = g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][(i + 1) % ALIGN_TABLE_SIZE] - g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPos[0][i];
    if (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] < -8192)
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] += 16384;
    if (g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] > 8192)
      g_sM1Enc.encAlignData.sAbcAlignData.s16AlignPosDista[i] -= 16384;
  }
#endif
  storeParameter();
  data[0] = CMD_CLIB_ZERO;
  data[1] = mot_id;
#ifndef _BOOT
  BOARD_CanSend(CAN_SEND_ID, data, 2);
#endif
}

Param_Data *readParameter(uint16_t paraNo)
{
  if (paraNo < PARNO_MAX)
    return motParam[paraNo].value;

  return NULL;
}

#ifdef JACKING_MOTOR_FUN // 升降功能打开
void writeJackingMotorParameter(uint16_t paraNo, Param_Data data)
{
  if (paraNo < JACKING_MOTOR_PARA_MAX)
  {

    if (Jacking_motor_param[paraNo].prop & PROP_WR_MASK)
    {

      setParamValue(Jacking_motor_param[paraNo].value, &data, Jacking_motor_param[paraNo].prop);
      if (Jacking_motor_param[paraNo].prop & PROP_SAVE_MASK)
      { // save parameter to flash
        storeParameter();
      }
    }
  }
}

uint32_t readJackingMotorParameter(uint16_t paraNo)
{
  uint32_t ret = 0;
  if (paraNo == 0)
    return JACKING_MOTOR_PARA_MAX;
  if (paraNo < JACKING_MOTOR_PARA_MAX)
  {
    if ((Jacking_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_U4)
      ret = Jacking_motor_param[paraNo].value->u4_data;
    else if ((Jacking_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_I4)
      ret = Jacking_motor_param[paraNo].value->i4_data;
    else if ((Jacking_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_U2)
      ret = Jacking_motor_param[paraNo].value->u2_data;
    else if ((Jacking_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_I2)
      ret = Jacking_motor_param[paraNo].value->i2_data;
    else if ((Jacking_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_U1)
      ret = Jacking_motor_param[paraNo].value->u1_data;
    else if ((Jacking_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_I1)
      ret = Jacking_motor_param[paraNo].value->i1_data;
  }

  return ret;
}

#endif

#ifdef WIRE_MOTOR_FUN // 穿丝功能打开
void WFMotorParameterInit(void)
{
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
  {
    if (wf_param_head_st.param_groups > WF_PARARM_MAX)
      wf_param_head_st.param_groups = 1;
  }
  else
  {
    P24CLXX_Read(0, (uint8_t *)&wf_param_head_st.param_head_flag, sizeof(wf_param_head_st));
    if (wf_param_head_st.param_groups > WF_PARARM_MAX)
      wf_param_head_st.param_groups = 1;
  }
  /*if(wf_param_head_st.param_head_flag != 0x55aa)
    wf_param_head_st.param_groups = 0 ;*/
}
void writeWfMotorParameter(uint16_t paraNo, Param_Data data)
{

  if (paraNo < WF_MOTOR_PARA_MAX)
  {
    // if(!isSameParamValue(wf_motor_param[paraNo].value, &data, wf_motor_param[paraNo].prop))
    {
      // if(wf_motor_param[paraNo].prop &PROP_WR_MASK)
      {

        setParamValue(wf_motor_param[paraNo].value, &data, wf_motor_param[paraNo].prop);
        //          if(motParam[paraNo].prop & PROP_SAVE_MASK)
        if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
        {
          // storeParameter();
        }
        else
        {
          uint16_t high_wr_cur = 0;
          uint16_t low_wr_cur = 0;
          uint16_t wr_cur_ptr = sizeof(wf_param_head_st) + (paraNo) * 8;
          memcpy(save_buf, (uint8_t *)&wf_motor_param[paraNo].paraNo, sizeof(uint32_t));
          memcpy(save_buf + 4, (uint8_t *)wf_motor_param[paraNo].value, sizeof(Param_Data));
          high_wr_cur = (wr_cur_ptr << 8);
          low_wr_cur = (wr_cur_ptr >> 8);
          P24CLXX_Write(high_wr_cur | low_wr_cur, save_buf, 8);
        }
      }
    }
  }
}
void SaveWfMotorCurGroup(uint8_t group)
{
  if (group > WF_MOTOR_PARA_MAX)
  {
    group = 1;
  }
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
  {
    wf_param_head_ptr->param_cur_group = group;
    //storeParameter();
  }
  else
  {
    P24CLXX_Read(0, (uint8_t *)&wf_param_head_st.param_head_flag, sizeof(WIRE_PARAM_HEAD));
    wf_param_head_st.param_cur_group = group;
    P24CLXX_Write(0, (uint8_t *)&wf_param_head_st.param_head_flag, sizeof(WIRE_PARAM_HEAD));
  }
}

void SaveWfMotorGroupParamToEpp(uint8_t group)
{
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
  {
    uint16_t wr_begin_ptr = sizeof(wf_param_head_st) + WF_MOTOR_PARA_MAX * group * 8;
    uint16_t wr_cur_ptr = wr_begin_ptr;

    for (int i = 0; i < WF_MOTOR_PARA_MAX; i++)
    {
      memcpy(save_buf, (uint8_t *)&wf_motor_param[i].paraNo, sizeof(uint32_t));
      memcpy(save_buf + 4, (uint8_t *)wf_motor_param[i].value, sizeof(Param_Data));
      memcpy(wf_motor_group_ptr + wr_cur_ptr, save_buf, 8);
      wr_cur_ptr += 8;
    }
    if(group ==0)
      storeParameter();
  }
  else
  {
    uint16_t wr_begin_ptr = sizeof(wf_param_head_st) + WF_MOTOR_PARA_MAX * group * 8;
    uint16_t wr_cur_ptr = wr_begin_ptr;

    for (int i = 0; i < WF_MOTOR_PARA_MAX; i++)
    {
      //		if((wf_motor_param[i].prop & PROP_SAVE_MASK) == PROP_SAVE)
      {
        uint16_t high_wr_cur = 0;
        uint16_t low_wr_cur = 0;
        memcpy(save_buf, (uint8_t *)&wf_motor_param[i].paraNo, sizeof(uint32_t));
        memcpy(save_buf + 4, (uint8_t *)wf_motor_param[i].value, sizeof(Param_Data));
        high_wr_cur = (wr_cur_ptr << 8);
        low_wr_cur = (wr_cur_ptr >> 8);
        // if(wf_motor_param[i].paraNo!= PARNO_MAX+4)
        P24CLXX_Write(high_wr_cur | low_wr_cur, save_buf, 8);
        wr_cur_ptr += 8;
        board_delay_ms(10);
      }
    }
  }
}

uint8_t GetWfMotorGroup(uint8_t idx)
{
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
  {
    if (idx == 0)
    {
      return wf_param_head_ptr->param_groups;
    }
    else
    {
      if (idx > WF_MOTOR_PARA_MAX)
      {
        idx = 1;
      }
      return wf_param_head_ptr->param_group[idx - 1];
    }
  }
  else
  {
    if (idx == 0)
    {
      return wf_param_head_st.param_groups;
    }
    else
    {
      if (idx > WF_MOTOR_PARA_MAX)
      {
        idx = 1;
      }
      return wf_param_head_st.param_group[idx - 1];
    }
  }
}
uint8_t GetWfMotorCurGroup(void)
{
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
    return wf_param_head_ptr->param_cur_group;
  else
    return wf_param_head_st.param_cur_group;
}
uint8_t GetWfMotorFactoryGroup(uint8_t idx)
{
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
  {
    if (idx == 0)
    {
      return wf_param_head_ptr->param_groups;
    }
    else
    {
      if (idx > WF_MOTOR_PARA_MAX)
      {
        idx = 1;
      }
      return wf_param_head_ptr->factory_group[idx - 1];
    }
  }
  else
  {
    if (idx == 0)
    {
      return wf_param_head_st.param_groups;
    }
    else
    {
      if (idx > WF_MOTOR_PARA_MAX)
      {
        idx = 1;
      }
      return wf_param_head_st.factory_group[idx - 1];
    }
  }
}
void ReadWfMotorGroupParamFromEpp(uint8_t group)
{
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
  {
    uint16_t rd_begin_ptr = sizeof(wf_param_head_st) + WF_MOTOR_PARA_MAX * group * 8;
    uint16_t no, rd_cur_ptr;
    Param_Data val;
    rd_cur_ptr = rd_begin_ptr;
    for (int i = 0; i < WF_MOTOR_PARA_MAX; i++)
    {

      memcpy(save_buf, wf_motor_group_ptr + rd_cur_ptr, 8);
      no = *((uint16_t *)(save_buf));
      rd_cur_ptr += sizeof(uint32_t);
      val = *((Param_Data *)(save_buf + 4));
      rd_cur_ptr += sizeof(Param_Data);
      if (wf_motor_param[i].paraNo == no)
      {
         
          setParamValue(wf_motor_param[i].value, &val, wf_motor_param[i].prop);
      }
    }
  }
  else
  {
    uint16_t rd_begin_ptr = sizeof(wf_param_head_st) + WF_MOTOR_PARA_MAX * group * 8;
    uint16_t no, rd_cur_ptr;
    Param_Data val;
    rd_cur_ptr = rd_begin_ptr;
    for (int i = 0; i < WF_MOTOR_PARA_MAX; i++)
    {
      uint16_t high_wr_cur = 0;
      uint16_t low_wr_cur = 0;
      high_wr_cur = (rd_cur_ptr << 8);
      low_wr_cur = (rd_cur_ptr >> 8);
      P24CLXX_Read(high_wr_cur | low_wr_cur, save_buf, 8);
      board_delay_ms(10);
      no = *((uint16_t *)(save_buf));
      rd_cur_ptr += sizeof(uint32_t);
      val = *((Param_Data *)(save_buf + 4));
      rd_cur_ptr += sizeof(Param_Data);
      if (wf_motor_param[i].paraNo == no)
      {
        setParamValue(wf_motor_param[i].value, &val, wf_motor_param[i].prop);
      }
    }
  }
}
void DeleteAllWfMotorGroup(void)
{
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
  {
    wf_param_head_ptr->param_groups = 0;
    wf_param_head_ptr->prarm_cur_idx = 0;
  }
  else
  {
    wf_param_head_st.param_groups = 0;
    wf_param_head_st.prarm_cur_idx = 0;
    P24CLXX_Write(0, (uint8_t *)&wf_param_head_st.param_head_flag, sizeof(wf_param_head_st));
  }
}
void AddWfMotorGroup(uint8_t gourp_id, uint8_t factory_id)
{
  if (BOARD_E145_GET_VER() == 1) // 硬件版本V2.0
  {
    wf_param_head_ptr->param_head_flag = 0x55aa;
    wf_param_head_ptr->param_group[wf_param_head_ptr->prarm_cur_idx] = gourp_id;
    wf_param_head_ptr->factory_group[wf_param_head_ptr->prarm_cur_idx] = factory_id;
    wf_param_head_ptr->prarm_cur_idx++;
    if (wf_param_head_ptr->prarm_cur_idx > WF_PARARM_MAX)
      wf_param_head_ptr->prarm_cur_idx = WF_PARARM_MAX - 1;
    wf_param_head_ptr->param_groups = wf_param_head_ptr->prarm_cur_idx;
  }
  else
  {
    wf_param_head_st.param_head_flag = 0x55aa;
    wf_param_head_st.param_group[wf_param_head_st.prarm_cur_idx] = gourp_id;
    wf_param_head_st.factory_group[wf_param_head_st.prarm_cur_idx] = factory_id;
    wf_param_head_st.prarm_cur_idx++;
    if (wf_param_head_st.prarm_cur_idx > WF_PARARM_MAX)
      wf_param_head_st.prarm_cur_idx = WF_PARARM_MAX - 1;
    wf_param_head_st.param_groups = wf_param_head_st.prarm_cur_idx;
    P24CLXX_Write(0, (uint8_t *)&wf_param_head_st.param_head_flag, sizeof(wf_param_head_st));
  }
}
void recoverWFMotorParam(uint8_t param_use_no)
{
  if (param_use_no <= wf_init_st.wire_params)
  {
  }
}

uint32_t readWfMotorParameter(uint16_t paraNo)
{
  uint32_t ret = 0;

  if (paraNo < WF_MOTOR_PARA_MAX)
  {
    if ((wf_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_U4)
      ret = wf_motor_param[paraNo].value->u4_data;
    else if ((wf_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_I4)
      ret = wf_motor_param[paraNo].value->i4_data;
    else if ((wf_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_U2)
      ret = wf_motor_param[paraNo].value->u2_data;
    else if ((wf_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_I2)
      ret = wf_motor_param[paraNo].value->i2_data;
    else if ((wf_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_U1)
      ret = wf_motor_param[paraNo].value->u1_data;
    else if ((wf_motor_param[paraNo].prop & PROP_TYPE_MASK) == PROP_TYPE_I1)
      ret = wf_motor_param[paraNo].value->i1_data;
  }

  return ret;
}
#endif
