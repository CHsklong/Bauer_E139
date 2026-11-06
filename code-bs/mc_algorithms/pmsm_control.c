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

#include "pmsm_control.h"
#include "ladrc.h"
#include "mcdrv.h"
#include "m1_pmsm_appconfig.h"
#include "m1_sm_ref_sol.h"
#include "cl_deadtime_comp.h"
#include "cl_dq_decouple.h"
#include "cl_srf_harm.h"
#include "hpm_csr_drv.h"
#include "hpm_clock_drv.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* B2 基值：与 FOC 一致；母线标称 ≈ U_MAX * √3 */
#ifndef M1_DT_VDC_NOM
#define M1_DT_VDC_NOM  (M1_U_MAX * 1.732050808F)
#endif
/* B1：永磁磁链 ψf [V·s/rad]；与 E160 同电机族初值，可按实测 Ke 覆盖 */
#ifndef M1_PSI_F
#define M1_PSI_F  (0.0187F)
#endif
/* B2：死区时间；pwm.c dead_zone_in_half_cycle=60 ≈ 150ns（800≈2us） */
#ifndef M1_DT_TD_SEC
#define M1_DT_TD_SEC  (150e-9F)
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MCS_ApplyB2DeadtimeComp(mcs_pmsm_foc_t *psFocPMSM);
static void MCS_ApplyB1DqDecouple(mcs_pmsm_foc_t *psFocPMSM);
static void MCS_ApplyC2SrfHarm(mcs_pmsm_foc_t *psFocPMSM);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* dead-time compensation voltage table */
float_t pfltUDtComp[DTCOMP_TABLE_SIZE] = DTCOMP_TABLE_DATA;

/* dead-time compensation look-up table */
GFLIB_LUT1D_T_FLT sLUTUDtComp;

/* B2 非线性死区与逆变器补偿 */
cl_deadtime_comp_t g_sDeadtimeComp;

/* B1 dq 交叉耦合解耦前馈 */
cl_dq_decouple_t g_sDqDecouple;

/* C2–C4 SRF 5/7/11/13 特定谐波抑制（默认开 5+7） */
cl_srf_harm_t g_sSrfHarm;

/* MCS_PMSMFocCtrl 耗时（µs），FreeMASTER 观察 */
volatile float_t g_fltM1FocTimeUs;
volatile float_t g_fltM1FocTimeUsMax;
volatile uint32_t g_ui32M1FocCycles;

/*static void MCS_DTComp(GMCLIB_2COOR_ALBE_T_FLT *sUAlBeDTComp,
                GMCLIB_3COOR_T_FLT *sIABC,
                float_t fltUDcBusFilt,
                float_t fltPwrStgCharIRange,
                float_t fltPwrStgCharLinCoeff);*/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief PMSM field oriented current control.
 *
 * This function is used to compute PMSM field oriented current control.
 *
 * @param psFocPMSM     The pointer of the PMSM FOC structure
 *
 * @return None
 */
void MCS_PMSMFocCtrl(mcs_pmsm_foc_t *psFocPMSM)
{
    uint64_t u64CycStart;
    uint64_t u64CycEnd;
    uint32_t ui32Cyc;
    float_t fltUs;
    uint32_t ui32CpuHz;

    u64CycStart = hpm_csr_get_core_cycle();

    /* pass electrical position from outside function if enabled else estimated
     * electrical position is selected */
    if (psFocPMSM->bPosExtOn)
        psFocPMSM->f16PosEl = psFocPMSM->f16PosElExt;
    else
        psFocPMSM->f16PosEl = psFocPMSM->f16PosElEst;
#if 0
    /* Position angle of the last PWM update */
    psFocPMSM->sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)psFocPMSM->f16PosElEst);
    psFocPMSM->sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)psFocPMSM->f16PosElEst);
#endif
    /* 3-phase to 2-phase transformation to stationary ref. frame */
#if (BOARD_TYPE == BOARD_E145)

    psFocPMSM->sIAlBe.fltAlpha = psFocPMSM->sIABC.fltA;
    psFocPMSM->sIAlBe.fltBeta = psFocPMSM->sIABC.fltB;
#else
    /* 3-phase to 2-phase transformation to stationary ref. frame */
    GMCLIB_Clark_FLT(&psFocPMSM->sIABC, &psFocPMSM->sIAlBe);
#endif

#if 0
    /* 2-phase to 2-phase transformation to rotary ref. frame */
    GMCLIB_Park_FLT(&psFocPMSM->sIAlBe, &psFocPMSM->sAnglePosEl, &psFocPMSM->sIDQ);
    GMCLIB_Park_FLT(&psFocPMSM->sUAlBeReq, &psFocPMSM->sAnglePosEl, &psFocPMSM->sUDQEst);
#endif
    /* BEMF observer in DQ system */
    //    psFocPMSM->acc32BemfErr = AMCLIB_PMSMBemfObsrvDQ_A32fff(&psFocPMSM->sIDQ,
    //                                                            &psFocPMSM->sUDQEst,
    //                                                            psFocPMSM->fltSpeedElEst,
    //                                                            &psFocPMSM->sBemfObsrv);
    //
    /* tracking observer calculating speed and position */
    // psFocPMSM->f16PosElEst = (frac16_t)AMCLIB_TrackObsrv_A32af(psFocPMSM->acc32BemfErr, &psFocPMSM->sTo);

    /* Speed estimation filter  */
    //    psFocPMSM->fltSpeedElEst = GDFLIB_FilterIIR1_FLT(psFocPMSM->sTo.fltSpeed, &psFocPMSM->sSpeedElEstFilt);

    /* for open loop control enabled parallel running of observer and FOC
     * open loop electrical position passed to rest of FOC */
    if (psFocPMSM->bTest)
        psFocPMSM->f16PosEl = 0;
    if (psFocPMSM->bOpenLoop || psFocPMSM->bPosExtOn)
    {
        /* GMCLIB_2COOR_ALBE_T_FLT sIAlBeTemp ;
         sIAlBeTemp.fltAlpha = psFocPMSM->sIAlBe.fltBeta ;
         sIAlBeTemp.fltBeta = psFocPMSM->sIAlBe.fltAlpha ;
         psFocPMSM->sIAlBe.fltBeta  = sIAlBeTemp.fltBeta ;
         psFocPMSM->sIAlBe.fltAlpha = sIAlBeTemp.fltAlpha ;*/
        psFocPMSM->sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)psFocPMSM->f16PosEl);
        psFocPMSM->sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)psFocPMSM->f16PosEl);
        GMCLIB_Park_FLT(&psFocPMSM->sIAlBe, &psFocPMSM->sAnglePosEl, &psFocPMSM->sIDQ);
    }
#if (BOARD_TYPE == BOARD_E145)
#else
    psFocPMSM->sIDQ.fltD = -psFocPMSM->sIDQ.fltD;
    psFocPMSM->sIDQ.fltQ = -psFocPMSM->sIDQ.fltQ;
#endif
    /* perform current control loop if enabled */
    if (psFocPMSM->bCurrentLoopOn)
    {
        /* D current error calculation */
        psFocPMSM->sIDQError.fltD = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltD, psFocPMSM->sIDQ.fltD);

        /* Q current error calculation */
        psFocPMSM->sIDQError.fltQ = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltQ, psFocPMSM->sIDQ.fltQ);

        /*** D - controller limitation calculation ***/
        psFocPMSM->sIdPiParams.fltLowerLim = MLIB_MulNeg_FLT(psFocPMSM->fltDutyCycleLimit, psFocPMSM->fltUDcBusFilt);
        psFocPMSM->sIdPiParams.fltUpperLim = MLIB_Mul_FLT(psFocPMSM->fltDutyCycleLimit, psFocPMSM->fltUDcBusFilt);

        /* D current PI controller */
        psFocPMSM->sUDQReq.fltD =
            GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltD, &psFocPMSM->bIdPiStopInteg, &psFocPMSM->sIdPiParams);

        /*** Q - controller limitation calculation ***/
        psFocPMSM->sIqPiParams.fltUpperLim = GFLIB_Sqrt_FLT(
            psFocPMSM->sIdPiParams.fltUpperLim * psFocPMSM->sIdPiParams.fltUpperLim -
            psFocPMSM->sUDQReq.fltD * psFocPMSM->sUDQReq.fltD);
        psFocPMSM->sIqPiParams.fltLowerLim = MLIB_Neg_FLT(psFocPMSM->sIqPiParams.fltUpperLim);

        /* Q current PI controller */
        psFocPMSM->sUDQReq.fltQ =
            GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltQ, &psFocPMSM->bIqPiStopInteg, &psFocPMSM->sIqPiParams);

        /* B1：PI 之后叠加 dq 解耦/反电势前馈，再进逆 Park */
        if (psFocPMSM->bFlagDqDecouple)
        {
            MCS_ApplyB1DqDecouple(psFocPMSM);
        }
    }

    /* 2-phase to 2-phase transformation to stationary ref. frame */
    GMCLIB_ParkInv_FLT(&psFocPMSM->sUDQReq, &psFocPMSM->sAnglePosEl, &psFocPMSM->sUAlBeReq);

    /* αβ 电压指令：先叠 SRF 谐波补偿，再叠 B2 死区补偿 */
    psFocPMSM->sUAlBeDTComp = psFocPMSM->sUAlBeReq;
    if (psFocPMSM->bFlagSrfHarm && psFocPMSM->bCurrentLoopOn)
    {
        MCS_ApplyC2SrfHarm(psFocPMSM);
    }
    if (psFocPMSM->bFlagDTComp)
    {
        MCS_ApplyB2DeadtimeComp(psFocPMSM);
    }

    /* DCBus ripple elimination */
    GMCLIB_ElimDcBusRipFOC_F16ff(psFocPMSM->fltUDcBusFilt, &psFocPMSM->sUAlBeDTComp, &psFocPMSM->sUAlBeCompFrac);

    /* space vector modulation */
// psFocPMSM->ui16SectorSVM = GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);
#if (BOARD_TYPE == BOARD_E145)
    GMCLIB_SvmStd2_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);
#else
    GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);
#endif

    u64CycEnd = hpm_csr_get_core_cycle();
    ui32Cyc = (uint32_t)(u64CycEnd - u64CycStart);
    g_ui32M1FocCycles = ui32Cyc;
    ui32CpuHz = hpm_core_clock;
    if (ui32CpuHz == 0U)
    {
        ui32CpuHz = clock_get_frequency(clock_cpu0);
    }
    if (ui32CpuHz != 0U)
    {
        fltUs = ((float_t)ui32Cyc * 1000000.0F) / (float_t)ui32CpuHz;
        g_fltM1FocTimeUs = fltUs;
        if (fltUs > g_fltM1FocTimeUsMax)
        {
            g_fltM1FocTimeUsMax = fltUs;
        }
    }
}

/*!
 * @brief PMSM field oriented speed control.
 *
 * This function is used to compute PMSM field oriented speed control.
 *
 * @param psFocPMSM     The pointer of the PMSM FOC structure
 * @param psSpeed       The pointer of the PMSM speed structure
 *
 * @return None
 */
void MCS_PMSMFocCtrlSpeed(mcs_speed_t *psSpeed)
{
    /* Speed saturation flag given by the Q current controller saturation flag and speed controller saturation flag */
    psSpeed->bSpeedPiStopInteg = (psSpeed->sSpeedPiParams.bLimFlag | psSpeed->bIqPiLimFlag) &
                                 (MLIB_Abs_FLT(psSpeed->fltSpeedCmd) >= MLIB_Abs_FLT(psSpeed->fltSpeedFilt));
    if (g_sM1Drive.eControl == kControlMode_SpeedFOC)
    {
        if (psSpeed->u8RampType == SPEED_S_RAMP)
        {
            /* Speed sRamp */
            if (psSpeed->fltSpeedCmd != psSpeed->sRampParSpdMe.fltTarget)
            {
                /* recalculate new S-ramp */
                GFLIB_FlexSRampCalcIncr_FLT(psSpeed->fltSpeedCmd,
                                            psSpeed->fltSRampDuration,
                                            &psSpeed->sRampParSpdMe);
            }
            psSpeed->fltSpeedRamp = GFLIB_FlexSRamp_FLT(&psSpeed->sRampParSpdMe);
        }
        else if (psSpeed->u8RampType == SPEED_TD_RAMP)
        {
            TD(&td_set_spd, psSpeed->fltSpeedCmd);
            psSpeed->fltSpeedRamp = td_set_spd.x1;
        }
        else
        {
            /* Speed ramp generation */
            psSpeed->fltSpeedRamp = GFLIB_Ramp_FLT(psSpeed->fltSpeedCmd, &psSpeed->sSpeedRampParams);
        }
    }
    else
    {
        /* Speed ramp generation */
       // psSpeed->fltSpeedRamp = GFLIB_Ramp_FLT(psSpeed->fltSpeedCmd, &psSpeed->sSpeedRampParams);
         psSpeed->fltSpeedRamp = psSpeed->fltSpeedCmd ;
    }

    /* Speed error calculation */

    /* required speed ramp */

    psSpeed->fltSpeedError = MLIB_Sub_FLT(psSpeed->fltSpeedRamp, psSpeed->fltSpeedFilt);

    /* Desired current by the speed PI controller */
    psSpeed->fltIqReq =
        GFLIB_CtrlPIpAW_FLT(psSpeed->fltSpeedError, &psSpeed->bSpeedPiStopInteg, &psSpeed->sSpeedPiParams);
}

/*!
 * @brief PMSM field oriented Position control.
 *
 * This function is used to compute PMSM field oriented Position control.
 *
 * @param psFocPMSM     The pointer of the PMSM FOC structure
 * @param psPosition       The pointer of the PMSM Position structure
 *
 * @return None
 */
#if 0
 void MCS_PMSMFocCtrlPosition(mcs_position_t *psPosition)
{
    /* 位置环停止积分标志 */
    psPosition->bPositionPiStopInteg = TRUE;

    
    if (psPosition->u8PosRampType == POS_S_RAMP)
    {
        /* 7段式位置规划 */

        if (psPosition->fltPositionCmd != psPosition->sPos7.fltTarget)
        {
                psPosition->sPos7.fltState = g_sM1Drive.sPosCtrl.fltPositionFilt;
                GFLIB_FlexSPos7CalcIncr_FLT_C(psPosition->fltPositionCmd, psPosition->sPos7.fltVel, 
                                                                         &psPosition->sPos7);
        }

        /* 每周期推进一步：得到位置规划输出 */
        psPosition->fltPositionRamp = GFLIB_FlexSPos7_FLT(&psPosition->sPos7);
    }
    else
    {
        /* ========== 位置斜坡生成 ========== */
        if(psPosition->sPositionRampParams.fltRampUp == 0.0f &&
           psPosition->sPositionRampParams.fltRampDown == 0.0f)
        {
            psPosition->fltPositionRamp = psPosition->fltPositionCmd;
        }
        else
        {
            psPosition->fltPositionRamp =
                GFLIB_Ramp_FLT(psPosition->fltPositionCmd, &psPosition->sPositionRampParams);
        }
    
    }

    /* 位置误差计算 */
    psPosition->fltPositionError =
        MLIB_Sub_FLT(psPosition->fltPositionRamp, psPosition->fltPositionFilt);

    /* 位置 PI 控制器输出：得到转速给定 */
    psPosition->fltSpeedReq =
        GFLIB_CtrlPIpAW_FLT(psPosition->fltPositionError,
                            &psPosition->bPositionPiStopInteg,
                            &psPosition->sPositionPiParams);
}
#endif
#if 1

void MCS_PMSMFocCtrlPosition(mcs_position_t *psPosition)
{
    /* Position saturation flag given by the Q current controller saturation flag and speed controller saturation flag */
    psPosition->bPositionPiStopInteg = TRUE;

    
    if (psPosition->u8PosRampType == POS_S_RAMP)
    {
        /* 7 */
        float adv_speed = 0.0f ;
        float pid_out =0.0f ;
        if (psPosition->fltPositionCmd != psPosition->sPos7.fltTarget)
        {
            psPosition->sPos7.fltState = g_sM1Drive.sPosCtrl.fltPositionFilt;
            GFLIB_FlexSPos7CalcIncr_FLT_C(psPosition->fltPositionCmd, psPosition->sPos7.fltVel, 
                                                                         &psPosition->sPos7);
        /* psPosition->sPos7.fltTimeTotal        */
        }

        /*  */
        psPosition->fltPositionRamp = GFLIB_FlexSPos7_FLT(&psPosition->sPos7);
         /*  */
        psPosition->fltPositionError =
        MLIB_Sub_FLT(psPosition->fltPositionRamp, psPosition->fltPositionFilt);
        adv_speed = psPosition->sPos7.fltVel /6.0f /g_fltM1speedAngularScale * psPosition->fltAdvGain ;
        /* Desired speed by the Position PI controller */
        pid_out =GFLIB_CtrlPIpAW_FLT(psPosition->fltPositionError, &psPosition->bPositionPiStopInteg, &psPosition->sPositionPiParams)  ;
      
        psPosition->fltSpeedReq = pid_out + adv_speed; // rpm = 360.0f / 60.0f
    }
    else
    {
        /* ==========  ========== */
        if(psPosition->sPositionRampParams.fltRampUp == 0.0f &&
           psPosition->sPositionRampParams.fltRampDown == 0.0f)
        {
            psPosition->fltPositionRamp = psPosition->fltPositionCmd;
        }
        else
        {
            psPosition->fltPositionRamp =
                GFLIB_Ramp_FLT(psPosition->fltPositionCmd, &psPosition->sPositionRampParams);
        }
         /*  */
        psPosition->fltPositionError =
        MLIB_Sub_FLT(psPosition->fltPositionRamp, psPosition->fltPositionFilt);

        /* Desired speed by the Position PI controller */
        psPosition->fltSpeedReq =
        GFLIB_CtrlPIpAW_FLT(psPosition->fltPositionError, &psPosition->bPositionPiStopInteg, &psPosition->sPositionPiParams);
    
    }

   

    /* if(psPosition->bPositionLimt)
         psPosition->fltSpeedReq = 0;*/
}
#endif
/*!
 * @brief PMSM 2-step rotor alignment - 120deg in first step and 0deg in second.
 *
 * This function is used for alignment rotor in two steps - 120deg in first step and 0deg in second
 *
 * @param psFocPMSM     The pointer of the PMSM FOC structure
 * @param psAlignment   The pointer of the motor control alignment structure
 *
 * @return None
 */
void MCS_PMSMAlignment(mcs_alignment_t *psAlignment)
{
    /* first half duration time is position set to 120 degree */
    if (psAlignment->ui16TimeHalf > 0)
    {
        if (psAlignment->bDirection)
            psAlignment->f16PosAlign = psAlignment->f16PosStep1;
        else
            psAlignment->f16PosAlign = psAlignment->f16PosStep2;
        psAlignment->ui16TimeHalf--;
    }
    else
    {
        psAlignment->f16PosAlign = FRAC16(0.0);
    }
}

/*!
 * @brief PMSM Open Loop Start-up
 *
 * This function is used to PMSM Open Loop Start-up
 *
 * @param psStartUp     The pointer of the PMSM open loop start up parameters structure
 *
 * @return None
 */
void MCS_PMSMOpenLoopStartUp(mcs_pmsm_startup_t *psStartUp)
{
    /* Open loop startup speed ramp */
    psStartUp->fltSpeedRampOpenLoop = GFLIB_Ramp_FLT(psStartUp->fltSpeedReq,
                                                     &psStartUp->sSpeedRampOpenLoopParams);

    /* generation of open loop position from the required speed */
    psStartUp->f16PosGen = GFLIB_Integrator_F16(MLIB_ConvSc_F16ff(psStartUp->fltSpeedRampOpenLoop, psStartUp->fltSpeedMax),
                                                &psStartUp->sSpeedIntegrator);

    /* position merging starts above merging speed threshold*/
    if (MLIB_Abs_FLT(psStartUp->fltSpeedRampOpenLoop) >= psStartUp->fltSpeedCatchUp)
    {
        /* increment position merging coefficient */
        psStartUp->f16RatioMerging = MLIB_AddSat_F16(psStartUp->f16RatioMerging, psStartUp->f16CoeffMerging);

        /* merging equation */
        psStartUp->f16PosMerged = MLIB_Add_F16(
            psStartUp->f16PosGen,
            MLIB_Mul_F16(MLIB_Sub_F16(psStartUp->f16PosEst, psStartUp->f16PosGen), psStartUp->f16RatioMerging));
    }
    else
    {
        psStartUp->f16PosMerged = psStartUp->f16PosGen;
    }

    /* clear open loop flag */
    if (psStartUp->f16RatioMerging == FRAC16(1.0))
        psStartUp->bOpenLoop = FALSE;
}

/*!
 * @brief PMSM scalar control, voltage is set based on required speed
 *
 * This function is used for alignment rotor in two steps - 120deg in first step and 0deg in second
 *
 * @param psFocPMSM      The pointer of the PMSM FOC structure
 * @param psScalarPMSM   The pointer of the PMSM scalar control structure
 *
 * @return None
 */
void MCS_PMSMScalarCtrl(mcs_pmsm_scalar_ctrl_t *psScalarPMSM)
{
    /* this part of code is executed when scalar control is turned-on */
    /* frequency ramp */
    psScalarPMSM->fltFreqRamp =
        GFLIB_Ramp_FLT(psScalarPMSM->fltFreqCmd, &psScalarPMSM->sFreqRampParams);

    /* voltage calculation */
    psScalarPMSM->sUDQReq.fltQ = psScalarPMSM->fltVHzGain * psScalarPMSM->fltFreqRamp;
    psScalarPMSM->sUDQReq.fltD = 0.0F;

    /* stator voltage angle , used the same integrator as for the open-loop start up*/
    /* stator voltage angle , used the same integrator as for the open-loop start up*/
    psScalarPMSM->f16PosElScalar = GFLIB_Integrator_F16(MLIB_ConvSc_F16ff(psScalarPMSM->fltFreqRamp, psScalarPMSM->fltFreqMax),
                                                        &psScalarPMSM->sFreqIntegrator);
}

/*!
 * @brief B2 死区补偿初始化
 *
 * 电流断点按 M1_I_MAX；管压降初值按 Rds≈19mΩ；
 * deadtime_gain 按 Td、fpwm、Vdc_nom 估算，上机后按过零畸变再调。
 */
void MCS_DeadtimeComp_Init(void)
{
    cl_deadtime_comp_cfg_t cfg;
    const float_t fltIbase = M1_I_MAX;
    const float_t fltUbase = M1_U_MAX;
    const float_t fltRds = 0.019F;
    const float_t fltTd = M1_DT_TD_SEC;
    const float_t fltVdt = M1_DT_VDC_NOM * fltTd * (float_t)M1_PWM_FREQ;
    float_t fltI;
    uint16_t k;

    cfg.deadtime_gain_q15 = (int32_t)(fltVdt / fltUbase * (float_t)CL_Q15_ONE);
    cfg.vdc_nom_q15 = CL_Q15_ONE;
    cfg.zero_band_q15 = (int32_t)(0.02F * (float_t)CL_Q15_ONE);
    cfg.zero_hyst_q15 = (int32_t)(0.005F * (float_t)CL_Q15_ONE);
    cfg.out_limit_q15 = (int32_t)(0.05F * (float_t)CL_Q15_ONE);

    cfg.current_bp_q15[0] = 0;
    cfg.current_bp_q15[1] = (int32_t)(0.10F * (float_t)CL_Q15_ONE);
    cfg.current_bp_q15[2] = (int32_t)(0.30F * (float_t)CL_Q15_ONE);
    cfg.current_bp_q15[3] = (int32_t)(0.60F * (float_t)CL_Q15_ONE);
    cfg.current_bp_q15[4] = CL_Q15_ONE;

    for (k = 0; k < CL_DEADTIME_LUT_SIZE; k++)
    {
        fltI = ((float_t)cfg.current_bp_q15[k] / (float_t)CL_Q15_ONE) * fltIbase;
        cfg.drop_lut_q15[k] = (int32_t)((fltI * fltRds) / fltUbase * (float_t)CL_Q15_ONE);
    }

    DeadtimeComp_Init(&g_sDeadtimeComp, &cfg);
}

void MCS_DeadtimeComp_Reset(void)
{
    DeadtimeComp_Reset(&g_sDeadtimeComp);
}

/*!
 * @brief B1 dq 解耦初始化
 *
 * ld/lq/flux_gain 按：
 *   ld_gain = Ld * Ibase * ωbase / Vbase
 *   flux_gain = ψf * ωbase / Vbase
 * ωbase 取 M1_N_MAX（电气 rad/s），与 sSpeed 电角速度标幺一致。
 */
void MCS_DqDecouple_Init(void)
{
    cl_dq_decouple_cfg_t cfg;
    const float_t fltIbase = M1_I_MAX;
    const float_t fltUbase = M1_U_MAX;
    const float_t fltWbase = M1_N_MAX;
    const float_t fltLd = M1_Ld;
    const float_t fltLq = M1_Lq;
    const float_t fltPsi = M1_PSI_F;
    float_t fltGain;

    fltGain = (fltLd * fltIbase * fltWbase) / fltUbase;
    cfg.ld_gain_q15 = (int32_t)(fltGain * (float_t)CL_Q15_ONE);

    fltGain = (fltLq * fltIbase * fltWbase) / fltUbase;
    cfg.lq_gain_q15 = (int32_t)(fltGain * (float_t)CL_Q15_ONE);

    fltGain = (fltPsi * fltWbase) / fltUbase;
    cfg.flux_gain_q15 = (int32_t)(fltGain * (float_t)CL_Q15_ONE);

    cfg.filter_alpha_q15 = (int32_t)(0.50F * (float_t)CL_Q15_ONE);
    cfg.omega_deadband_q15 = (int32_t)(0.02F * (float_t)CL_Q15_ONE);
    cfg.out_limit_q15 = (int32_t)(0.50F * (float_t)CL_Q15_ONE);

    DqDecouple_Init(&g_sDqDecouple, &cfg);
}

void MCS_DqDecouple_Reset(void)
{
    DqDecouple_Reset(&g_sDqDecouple);
}

void MCS_SrfHarm_Init(void)
{
    cl_srf_harm_cfg_t cfg;
    /* all_demo 保守初值：仅 5/7；Ki 按 16kHz→20kHz 比例约 0.8 */
    cfg.n_harm = 4u;
    cfg.order[0] = 5u;
    cfg.order[1] = 7u;
    cfg.order[2] = 11u;
    cfg.order[3] = 13u;
    cfg.dir[0] = -1;
    cfg.dir[1] = 1;
    cfg.dir[2] = -1;
    cfg.dir[3] = 1;
    cfg.enable[0] = 1u;
    cfg.enable[1] = 1u;
    cfg.enable[2] = 0u;
    cfg.enable[3] = 0u;
    cfg.kp_q15[0] = 32768;
    cfg.kp_q15[1] = 32768;
    cfg.kp_q15[2] = 32768;
    cfg.kp_q15[3] = 32768;
    cfg.ki_q15[0] = 213; /* 266 * 16/20 */
    cfg.ki_q15[1] = 230; /* 287 * 16/20 */
    cfg.ki_q15[2] = 213;
    cfg.ki_q15[3] = 230;
    cfg.rs_q15 = 0;
    cfg.ld_q10 = 0;
    cfg.lq_q10 = 0;
    cfg.omega_e_q15 = 0;
    cfg.lpf_enable = 1u;
    cfg.lpf_preset = 0u; /* 25 rad/s @16kHz 表；20kHz 略偏快，先保守 */
    cfg.impedance_enable = 0u;
    cfg.state_limit_q15 = 1000;
    cfg.out_limit_q15 = 300;

    SrfHarm_Init(&g_sSrfHarm, &cfg);
}

void MCS_SrfHarm_Reset(void)
{
    SrfHarm_Reset(&g_sSrfHarm);
}

/*!
 * @brief 将 B1 前馈叠加到 dq 电压指令（电流 PI 之后、逆 Park 之前）
 */
static void MCS_ApplyB1DqDecouple(mcs_pmsm_foc_t *psFocPMSM)
{
    cl_dq_decouple_io_t sIo;
    const float_t fltIscale = (float_t)CL_Q15_ONE / M1_I_MAX;
    const float_t fltWscale = (float_t)CL_Q15_ONE / M1_N_MAX;
    const float_t fltUscale = M1_U_MAX / (float_t)CL_Q15_ONE;
    float_t fltOmegaEl;

    fltOmegaEl = g_sM1Drive.sSpeed.fltSpeedFilt;

    sIo.id_q15 = (int32_t)(psFocPMSM->sIDQ.fltD * fltIscale);
    sIo.iq_q15 = (int32_t)(psFocPMSM->sIDQ.fltQ * fltIscale);
    sIo.omega_q15 = (int32_t)(fltOmegaEl * fltWscale);

    DqDecouple_Apply(&g_sDqDecouple, &sIo);

    psFocPMSM->sUDQReq.fltD += (float_t)sIo.vd_ff_q15 * fltUscale;
    psFocPMSM->sUDQReq.fltQ += (float_t)sIo.vq_ff_q15 * fltUscale;
}

/*!
 * @brief 将 B2 三相补偿叠加到 αβ 电压指令（逆 Park 之后、SVPWM 之前）
 */
static void MCS_ApplyB2DeadtimeComp(mcs_pmsm_foc_t *psFocPMSM)
{
    cl_deadtime_comp_io_t sIo;
    GMCLIB_3COOR_T_FLT sUAbcFf;
    GMCLIB_2COOR_ALBE_T_FLT sUAlBeFf;
    const float_t fltIscale = (float_t)CL_Q15_ONE / M1_I_MAX;
    const float_t fltUscale = M1_U_MAX / (float_t)CL_Q15_ONE;
    const float_t fltVdcNom = M1_DT_VDC_NOM;

    sIo.iu_q15 = (int32_t)(psFocPMSM->sIABC.fltA * fltIscale);
    sIo.iv_q15 = (int32_t)(psFocPMSM->sIABC.fltB * fltIscale);
    sIo.iw_q15 = (int32_t)(psFocPMSM->sIABC.fltC * fltIscale);
    if (fltVdcNom > 0.0F)
    {
        sIo.vdc_q15 = (int32_t)(psFocPMSM->fltUDcBusFilt / fltVdcNom * (float_t)CL_Q15_ONE);
    }
    else
    {
        sIo.vdc_q15 = 0;
    }

    DeadtimeComp_Apply(&g_sDeadtimeComp, &sIo);

    /* 极性与 PWM/电流正方向相反时需取反（文档要求先小增益验极性） */
    sUAbcFf.fltA = -(float_t)sIo.vu_ff_q15 * fltUscale;
    sUAbcFf.fltB = -(float_t)sIo.vv_ff_q15 * fltUscale;
    sUAbcFf.fltC = -(float_t)sIo.vw_ff_q15 * fltUscale;

    GMCLIB_Clark_FLT(&sUAbcFf, &sUAlBeFf);
    psFocPMSM->sUAlBeDTComp.fltAlpha += sUAlBeFf.fltAlpha;
    psFocPMSM->sUAlBeDTComp.fltBeta += sUAlBeFf.fltBeta;
}

/*!
 * @brief C2–C4：αβ 电流误差进 SRF，补偿电压叠到逆 Park 后的 αβ
 */
static void MCS_ApplyC2SrfHarm(mcs_pmsm_foc_t *psFocPMSM)
{
    cl_srf_harm_io_t sIo;
    GMCLIB_2COOR_DQ_T_FLT sIDQErr;
    GMCLIB_2COOR_ALBE_T_FLT sIAlBeErr;
    const float_t fltIscale = (float_t)CL_Q15_ONE / M1_I_MAX;
    const float_t fltUscale = M1_U_MAX / (float_t)CL_Q15_ONE;

    sIDQErr.fltD = psFocPMSM->sIDQError.fltD;
    sIDQErr.fltQ = psFocPMSM->sIDQError.fltQ;
    GMCLIB_ParkInv_FLT(&sIDQErr, &psFocPMSM->sAnglePosEl, &sIAlBeErr);

    sIo.err_alpha_q15 = (int32_t)(sIAlBeErr.fltAlpha * fltIscale);
    sIo.err_beta_q15 = (int32_t)(sIAlBeErr.fltBeta * fltIscale);
    sIo.theta_q16 = (uint16_t)psFocPMSM->f16PosEl;
    sIo.u_alpha_q15 = 0;
    sIo.u_beta_q15 = 0;

    SrfHarm_Apply(&g_sSrfHarm, &sIo);

    psFocPMSM->sUAlBeDTComp.fltAlpha += (float_t)sIo.u_alpha_q15 * fltUscale;
    psFocPMSM->sUAlBeDTComp.fltBeta += (float_t)sIo.u_beta_q15 * fltUscale;
}

/*!
 * @brief Dead-time compensation using LUT wit interpolation
 *
 * This function is used for output phase voltage dead time compensation
 *
 * @param sUAlBeDTComp   The pointer to alpha/beta voltages
 *        sIABC   The pointer to measured phase currents
 *        fltUDcBusFilt  DC-Bus filtered
 *        fltPwrStgCharIRange   Current range of sampling circuity
 *        fltPwrStgCharLinCoeff   Coefficient
 *
 * @return None
 */
/******************************************************************************
@brief   Dead-time compensation using LUT wit interpolation

@param   N/A

@return  N/A
******************************************************************************/
#if 0
 void MCS_DTComp(GMCLIB_2COOR_ALBE_T_FLT *sUAlBeDTComp, 
                GMCLIB_3COOR_T_FLT *sIABC,
                float_t fltUDcBusFilt,
                float_t fltPwrStgCharIRange,
                float_t fltPwrStgCharLinCoeff)
{

    register GMCLIB_3COOR_T_FLT sUABCErr;
    register float_t            fltUerrMax;
    register int16_t            i16CurrSign;

    /* maximal error voltage */
    fltUerrMax = *pfltUDtComp;

    /* compensate phase A */
    i16CurrSign = (sIABC->fltA > fltPwrStgCharIRange) - 
                  (sIABC->fltA < -fltPwrStgCharIRange);
    if (!i16CurrSign)
        sUABCErr.fltA = GFLIB_Lut1D_FLT(sIABC->fltA, 
                                           pfltUDtComp, 
                                           &sLUTUDtComp);
    else
        sUABCErr.fltA = i16CurrSign * 
            ((MLIB_Abs_FLT(sIABC->fltA) - fltPwrStgCharIRange) * 
             fltPwrStgCharLinCoeff - fltUerrMax);

    /* compensate phase B */
    i16CurrSign = (sIABC->fltB > fltPwrStgCharIRange) - 
                  (sIABC->fltB < -fltPwrStgCharIRange);
    if (!i16CurrSign)
        sUABCErr.fltB = GFLIB_Lut1D_FLT(sIABC->fltB,
                                           pfltUDtComp, 
                                           &sLUTUDtComp);
    else
        sUABCErr.fltB = i16CurrSign * 
          ((MLIB_Abs_FLT(sIABC->fltB) - fltPwrStgCharIRange) * 
           fltPwrStgCharLinCoeff - fltUerrMax);

    /* compensate phase C */
    i16CurrSign = (sIABC->fltC > fltPwrStgCharIRange) - 
                  (sIABC->fltC < -fltPwrStgCharIRange);
    if (!i16CurrSign)
        sUABCErr.fltC = GFLIB_Lut1D_FLT(sIABC->fltC, 
                                           pfltUDtComp, 
                                           &sLUTUDtComp);
    else
        sUABCErr.fltC = i16CurrSign * 
          ((MLIB_Abs_FLT(sIABC->fltC) - fltPwrStgCharIRange) * 
           fltPwrStgCharLinCoeff - fltUerrMax);

    /* add compensation voltages */
    sUAlBeDTComp->fltAlpha += (0.333333333333F) * fltUDcBusFilt * 
                               (sUABCErr.fltA + sUABCErr.fltA - 
                                sUABCErr.fltB - sUABCErr.fltC);

    sUAlBeDTComp->fltBeta += (0.5773502691896F) * fltUDcBusFilt * 
                              (sUABCErr.fltB - sUABCErr.fltC);

}
#endif
