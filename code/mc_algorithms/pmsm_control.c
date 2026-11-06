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
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* dead-time compensation voltage table */
float_t pfltUDtComp[DTCOMP_TABLE_SIZE] = DTCOMP_TABLE_DATA;

/* dead-time compensation look-up table */
GFLIB_LUT1D_T_FLT sLUTUDtComp;

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
    /* pass electrical position from outside function if enabled else estimated
     * electrical position is selected */
    if (psFocPMSM->bPosExtOn)
        psFocPMSM->f16PosEl = psFocPMSM->f16PosElExt;                           //有传感器，用编码器读取角度
    else
        psFocPMSM->f16PosEl = psFocPMSM->f16PosElEst;                           //无传感器，用观测器观测
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
    GMCLIB_Clark_FLT(&psFocPMSM->sIABC, &psFocPMSM->sIAlBe);                    //clarke变化（nxp把基础数学变化写成了宏定义）
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
        psFocPMSM->f16PosEl = 0;                                //bTest置1，强制电角度为0
    if (psFocPMSM->bOpenLoop || psFocPMSM->bPosExtOn)           //park变化
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
    psFocPMSM->sIDQ.fltD = -psFocPMSM->sIDQ.fltD;               //q、d轴电流取反
    psFocPMSM->sIDQ.fltQ = -psFocPMSM->sIDQ.fltQ;
#endif
    /* perform current control loop if enabled */
    if (psFocPMSM->bCurrentLoopOn)                              //电流环闭环控制
    {
        /* D current error calculation */
        psFocPMSM->sIDQError.fltD = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltD, psFocPMSM->sIDQ.fltD);      //D轴电流误差

        /* Q current error calculation */
        psFocPMSM->sIDQError.fltQ = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltQ, psFocPMSM->sIDQ.fltQ);      //Q轴电流误差

        /*** D - controller limitation calculation ***/
        psFocPMSM->sIdPiParams.fltLowerLim = MLIB_MulNeg_FLT(psFocPMSM->fltDutyCycleLimit, psFocPMSM->fltUDcBusFilt);   //D轴限幅
        psFocPMSM->sIdPiParams.fltUpperLim = MLIB_Mul_FLT(psFocPMSM->fltDutyCycleLimit, psFocPMSM->fltUDcBusFilt);

        /* D current PI controller */
        psFocPMSM->sUDQReq.fltD =
            GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltD, &psFocPMSM->bIdPiStopInteg, &psFocPMSM->sIdPiParams);    //D轴PI控制器

        /*** Q - controller limitation calculation ***/
        psFocPMSM->sIqPiParams.fltUpperLim = GFLIB_Sqrt_FLT(                                                        //Q轴限幅
            psFocPMSM->sIdPiParams.fltUpperLim * psFocPMSM->sIdPiParams.fltUpperLim -
            psFocPMSM->sUDQReq.fltD * psFocPMSM->sUDQReq.fltD);
        psFocPMSM->sIqPiParams.fltLowerLim = MLIB_Neg_FLT(psFocPMSM->sIqPiParams.fltUpperLim);

        /* Q current PI controller */
        psFocPMSM->sUDQReq.fltQ =
            GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltQ, &psFocPMSM->bIqPiStopInteg, &psFocPMSM->sIqPiParams);    //Q轴PI控制器
    }

    /* 2-phase to 2-phase transformation to stationary ref. frame */
    GMCLIB_ParkInv_FLT(&psFocPMSM->sUDQReq, &psFocPMSM->sAnglePosEl, &psFocPMSM->sUAlBeReq);    //反park变化

    /* dead-time compensation */

    psFocPMSM->sUAlBeDTComp = psFocPMSM->sUAlBeReq;
#if 0
    if(psFocPMSM->bFlagDTComp)
    {
        MCS_DTComp(&psFocPMSM->sUAlBeDTComp,
                   &psFocPMSM->sIABC,
                   psFocPMSM->fltUDcBusFilt,
                   psFocPMSM->fltPwrStgCharIRange,
                   psFocPMSM->fltPwrStgCharLinCoeff);
    }
#endif
    /* DCBus ripple elimination */
    GMCLIB_ElimDcBusRipFOC_F16ff(psFocPMSM->fltUDcBusFilt, &psFocPMSM->sUAlBeDTComp, &psFocPMSM->sUAlBeCompFrac);     //母线电压纹波消除

    /* space vector modulation */
// psFocPMSM->ui16SectorSVM = GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);
#if (BOARD_TYPE == BOARD_E145)
    GMCLIB_SvmStd2_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);
#else
    GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);                      //7段式SVPWM输出（空间矢量调制）
#endif
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
                                 (MLIB_Abs_FLT(psSpeed->fltSpeedCmd) >= MLIB_Abs_FLT(psSpeed->fltSpeedFilt));     //速度环pi饱和标识
    if (g_sM1Drive.eControl == kControlMode_SpeedFOC)                                                             
    {
        if (psSpeed->u8RampType == SPEED_S_RAMP)                                  //速度环S型曲线平滑加速
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
        else if (psSpeed->u8RampType == SPEED_TD_RAMP)                              //抗扰动平滑加速
        {
            TD(&td_set_spd, psSpeed->fltSpeedCmd);
            psSpeed->fltSpeedRamp = td_set_spd.x1;
        }
        else                                                                       //线性斜坡加速
        {
            /* Speed ramp generation */
            psSpeed->fltSpeedRamp = GFLIB_Ramp_FLT(psSpeed->fltSpeedCmd, &psSpeed->sSpeedRampParams);
        }
    }
    else
    {
        /* Speed ramp generation */
       // psSpeed->fltSpeedRamp = GFLIB_Ramp_FLT(psSpeed->fltSpeedCmd, &psSpeed->sSpeedRampParams);
         psSpeed->fltSpeedRamp = psSpeed->fltSpeedCmd ;                               //非速度环模式，用位置环给的速度
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
        /* 7 */                                         //位置环7段式S型曲线规划
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
        MLIB_Sub_FLT(psPosition->fltPositionRamp, psPosition->fltPositionFilt);       //计算位置误差
        adv_speed = psPosition->sPos7.fltVel /6.0f /g_fltM1speedAngularScale * psPosition->fltAdvGain ;       //计算速度前馈
        /* Desired speed by the Position PI controller */
        pid_out =GFLIB_CtrlPIpAW_FLT(psPosition->fltPositionError, &psPosition->bPositionPiStopInteg, &psPosition->sPositionPiParams)  ;  //PI反馈控制
      
        psPosition->fltSpeedReq = pid_out + adv_speed; // rpm = 360.0f / 60.0f     //计算出的速度前馈加入到速度环给定值
    }
    else                                                    
    {
        /* ==========  ========== */                                  //线性斜坡规划
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
void MCS_PMSMAlignment(mcs_alignment_t *psAlignment)                        //转子预定位，用于增量式编码器或者无感控制
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
