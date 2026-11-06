/*******************************************************************************
*
* Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
* Copyright 2016-2021 NXP
*
 * License: LA_OPT_NXP_Software_License
 *
 * NXP Confidential. This software is owned or controlled by NXP and may
 * only be used strictly in accordance with the applicable license terms.
 * By expressly accepting such terms or by downloading, installing,
 * activating and/or otherwise using the software, you are agreeing that
 * you have read, and that you agree to comply with and are bound by,
 * such license terms. If you do not agree to be bound by the applicable
 * license terms, then you may not retain, install, activate or otherwise
 * use the software.
* 
*
****************************************************************************//*!
*
* @brief Decoupling calculation 
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GMCLIB_Decoupling_FLT.h"

/*******************************************************************************
* 
* @brief    This function calculates decoupled d-axis and q-axis voltage commands
*           of PMSM. The d,q axis decoupling terms are added to controller outputs.
*
* @param  ptr - GMCLIB_2COOR_DQ_T_FLT *psUDQ - IN  - pointer to the structure of
*               d-axis and q-axis input voltages from d-axis and q-axis controllers
*             - GMCLIB_2COOR_DQ_T_FLT *psIDQ - IN  - pointer to the structure of   
*               d-axis and q-axis input current from d-axis and q-axis controllers
*             - GMCLIB_DECOUPLINGPMSM_T_A32 *psParam - IN  - pointer to the structure
*               of motor parameters needed for decoupling scaled to 16-bit frac. range.                    
*               The members of the structure are:
*                   - float_t fltLd = Ld - inductance in D axis                                        
*                   - float_t fltLq = Lq - inductance in Q axis              
*             - GMCLIB_2COOR_DQ_T_FLT *psUDQDec - OUT - pointer to the structure of 
*               decoupled d-axis and q-axis voltage commands
* @param  in  - frac16_t fltSpeedEl  - actual measured motor electric speed
*
* @remarks: The decoupling algorithm is calculated according to the following equations:
*           ud_dec = ud - Lq * fltSpeedEl * iq        (1)        
*           uq_dec = uq + Ld * fltSpeedEl * id        (2)
*   
*******************************************************************************/
void GMCLIB_DecouplingPMSM_FLT_FC(const GMCLIB_2COOR_DQ_T_FLT *psUDQ,
                                  const GMCLIB_2COOR_DQ_T_FLT *psIDQ,
                                  float_t fltSpeedEl,
                                  const GMCLIB_DECOUPLINGPMSM_T_FLT *psParam,
                                  GMCLIB_2COOR_DQ_T_FLT *psUDQDec)
{      
    /* Decoupled D component - calculate ud_dec = ud - Lq * f16SpeedEl * iq  */
    psUDQDec->fltD = MLIB_Msu_FLT(psUDQ->fltD, MLIB_Mul_FLT(fltSpeedEl, psIDQ->fltQ), psParam->fltLq);
    
    /* Decoupled Q component - calculate uq_dec = uq + Ld * f16SpeedEl * id   */  
    psUDQDec->fltQ = MLIB_Mac_FLT(psUDQ->fltQ, MLIB_Mul_FLT(fltSpeedEl, psIDQ->fltD), psParam->fltLd);  
}
