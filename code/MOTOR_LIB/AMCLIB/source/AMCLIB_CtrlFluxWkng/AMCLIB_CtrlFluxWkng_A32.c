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
* @brief  Flux weakening controller 
* 
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/ 
#include "AMCLIB_CtrlFluxWkng_A32.h"

/***************************************************************************//*!
*
* @brief  Flux weakening control to achieve higher motor speed.
*
* @param  ptr - AMCLIB_CTRL_FLUX_WKNG_T_A32 *psCtrl 
*               - pointer to the structure of the FW controller 
* @param  in  - frac16_t f16IQErr - Iq current error <-1;1)
*             - frac16_t f16UQReq - Uq required voltage <-1;1)
*             - frac16_t f16UQLim - Uq voltage limit (0;1)
*
* @return frac16_t - Id required current <-1;1) 
*
* @remarks  The function calculates flux weakening algorithms to spread motor speed range,
*           according to following equations :
*
*   U_Error  = (Uq_Limit - |Uq_Req|) 
*   FW_Error = U_Error - |Iq_Error| 
*   output   = PI controller (FW_Error)
*
****************************************************************************/
frac16_t AMCLIB_CtrlFluxWkng_F16_FC(frac16_t f16IQErr, frac16_t f16UQReq, frac16_t f16UQLim,
                                    AMCLIB_CTRL_FLUX_WKNG_T_A32 *psCtrl)
{
    register frac32_t f32Temp; 
    register frac32_t f32UFWErr;  
    register frac32_t f32FWErr; 
     
    /* Voltage error calculation */
    f32UFWErr = MLIB_Sub_F16(f16UQLim, MLIB_AbsSat_F16(f16UQReq));    
          
    /* Iq current error filtering */
    f32Temp = GDFLIB_FilterIIR1_F16(MLIB_AbsSat_F16(f16IQErr), &psCtrl->sIqErrIIR1Param); 
    
    /* Field weakening error calculation */
    f32FWErr = MLIB_SubSat_F16(f32UFWErr, f32Temp);  
    
    /* Storing errors into structure to be visible */
    psCtrl->f16IqErrIIR1 = (frac16_t) f32Temp; 
    psCtrl->f16UFWErr    = (frac16_t) f32UFWErr;         
    psCtrl->f16FWErr     = (frac16_t) f32FWErr;   
    
    /* Flux weakening controller */
    f32Temp = GFLIB_CtrlPIpAW_F16(f32FWErr, psCtrl->pbStopIntegFlag, &psCtrl->sFWPiParam);  
    
    /* return Iq required */    
    return ((frac16_t)f32Temp);    
}
