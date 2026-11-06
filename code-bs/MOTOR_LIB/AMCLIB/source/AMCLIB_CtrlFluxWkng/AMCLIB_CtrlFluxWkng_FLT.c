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
#include "AMCLIB_CtrlFluxWkng_FLT.h"

/***************************************************************************//*!
*
* @brief  Flux weakening control to achieve higher motor speed.
*
* @param  ptr - AMCLIB_CTRL_FLUX_WKNG_T_FLT *psCtrl 
*               - pointer to the structure of the FW controller 
* @param  in  - float_t fltIQErr - Iq current error 
*             - float_t fltUQReq - Uq required voltage 
*             - float_t fltUQLim - Uq voltage limit (fltUQLim > 0)
*
* @return float_t - Id required current  
*
* @remarks  The function calculates flux weakening algorithms to spread motor speed range,
*           according to following equations :
*
*   U_Error  = (Uq_Limit - |Uq_Req|) * I_Gain / U_Gain
*   FW_Error = U_Error - |Iq_Error| 
*   output   = PI controller (FW_Error)
*
****************************************************************************/
float_t AMCLIB_CtrlFluxWkng_FLT_FC(float_t fltIQErr, float_t fltUQReq, float_t fltUQLim,
                                   AMCLIB_CTRL_FLUX_WKNG_T_FLT *psCtrl)
{
    register float_t fltTemp; 
    register float_t fltUFWErr;  
    register float_t fltFWErr; 
     
    /* Voltage error calculation */
    fltUFWErr = MLIB_Mul_FLT(MLIB_Sub_FLT(fltUQLim, MLIB_Abs_FLT(fltUQReq)),psCtrl->fltIGainUgain);    
          
    /* Iq current error filtering */
    fltTemp = GDFLIB_FilterIIR1_FLT(MLIB_Abs_FLT(fltIQErr), &psCtrl->sIqErrIIR1Param); 
    
    /* Fiel weakening error calculation */
    fltFWErr = MLIB_Sub_FLT(fltUFWErr, fltTemp);  
    
    /* Storing errors into structure */
    psCtrl->fltIqErrIIR1 = (float_t) fltTemp; 
    psCtrl->fltUFWErr    = (float_t) fltUFWErr;         
    psCtrl->fltFWErr     = (float_t) fltFWErr;   
    
    /* Flux weakening controller */
    fltTemp = GFLIB_CtrlPIpAW_FLT(fltFWErr, psCtrl->pbStopIntegFlag, &psCtrl->sFWPiParam);  
    
    /* return Iq required */    
    return (fltTemp);    
}
