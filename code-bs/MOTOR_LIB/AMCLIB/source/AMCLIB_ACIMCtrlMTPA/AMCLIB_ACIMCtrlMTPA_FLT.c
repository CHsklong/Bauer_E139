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
* @brief ACIM Optimal Rotor Field Excitation  
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "AMCLIB_ACIMCtrlMTPA_FLT.h"   

/***************************************************************************//*!
*
* @brief    ACIM Maximum Torque per Ampere control 
*
* @param    in   float_t fltIq - measured Q current                       
*           ptr  AMCLIB_ACIM_CTRL_MTPA_T_FLT *psCtrl - pointer to the function parameter structure 
*
* @return   float_t - required D current (flux exciting)
*       
* @remarks  The function calculates the optimum rotor flux exciting for maximum efficiency (defined by 
*           required D current output) from the absolute value of measured Q current by filtering using 
*           the exponential filter and output minimum and maximum current limitation. 
*
*           Equations:
*           Id = FilterExp (Abs(Iq))
*           Id = limit (Id)                    
*
****************************************************************************/
float_t AMCLIB_ACIMCtrlMTPA_FLT_FC(float_t fltIq,
                                   AMCLIB_ACIM_CTRL_MTPA_T_FLT *psCtrl)
{
    register float_t fltTemp;

    /* input filtering by exponential filter */
    fltTemp = GDFLIB_FilterExp_FLT (MLIB_Abs_FLT(fltIq), &psCtrl->fltIdExpParam); 

    /* output limitation */
    fltTemp = GFLIB_Limit_FLT(fltTemp, psCtrl->fltLowerLim, psCtrl->fltUpperLim);    
     
    /* return limited value */
    return (fltTemp);
}
