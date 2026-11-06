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
* @brief  Float tracking observer   
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/ 
#include "AMCLIB_TrackObsrv_FLT.h"

/***************************************************************************//*!
*
* @brief    Tracking observer estimates the position (by accumulator type) 
*           using an angle error
*
* @param    in  - acc32_t a32ThetaErr - error of angle
* @param    ptr - AMCLIB_TRACK_OBSRV_T_FLT *psCtrl
*                 - frac32_t f32Theta  - Estimated position <-1;1) corresponds to <-pi;pi)
*                 - float_t  fltSpeed  - Estimated speed 
*                 - float_t  fltI_1    - Internal integrator 
*                 - float_t  fltIGain  - Integ. constant to get speed from error
*                 - float_t  fltPGain  - Prop. constant to get speed from angle
*                 - float_t  fltThGain - Constant to get angle from speed         
*
* @return  acc32_t a32Theta - estimated position where <-1;1) correspond <-pi;pi) 
*                         
* @remarks  The function calculates tracking observer to determine the angular 
*           speed and position of input error functional signal according to 
*           following equations:
*           
*   fltTmpI_1 = fltI_1    + (fltError * fltIGain)  
*   fltSpeed  = fltTmpI_1 + (fltError * fltPGain)
*   frac32_t  = frac32_t  + (fltSpeed * fltThGain) 
*                                   
****************************************************************************/
acc32_t AMCLIB_TrackObsrv_A32af_FC(acc32_t a32ThetaErr, 
                                   AMCLIB_TRACK_OBSRV_T_FLT *psCtrl)    
{
    register acc64_t a64Temp;    
    register frac32_t f32Theta;   
    register acc32_t a32TmpSpeed;
    register frac32_t f32ThGain; 
    register float_t fltTmpSpeed;
    register float_t fltTmpI_1 ;
    register float_t fltThetaErr;
    
    /* Store more times red quantity to register */
    fltThetaErr = MLIB_Conv_FLTa (a32ThetaErr); 
    
    /* Integration part of controller calculations */
    fltTmpI_1 = MLIB_Mac_FLT(psCtrl->fltI_1, fltThetaErr, psCtrl->fltIGain);
    
    /* Proportional part of controller calculations and Speed calculation  */  
    fltTmpSpeed = MLIB_Mac_FLT(fltTmpI_1, fltThetaErr, psCtrl->fltPGain);   
    
    /* Preparing acc32 for multiplication */
    a32TmpSpeed = MLIB_Conv_A32f (fltTmpSpeed);
    f32ThGain   = MLIB_Conv_F32f (psCtrl->fltThGain);
    
    /* Theta increment calculation */  
    a64Temp  = psCtrl->f32Theta;
    a64Temp += (acc64_t) (((int64_t) a32TmpSpeed) * ((int64_t) f32ThGain))>>15;    
    /* revolution (integer part of acc) removing */ 
    f32Theta = (frac32_t) a64Temp;
        
    /* Store values to structure members */ 
    psCtrl->f32Theta = f32Theta;    
    psCtrl->fltSpeed = fltTmpSpeed;
    psCtrl->fltI_1   = fltTmpI_1;
    
    return ((acc32_t)(f32Theta>>16));    
}
