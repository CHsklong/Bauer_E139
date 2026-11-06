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
* @brief  Float angle tracking observer 
* 
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/ 
#include "AMCLIB_AngleTrackObsrv_FLT.h"

/***************************************************************************//*!
*
* @brief  Tracks the angle (using accumulator) of the system and calculates the speed.
*
* @param  ptr - GMCLIB_2COORD_SINCOS_T_FLT *psSinCos - input angle in Sin/Cos 
*             - AMCLIB_ANGLE_TRACK_OBSRV_T_FLT *psCtrl   
*                 -  float_t  fltSpeed    - Estimated speed  
*                 -  frac32_t f32A2       - Internal integrator 
*                 -  acc32_t  a32Theta    - Estimated angle in acc32_t <-pi;pi) 
*                                           correspond frac16_t part <-1.0; 1.0)
*                -  float_t  fltSinEstim  - Sine of the estimated angle 
*                -  float_t  fltCosEstim  - Cosine of the estimated angle 
*                -  float_t  fltK1Gain    - Constant to get speed from error 
*                -  float_t  fltK2Gain    - Prop. constant to get angle from speed
*                -  float_t  fltA2Gain    - Integ. constant to get angle from speed 
*
* @return  acc32_t a32Theta - estimated position where <-1;1) correspond <-pi;pi) 
*
* @remarks  The function calculates angle tracking observer to determine the angular 
*           speed, estimated position (including sine and cosine values) of input angle
*           functional signal.
*
*   error = fltSin * fltCosEstim - fltCos * fltSinEstim
*   fltSpeed = fltSpeed + (error * fltK1Gain)  
*   A2 = A2 + (fltSpeed * fltA2Gain)
*   a32Theta = (fltSpeed * fltK2Gain) + A2
*   fltSinEstim = Sin (a32Theta)
*   fltCosEstim = Cos (a32Theta)             
*
****************************************************************************/
acc32_t AMCLIB_AngleTrackObsrv_A32ff_FC(const GMCLIB_2COOR_SINCOS_T_FLT *psSinCos,
                                        AMCLIB_ANGLE_TRACK_OBSRV_T_FLT *psCtrl)
{
    register acc64_t a64Temp; 
    register acc32_t a32Temp;   
    register acc32_t a32TmpSpeed;     
    register acc32_t a32A2Gain;   
    register acc32_t a32K2Gain;   
    register float_t fltSpeed;
    register float_t fltError;
    register float_t fltSinEstim, fltCosEstim;
    register float_t fltSin, fltCos;   
    
    /* Storing quantities to local constants */
    fltSin = psSinCos -> fltSin;
    fltCos = psSinCos -> fltCos;       
    fltSpeed = psCtrl -> fltSpeed;  
    fltSinEstim = psCtrl -> fltSinEstim;
    fltCosEstim = psCtrl -> fltCosEstim;     
    
    /* fltSin * fltCosEstim - fltCos * fltSinEstim calculation */
    fltError = MLIB_Msu4_FLT(fltSin, fltCosEstim, fltCos, fltSinEstim);
    
    /* Speed error calculations */
    fltSpeed = MLIB_Mac_FLT(fltSpeed, fltError, psCtrl -> fltK1Gain);
  
    /* Preparing acc32 for multiplication */
    a32TmpSpeed = MLIB_Conv_A32f (fltSpeed);
    a32A2Gain   = MLIB_Conv_A32f (psCtrl -> fltA2Gain);  
    a32K2Gain   = MLIB_Conv_A32f (psCtrl -> fltK2Gain); 
    
    /* A2 Speed calculation and Theta calculation */
    a64Temp  = (acc64_t)psCtrl -> f32A2;
    a64Temp += (acc64_t) (((int64_t) a32TmpSpeed) * ((int64_t) a32A2Gain))<<1;  
    psCtrl -> f32A2 = (frac32_t) a64Temp; 
    a64Temp += (acc64_t) (((int64_t) a32TmpSpeed) * ((int64_t) a32K2Gain))<<1;
    /* revolution (integer part of acc) removing */
    a32Temp = (acc32_t) a64Temp;                                                                                                        
    psCtrl -> a32Theta = a32Temp;  
    
    a32Temp = a32Temp >> 16;
    
    /* Sine, cosine estim values calculation */
    fltSinEstim = GFLIB_Sin_FLTa(a32Temp);
    fltCosEstim = GFLIB_Cos_FLTa(a32Temp);       
    
    /* Store quantities to structure */
    psCtrl -> fltSpeed = fltSpeed;
   
    psCtrl -> fltSinEstim = fltSinEstim;
    psCtrl -> fltCosEstim = fltCosEstim;
    
    return ((acc32_t)a32Temp);   
}

/***************************************************************************//*!
*
* @brief  Initialization for the angle tracking observer
*
* @param  in   -  acc32_t a32ThetaInit - initial value of the angle <-pi;pi) 
*                 correspond frac16_t part <-1.0; 1.0)
* @param  ptr  -  AMCLIB_ANGLE_TRACK_OBSRV1_T_F32 *psCtrl
*                 -  float_t  fltSpeed    - Estimated speed  
*                 -  frac32_t f32A2       - Internal integrator 
*                 -  acc32_t  a32Theta    - Estimated angle in acc32_t <-pi;pi) 
*                                           correspond frac16_t part <-1.0; 1.0)
*                 -  float_t  fltSinEstim - Sine of the estimated angle <-1;1)
*                 -  float_t  fltCosEstim - Cosine of the estimated angle <-1;1)
*                 -  float_t  fltK1Gain   - Constant to get speed from error 
*                 -  float_t  fltK2Gain   - Prop. constant to get angle from speed 
*                 -  float_t  fltA2Gain   - Integ. constant to get angle from speed 
*
* @return none
*
* @remarks  The function calculates initializes the angle tracking observer by following rules:
*           
*   f32Speed = 0
*   A2 = a32ThetaInit
*   a32Theta = a32ThetaInit
*   f16SinEstim = Sin (a32ThetaInit)
*   f16CosEstim = Cos (a32ThetaInit)
*
****************************************************************************/
void AMCLIB_AngleTrackObsrvInit_A32_FC(acc32_t a32ThetaInit, 
                                       AMCLIB_ANGLE_TRACK_OBSRV_T_FLT *psCtrl)
{
    psCtrl->fltSpeed = 0.0F ;
    psCtrl->f32A2 = a32ThetaInit<<16; 
    psCtrl->a32Theta = a32ThetaInit;   
    psCtrl->fltSinEstim = GFLIB_Sin_FLTa(a32ThetaInit);
    psCtrl->fltCosEstim = GFLIB_Cos_FLTa(a32ThetaInit);  
}
