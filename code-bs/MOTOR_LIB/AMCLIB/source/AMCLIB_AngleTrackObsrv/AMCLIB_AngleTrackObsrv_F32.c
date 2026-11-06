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
* @brief  Angle tracking observer 
* 
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/ 
#include "AMCLIB_AngleTrackObsrv_F32.h"

/***************************************************************************//*!
*
* @brief  Tracks the angle of the system and calculates the speed.
*
* @param  ptr - GMCLIB_2COORD_SINCOS_T_F16 *psSinCos - input angle in Sin/Cos 
*             - AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl
*               - frac32_t f32Speed    - Estimated speed <-1;1)
*               - frac32_t f32A2       - Internal integrator <-1;1)
*               - frac16_t f16Theta    - Estimated angle <-1;1) corresponds to <-pi;pi)
*               - frac16_t f16SinEstim - Sine of the estimated angle <-1;1)
*               - frac16_t f16CosEstim - Cosine of the estimated angle <-1;1)
*               - frac16_t f16K1Gain   - Constant to get speed from error <0;1)
*               - int16_t  i16K1GainSh - Shift for f16K1Gain <-15;15>
*               - frac16_t f16K2Gain   - Prop. constant to get angle from speed <0;1)
*               - int16_t  i16K2GainSh - Shift for f16K2Gain <-15;15>
*               - frac16_t f16A2Gain   - Integ. constant to get angle from speed <0;1)
*               - int16_t  i16A2GainSh - Shift for f16A2Gain <-15;15>
*
* @param  in    None
*
* @return frac16_t - the calculated angle in frac16_t <-1;1) 
*
* @remarks  The function calculates angle tracking observer to determine the angular 
*           speed, estimated position (including sine and cosine values) of input angle
*           functional signal by following:
*
*   error = sin (inAngle-EstimAngle) = f16Sin * f16CosEstim - f16Cos * f16SinEstim
*
*   f32Speed = f32Speed + [(error * f16K1Gain) << i16K1GainSh] 
*
*   A2 = A2 + [(f32Speed * f16A2Gain) << i16A2GainSh]
*
*   f16Theta = rnd{[(f32Speed * f16K2Gain) << i16K2GainSh] + A2}
*
*   f16SinEstim = sin(f16Theta)
*   f16CosEstim = cos(f16Theta)             
*
****************************************************************************/
frac16_t AMCLIB_AngleTrackObsrv_F16_FC(const GMCLIB_2COOR_SINCOS_T_F16 *psSinCos,
                                       AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl)
{
    register frac32_t f32Temp;
    register frac32_t f32Temp2; 
    register frac32_t f32Theta;
    register frac32_t f32A2Speed, f32Speed; 
    register frac32_t f32Out;
    register int32_t  i32SatMin, i32SatMax, i16K1GainSh, i16K2GainSh, i16A2GainSh;
    
    /* Storing quantities to local constants */  
    f32Speed  = psCtrl -> f32Speed;    
    i16K1GainSh = psCtrl -> i16K1GainSh;
    i16K2GainSh = psCtrl -> i16K2GainSh;
    i16A2GainSh = psCtrl -> i16A2GainSh;
    
    /* Error calculation: 
    error = sin (inAngle-EstimAngle) = f16Sin * f16CosEstim - f16Cos * f16SinEstim */  
    f32Temp =(frac32_t)((int32_t)psSinCos -> f16Sin*(int32_t)psCtrl -> f16CosEstim)<<1; 
    f32Temp2=(frac32_t)((int32_t)psSinCos -> f16Cos*(int32_t)psCtrl -> f16SinEstim)<<1;
    /* Saturation subtraction */
    f32Out =  (int32_t)(MLIB_Sub_F32_Ci(f32Temp, f32Temp2));              /* Subtraction without saturation to find saturation max. and min. */
    i32SatMax = (~(int32_t)(f32Temp) &   (int32_t)(f32Temp2)) & f32Out;   /* Saturation max. from minuend, subtrahend and subtraction without sat. */
    i32SatMin =   (int32_t)(f32Temp) & (~(int32_t)(f32Temp2)) & (~f32Out);/* Saturation min. from minuend, subtrahend and subtraction without sat. */
    f32Out = (i32SatMin < 0) ? INT32_MIN : f32Out;                        /* Low limit restriction */
    f32Out = (i32SatMax < 0) ? INT32_MAX : f32Out;                        /* Hight limit restriction */
    
    /* Speed error integration to obtain speed */
    f32Temp2 = (frac32_t)(frac64_t)(((frac64_t)f32Out * (frac64_t)psCtrl -> f16K1Gain) >> 15);
    f32Temp2 = (f32Temp2 == INT32_MIN) ? INT32_MAX : f32Temp2;    
    /* Bi-direction shift with saturation */
    if  (i16K1GainSh<(int16_t)0)                      /* negative shift = shift to left */
    {   f32Out = (frac32_t)(f32Temp2 >> -i16K1GainSh);
    }    
    else                                                 /* positive shift = saturation right shift */
    {
        f32Out = (frac32_t)(f32Temp2 << i16K1GainSh);
        f32Temp = (frac32_t)((frac32_t)INT32_MAX >> i16K1GainSh);
        if(f32Temp2>f32Temp)                             /* test high overflow */
        {   f32Out = (frac32_t)INT32_MAX;                /* high limit for left shift */
        }
        else
        {
            f32Temp = (frac32_t)((frac32_t)INT32_MIN >> i16K1GainSh);
            if(f32Temp2 <= f32Temp)                      /* test low overflow */
            {   f32Out = (frac32_t)INT32_MIN;            /* low limit for left shift */
            }
        }
    }
    /* Addition with saturation of speed */ 
    f32Temp   =   f32Out + f32Speed;
    i32SatMax = ~(f32Out | f32Speed) & f32Temp;
    i32SatMin =  (f32Out & f32Speed) & ~f32Temp;
    f32Temp = (i32SatMin < 0) ? INT32_MIN : f32Temp;
    f32Temp = (i32SatMax < 0) ? INT32_MAX : f32Temp;
    f32Speed = f32Temp;
    
    /* Integration part of controller calculation to obtain theta */
    f32Temp2 = (frac32_t)(frac64_t)((((frac64_t)f32Speed) * (frac64_t)psCtrl -> f16A2Gain) >> 15);    
    f32Temp2 = (f32Temp2 == INT32_MIN) ? INT32_MAX : f32Temp2;
    /* Bi-direction shift with saturation */
    if  (i16A2GainSh<(int16_t)0)                      /* negative shift = shift to left */
    {   f32Out = (frac32_t)(f32Temp2 >> -i16A2GainSh);
    }    
    else                                                 /* positive shift = saturation right shift */
    {
        f32Out = (frac32_t)(f32Temp2 << i16A2GainSh);
        f32Temp = (frac32_t)((frac32_t)INT32_MAX >> i16A2GainSh);
        if(f32Temp2>f32Temp)                             /* test high overflow */
        {   f32Out = (frac32_t)INT32_MAX;                /* high limit for left shift */
        }
        else
        {
            f32Temp = (frac32_t)((frac32_t)INT32_MIN >> i16A2GainSh);
            if(f32Temp2 <= f32Temp)                      /* test low overflow */
            {   f32Out = (frac32_t)INT32_MIN;            /* low limit for left shift */
            }
        }
    }
    f32A2Speed = psCtrl -> f32A2 + f32Out;               /* Speed A2 result with overflow*/
    
    /* Integration part of controller calculation to obtain theta */    
    f32Temp2 = (frac32_t)(frac64_t)(((frac64_t)f32Speed * (frac64_t)psCtrl -> f16K2Gain) >> 15); 
    f32Temp2 = (f32Temp2 == INT32_MIN) ? INT32_MAX : f32Temp2;        
    /* Bi-direction shift with saturation */
    if  (i16K2GainSh<(int16_t)0)                      /* negative shift = shift to left */
    {   f32Out = (frac32_t)(f32Temp2 >> -i16K2GainSh);
    }    
    else                                                 /* positive shift = saturation right shift */
    {
        f32Out = (frac32_t)(f32Temp2 << i16K2GainSh);
        f32Temp = (frac32_t)((frac32_t)INT32_MAX >> i16K2GainSh);
        if(f32Temp2>f32Temp)                             /* test high overflow */
        {   f32Out = (frac32_t)INT32_MAX;                /* high limit for left shift */
        }
        else
        {
            f32Temp = (frac32_t)((frac32_t)INT32_MIN >> i16K2GainSh);
            if(f32Temp2 <= f32Temp)                      /* test low overflow */
            {   f32Out = (frac32_t)INT32_MIN;            /* low limit for left shift */
            }
        }
    }
    f32Temp = f32A2Speed + f32Out;                      /* Speed A2 addition with overflow*/
    
    /* Theta calculation */
    f32Theta = (frac16_t)(f32Temp >> 16); 
    
    /* Store local quantities to structure */
    psCtrl -> f32Speed = f32Speed;
    psCtrl -> f32A2    = f32A2Speed;      
    psCtrl -> f16Theta = f32Theta;     
    psCtrl -> f16SinEstim = GFLIB_Sin_F16(f32Theta); /* Sine estim values calculation */ 
    psCtrl -> f16CosEstim = GFLIB_Cos_F16(f32Theta); /* Cosine estim values calculation */ 
    
    return ((frac16_t)(f32Theta));
}

/***************************************************************************//*!
*
* @brief  Initialization for the angle tracking observer
*
* @param  ptr    AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl
*                    frac32_t f32Speed    - Estimated speed <-1;1)
*                    frac32_t f32A2       - Internal integrator <-1;1)
*                    frac16_t f16Theta    - Estimated angle <-1;1) corresponds to <-pi;pi)
*                    frac16_t f16SinEstim - Sine of the estimated angle <-1;1)
*                    frac16_t f16CosEstim - Cosine of the estimated angle <-1;1)
*                    frac16_t f16K1Gain   - Constant to get speed from error <0;1)
*                    int16_t i16K1GainSh  - Shift for f16K1Gain <-15;15>
*                    frac16_t f16K2Gain   - Prop. constant to get angle from speed <0;1)
*                    int16_t i16K2GainSh  - Shift for f16K2Gain <-15;15>
*                    frac16_t f16A2Gain   - Integ. constant to get angle from speed <0;1)
*                    int16_t i16A2GainSh  - Shift for f16A2Gain <-15;15>
*                
* @param  in     frac16_t f16ThetaInit - initial value of the angle
*
* @return none
*
* @remarks  The function initializes angle tracking observer by following:
*
*   f32Speed = 0
*   A2 = f16ThetaInit << 16
*   f16Theta = f16ThetaInit
*   f16SinEstim = sin(f16ThetaInit)
*   f16CosEstim = cos(f16ThetaInit)
*
****************************************************************************/
void AMCLIB_AngleTrackObsrvInit_F16_FC(frac16_t f16ThetaInit, 
                                       AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl)
{
    psCtrl->f32Speed = (frac32_t)0 ;
    psCtrl->f32A2 = f16ThetaInit << 16; 
    psCtrl->f16Theta = f16ThetaInit;   
    psCtrl->f16SinEstim = GFLIB_Sin_F16(f16ThetaInit);
    psCtrl->f16CosEstim = GFLIB_Cos_F16(f16ThetaInit);  
}
