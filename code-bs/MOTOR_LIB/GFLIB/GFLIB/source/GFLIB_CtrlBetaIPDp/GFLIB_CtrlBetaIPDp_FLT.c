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
* @brief  Float parallel beta-IPD controller with and without anti-wind-up protection
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_CtrlBetaIPDp_FLT.h"

/***************************************************************************//*!
*
* @brief float input and output parallel form of Beta-IPD controller with
*        anti-wind-up protection (restricting overshot from required value change)
*
* @param  ptr   GFLIB_CTRL_BETA_IPD_P_AW_T_FLT *psParam
*               - pointer to the structure of the beta-IPD  controller
*               
*                 GFLIB_CTRL_BETA_IPD_P_AW_T_FLT data structure:
*                   float_t fltPGain - proportional gain, in non-negative float value 
*                   float_t fltIGain - integral gain, in non-negative float value   
*                   float_t fltDGain - derivative gain, in non-negative float value   
*                   float_t fltIAccK_1- state variable; fractional part (lower long)
*                         of integral part at step k-1; can be modified outside of 
*                         the function; input/output parameter in the following
*                          float range: -1.0 <= fltIAccK_1 < 1.0                               
*                   float_t fltInErrK_1 - error at the step k-1, in float range;  
*                   float_t fltUpperLim - upper limit of the controller output;
*                           input parameter, in float range: fltUpperLim > fltLowerLim
*                   float_t fltLowerLim - lower limit of the controller output;
*                           input parameter, in float range: fltUpperLim > fltLowerLim
*                   float_t fltInErrDK_1 - error for derivative calculation at
*                           the step k-1, in float range; 
*                   float_t fltBetaGain - beta gain, in non-negative float value 
*                   bool_t bLimFlag - limitation flag; if set, the controller
*                           output reached either fltUpperLim or fltLowerLim,
*                           otherwise it is cleared output parameter 
*               
*               const bool_t *pbStopIntegFlag
*                   - Pointer to a bool_t variable variable; if the pointed variable
*                     is false, then the integral part is limited only by
*                     the beta-IPD controller limits. If the pointed variable is true, then
*                     the integral part is frozen immediately. The fltInErrK_1 keeps 
*                     updated although the pointed flag is true. 
*                   
* @param  in     frac16_t fltInReq - Input required value of the controller at step K processed
*                         by P, I, D and Beta terms of the beta-IPD algorithm fractional value in float range
*                frac16_t fltIn - Input feedback value of the controller at step K processed
*                         by P, I and D and Beta terms of the beta-IPD algorithm fractional value in float range
*                                                  
* @return This function returns: fltLowerLim <= result <= fltUpperLim - float_t value
*
* @remarks      beta-IPD controller algorithm:                                                                            
*       w(t) - input required value in continuous time domain   
*       y(t) - input feedback (measured) value in continuous time domain
*       u(t) - controller output in continuous time domain              
*                                                                       
*       e(k) - input required value at step k - discrete time domain 
*       e(k) - input feedback (measured) value at step k - discrete time domain 
*       u(k) - controller output at step k - discrete time domain                                                 
*                                                                       
*       The beta-IPD controller algorithm in continuous time domain:          
*                                     /t                                       
*       u(t) = Kp(B*w(t)-y(t)) + Ki * | (w(t)-y(t))*dt + Kd*d((w(t)-y(t)))/dt                           
*                                     /0     
*       The beta-IPD controller algorithm in discrete time domain:  
*       u(k) = Kpf*(B*w(k)-y(k)) + ui(k-1) + Kif*(e(k)+e(k-1))/2 + Kdf((w(k)-y(k))-e_d(k-1))  
*                                        
*       Kp - controller gain                                            
*       Ki - integral gain 
*       Kd - derivative gain
*       B  - beta gain to restrict overshot from required value change (if B = 1 ... clasical PID controller)
*                                                                     
*******************************************************************************/ 
float_t GFLIB_CtrlBetaIPDpAW_FLT_FC(float_t fltInReq, float_t fltIn, float_t fltInErrD,
                                    const bool_t *pbStopIntegFlag, 
                                    GFLIB_CTRL_BETA_IPD_P_AW_T_FLT *psParam)
{
    register float_t fltProportional;
    register float_t fltIntegral;
    register float_t fltOut;
    register float_t fltUpperLimit;
    register float_t fltLowerLimit;
    register float_t fltInSum;
    register float_t fltInErr;
    register uint32_t u32LimFlag;
        
    u32LimFlag = FALSE;                 /* Set output limitation flag to 0, will be replaced by 1 in case of limitation*/      
    fltUpperLimit = psParam->fltUpperLim;                          /* Calculation of limits in 32-bit */
    fltLowerLimit = psParam->fltLowerLim;    
    
    /* Proportional part calculation */
    fltInErr = MLIB_Sub_FLT(MLIB_Mul_FLT(fltInReq, psParam->fltBetaGain), fltIn); /* error calculation with Beta component */
    fltProportional = MLIB_Mul_FLT(fltInErr, psParam->fltPGain);   /* error * PGain to Proportional part */
    
    fltInErr = MLIB_Sub_FLT(fltInReq, fltIn);                      /* error calculation without Beta component */

    /* Stop integration if flag is set */     
    if ( *pbStopIntegFlag )                                        /* if stop flag is set */
    {   fltInErr = 0.0F;                                           /* integration will be stooped     */
    }
    
    /* Integral part calculation  */
    fltInSum = MLIB_Mul_FLT(MLIB_Add_FLT (fltInErr, psParam->fltInErrK_1), 0.5F);  /* (input + previous input)/2 */   
    fltIntegral = MLIB_Mac_FLT (psParam->fltIAccK_1, fltInSum, psParam->fltIGain); /* IAcc + previous sum * IGain */
    
    /* Integral part limitation */
    if (fltIntegral >= fltUpperLimit)                              /* integral part high limitation  */
    {   fltIntegral = fltUpperLimit;
    }
    else
    {   if(fltIntegral <= fltLowerLimit)                           /* integral part low limitation  */
        {   fltIntegral = fltLowerLimit;
        }
    }  
    fltOut = MLIB_Add_FLT (fltProportional, fltIntegral);          /* P + I summation */      
    fltInSum = MLIB_Sub_FLT (fltInErrD, psParam->fltInErrDK_1);   /* (fltInSum = D input - previous D input) */   
    fltOut = MLIB_Mac_FLT (fltOut, fltInSum, psParam->fltDGain);  /* P + I summation + fltInSum * DGain */                                   
    
    /* Output limitation */
    if (fltOut >= fltUpperLimit)                                   /* Controller output high limitation (both prop + integ parts) */
    {   fltOut  = fltUpperLimit;                                  
        u32LimFlag = TRUE;                                         /* Storing of output limitation flag as true*/
    }                                                                     
    else                                                                  
    {   if(fltOut <= fltLowerLimit)                                /* Controller output low limitation  */       
        {   fltOut = fltLowerLimit;                                       
            u32LimFlag = TRUE;                                     /* Storing of output limitation flag */
        }
    }
    /* Storing quantities to controller structure for next iteration */
    psParam->fltIAccK_1 = fltIntegral;                             /* Storing state value of the integral */   
    psParam->fltInErrK_1= fltInErr;                                /* Updating controller input state variables */
    psParam->fltInErrDK_1= fltInErrD;                              /* Updating controller input derivative state variables */
    psParam->bLimFlag = (bool_t) u32LimFlag;  
   
    return(fltOut);
}       
