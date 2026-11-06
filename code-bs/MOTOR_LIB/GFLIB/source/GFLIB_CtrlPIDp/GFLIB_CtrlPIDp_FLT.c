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
* @brief  Float parallel PID controller with and without anti-wind-up protection
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_CtrlPIDp_FLT.h"

/***************************************************************************//*!
*
* @brief float input and output parallel form of PID controller with
*         anti-wind-up protection
*
* @param  ptr   GFLIB_CTRL_PID_P_AW_T_FLT *psParam
*               - pointer to the structure of the PID controller
*               
*                 GFLIB_CTRL_PID_P_AW_T_FLT data structure:
*                   float_t fltPGain - proportional gain, in non-negative float value 
*                   float_t fltIGain - integral gain, in non-negative float value   
*                   float_t fltDGain - derivative gain, in non-negative float value   
*                   float_t fltIAccK_1- state variable; fractional part (lower long)
*                         of integral part at step k-1; can be modified outside of 
*                         the function; input/output parameter in the following
*                          float range: -1.0 <= fltIAccK_1 < 1.0                               
*                   float_t fltInErrK_1 - error at the step k-1, in float range;  
*                   float_t fltUpperLim - upper limit of the controller output;
*                         input parameter, in float range: fltUpperLim > fltLowerLim
*                   float_t fltLowerLim - lower limit of the controller output;
*                         input parameter, in float range: fltUpperLim > fltLowerLim
*                   float_t fltInErrDK_1 - error for derivative calculation at
*                         the step k-1, in float range; 
*                   bool_t bLimFlag - limitation flag; if set, the controller
*                         output reached either fltUpperLim or fltLowerLim,
*                         otherwise it is cleared output parameter 
*               
*               const bool_t *pbStopIntegFlag
*                 - Pointer to a bool_t variable variable; if the pointed variable
*                   is false, then the integral part is limited only by
*                   the PID controller limits. If the pointed variable is true, then
*                   the integral part is frozen immediately. The fltInErrK_1 keeps 
*                   updated although the pointed flag is true. 
*                   
* @param  in    float_t fltInErr - Input error of the controller at step K processed
*                   by P, I and D terms of the PID algorithm, in float range                                              
*
* @return This function returns: fltLowerLim <= result <= fltUpperLim - float_t value
*
* @remarks      PID controller algorithm:                                                                            
*       e(t) - input error in continuous time domain  
*       u(t) - controller output in continuous time domain              
*                                                                       
*       e(k) - input error at step k - discrete time domain   
*       u(k) - controller output at step k - discrete time domain                                                 
*                                                                       
*       The PI controller algorithm in continuous time domain:          
*                             /t                                       
*       u(t) = Kp*e(t) + Ki * | e(t)*dt + Kd*de(t)/dt                            
*                             /0     
*       The PID controller algorithm in discrete time domain:  
*       u(k) = Kpf*e(k) + ui(k-1) + Kif*(e(k)+e(k-1))/2 + Kdf(e_d(k)-e_d(k-1)) 
*                                        
*       Kp - proportional gain
*       Ki - integral gain
*       Kd - derivative gain                                    
*                                                                       
*******************************************************************************/ 
float_t GFLIB_CtrlPIDpAW_FLT_FC(float_t fltInErr,
                                float_t fltInErrD,
                                const bool_t *pbStopIntegFlag,
                                GFLIB_CTRL_PID_P_AW_T_FLT *psParam)
{
    register float_t fltProportional;
    register float_t fltIntegral;
    register float_t fltOut;
    register float_t fltUpperLimit;
    register float_t fltLowerLimit;
    register float_t fltInSum;

    register uint32_t u32LimFlag;
        
    u32LimFlag = FALSE;              /* Set output limitation flag to 0, will be replaced by 1 in case of limitation*/      
    fltUpperLimit = psParam->fltUpperLim;                          /* Calculation of limits in 32-bit */
    fltLowerLimit = psParam->fltLowerLim;    
    
    /* Proportional part calculation */
    fltProportional = MLIB_Mul_FLT(fltInErr, psParam->fltPGain);   /* error * PGain to Proportional part */
    
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
