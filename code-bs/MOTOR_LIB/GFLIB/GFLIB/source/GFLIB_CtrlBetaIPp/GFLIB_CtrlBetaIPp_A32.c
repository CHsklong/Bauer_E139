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
* @brief  Parallel beta-IP controller with and without anti-wind-up protection
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_CtrlBetaIPp_A32.h"

/***************************************************************************//*!
*
* @brief  16-bit input and output parallel form of beta-IP controller with
*         anti-wind-up protection (restricting overshot from required value change)
*
* @param  ptr  GFLIB_CTRL_BETA_IP_P_AW_T_A32 *psParam
*              - pointer to the structure of the beta-IP controller
*
*                  GFLIB_CTRL_BETA_IP_P_AW_T_A32 data structure:
*                    acc32_t a32PGain - proportional gain; input parameter
*                          in the following range: 0 <= f16PGain < 65536.0  
*                    acc32_t a32IGain - integral gain; input parameter
*                          in the following range: 0 <= f16IGain < 65536.0 
*                    frac32_t f32IAccK_1 - state variable; fractional part 
*                          (lower long) of integral part at step k-1;
*                          can be modified outside of the function; input/output 
*                          parameter in the following range -1.0 <= f32IAccK_1 < 1.0  
*                    frac16_t f16InErrK_1 - error at the step k-1; input/output 
*                          parameter in the following range: -1.0 <= f16InErrK_1 < 1.0   
*                    frac16_t f16UpperLim - upper limit of the controller output;
*                          input parameter in the following range: 
*                          1.0 <= f16UpperLim < 1.0 and f16UpperLim > f16LowerLim
*                    frac16_t f16LowerLim - lower limit of the controller output;
*                          input parameter in the following range:    
*                          1.0 <= f16LowerLim < 1.0 and f16UpperLim > f16LowerLim
*                    frac16_t f16BetaGain - beta gain; input parameter
*                          in the following range: 0 <= f16BetaGain < 1.0 
*                    bool_t bLimFlag - limitation flag; if set, the controller
*                          output reached either f16UpperLim or f16LowerLim,
*                          otherwise it is cleared output parameter 
*
*                const bool_t *pbStopIntegFlag
*                  - Pointer to a bool_t variable variable; if the pointed variable is false,
*                    then the integral part is limited only by the beta-IP controller limits.
*                    If the pointed variable is true, then the integral part is frozen
*                    immediately. The f16InErrK_1 keeps updated although the pointed flag is true. 
*                    
* @param  in     frac16_t f16InReq - Input required value of the controller at step K processed
*                    by P, I and Beta terms of the beta-IP algorithm fractional value in the range <-1;1)
*                frac16_t f16In - Input feedback value of the controller at step K processed
*                    by P, I and Beta terms of the beta-IP algorithm fractional value in the range <-1;1)
*
* @return This function returns: f16LowerLim <= result <= f16UpperLim - frac16_t value <-1;1)

* @remarks      beta-IP controller algorithm:                                                                            
*       w(t) - input required value in continuous time domain   
*       y(t) - input feedback (measured) value in continuous time domain
*       u(t) - controller output in continuous time domain              
*                                                                       
*       e(k) - input required value at step k - discrete time domain 
*       e(k) - input feedback (measured) value at step k - discrete time domain 
*       u(k) - controller output at step k - discrete time domain                                                 
*                                                                       
*       The beta-IP controller algorithm in continuous time domain:          
*                                     /t                                       
*       u(t) = Kp(B*w(t)-y(t)) + Ki * | (w(t)-y(t))*dt                          
*                                     /0     
*       The beta-IP controller algorithm in discrete time domain:  
*       u(k) = Kpf*(B*w(k)-y(k)) + ui(k-1) + Kif*(e(k)+e(k-1))/2   
*                                        
*       Kp - controller gain                                            
*       Ki - integral gain 
*       B  - beta gain to restrict overshot from required value change (if B = 1 ... classical PI controller)
*                                                                        
*******************************************************************************/ 
frac16_t GFLIB_CtrlBetaIPpAW_F16_FC(frac16_t f16InReq, frac16_t f16In,
                                    const bool_t *pbStopIntegFlag, 
                                    GFLIB_CTRL_BETA_IP_P_AW_T_A32 *psParam)
{
    register acc64_t  a64Proportional;
    register acc64_t  a64Integral;
    register acc64_t  a64Temp;
    register acc64_t  a64Out;    
    register frac32_t f32UpperLimit;
    register frac32_t f32LowerLimit;    
    register frac32_t f32InSum;
    register frac16_t f16InErr;  
	
    /* from structure to register, each calculation is shifted right and on the end shifted left (output) */
    f32UpperLimit = (frac32_t) psParam->f16UpperLim << 15;         /* Upper limit in 32-bit */
    f32LowerLimit = (frac32_t) psParam->f16LowerLim << 15;         /* Lower limit in 32-bit */		
	
    /* Proportional part calculation, no shift left */	
    f32InSum = MLIB_Mul_F32ss(f16InReq, psParam->f16BetaGain);     /* error calculation with Beta component */ 
    f32InSum = MLIB_SubSat_F32(f32InSum, (frac32_t)(int16_t)(uint16_t)f16In << 16);           /* error calculation with Beta component */ 
	
    a64Proportional = (acc64_t)((int64_t)f32InSum * (int64_t) psParam->a32PGain)>>16;
     
    f16InErr = MLIB_SubSat_F16(f16InReq, f16In);                   /* error calculation without Beta component */
                                                                   
    /* Stop integration if flag is set */                          
    if (*pbStopIntegFlag)                                          /* if stop flag is set */
    {   f16InErr  = 0;                                             /* integration will be stopped */
    }  
    
    /* Integral part calculation  */
    f32InSum = (frac32_t)(int16_t)(uint16_t)f16InErr + (frac32_t)(int16_t)(uint16_t)psParam->f16InErrK_1;  /* current + previous input */                
    a64Temp = (acc64_t)(((int64_t) f32InSum * (int64_t) psParam->a32IGain)>>1); 
    a64Integral = a64Temp + ((acc64_t)psParam->f32IAccK_1 >> 1);   /* integral part    */                 
    
    /* Integral part limitation */
    if (a64Integral >= (acc64_t)f32UpperLimit)                     /* limitation of integral part      */
    {   a64Integral  = (acc64_t)f32UpperLimit;
    }
    else
    {   if(a64Integral <= (acc64_t)f32LowerLimit)
        {   a64Integral = (acc64_t)f32LowerLimit;
        }
    }  
    a64Out = a64Proportional + a64Integral;           /* Controller output calculation by adding the P and I terms */

    /* Output limitation */
    psParam->bLimFlag = FALSE;                        /* Clear flag and set later in case of limitation */                      
    if (a64Out >= (acc64_t)f32UpperLimit)
    {   a64Out  = (acc64_t)f32UpperLimit;              
        psParam->bLimFlag = TRUE;                     /* Storing of output limitation flag */
    }                                                 
    else                                              
    {   if(a64Out <= (acc64_t)f32LowerLimit)          
        {   a64Out = (acc64_t)f32LowerLimit;          
            psParam->bLimFlag = TRUE;                 /* Storing of output limitation flag */
        }                                             
    }   
    
    /* Storing actual values into structure */
    psParam->f32IAccK_1  = (frac32_t)a64Integral << 1;/* Storing state value of the integral */    
    psParam->f16InErrK_1 = f16InErr;                  /* Storing controller input error */
   
    /* return controller output */
    return((frac16_t)(a64Out>>15)); 
}
