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
* @brief  Flex s-ramp functions with 16-bit fractional output 
* 
*******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_FlexSRamp_A32.h"

/***************************************************************************//*!
*
* @brief  Flex s-ramp calculation
*
* @param  ptr  GFLIB_FLEXSRAMP_T_F32 *psParam
*           - f32State: State variable keeping the last state x
*           - f32Incr: derivative of x (dX) - acceleration
*           - f32AIncr: derivative of a (dA) - der. of acceleration
*           - f32ADes: desired acceleration - must be <= f32Incrmax
*           - f32Target: Target value x
*           - f32Ts: Sample time [s]
*           - f32IncrMax: Maximum increment of x
*           - f32XT1: X(T1) - x at the instant where the ramp got the desired 
*                     acceleration (f32Incr = const)
*           - f32XT2: X(T2) - x at the instant where the ramp started to decelerate
*           - f16DA: Defined derivative of acceleration: dA = A / T
*           - u16AccState: State of the ramp process 0: f32Incr rising, 
*                          1:f32Incr = const, 2: f32Incr falling
*           - bReachFlag: Flag is set to TRUE if the desired value is achieved*
* @return Ramp output - frac16_t value <-1;1)
*       
* @remarks  The function calculations are separated to three intervals:
*           const. jerk interval:jerk is constant, acceleration is proportional to time, 
*                                speed increases by quadratic, position changes with power of 3 
*           const. acceleration: jerk is constant, acceleration is proportional to time, 
*                                speed increases by quadratic, position rise up with power of 3 
*           const. jerk(inverse):jerk is constant, acceleration is proportional to time, 
*                                speed decreased by quadratic, position fall down with power of 3 
*
****************************************************************************/
frac16_t GFLIB_FlexSRamp_F16_FC(GFLIB_FLEXSRAMP_T_F32 *psParam)
{
    register frac32_t f32State;
    register frac32_t f32StateNew;
    register frac32_t f32Incr;    
    register frac32_t f32Target; 
    register bool_t bTargetHigherState;
    register bool_t bStateHigherTarget;  
    
    /* Store more times red quantities to registers */  
    f32State = psParam -> f32State;
    f32Target= psParam -> f32Target;
    f32Incr  = psParam -> f32Incr;
    bTargetHigherState = f32Target > f32State;
    bStateHigherTarget = f32State > f32Target;

    /* Increment calculation */
    /* Current state > X(T2) - deacceleration part with constant inverse jerk*/
    if (psParam -> u16AccState >= 1)
    {   
        if (psParam -> u16AccState > 1)
        {
            /* Increment calculation - trapezoidal integration, i.e. the first value is the half */
            f32Incr = MLIB_SubSat_F32(f32Incr, psParam -> f32AIncr);          
            if (f32Incr < (psParam -> f32AIncr >> 1))
            {
                f32Incr = psParam -> f32AIncr >> 1;
            }
            /* State calculations */
            if (bTargetHigherState)
            {
                f32StateNew = MLIB_AddSat_F32(f32State, f32Incr);
            }
            else 
            {   
                if (bStateHigherTarget)
                {
                    f32StateNew = MLIB_SubSat_F32(f32State, f32Incr);
                } 
                else 
                {   
                    f32StateNew = f32State;
                }
            }
            /* State limitation if target achieved */
            if (bTargetHigherState && (f32StateNew > f32Target))
            {
                f32StateNew = f32Target;
            }
            else if (bStateHigherTarget && (f32StateNew < f32Target))
            {
                f32StateNew = f32Target;
            }     
        }    
        else  /* X(T1) < current state < X(T2) - const. accel. part with zero jerk*/
        {
            /* State calculations */
            if (bTargetHigherState)
            {
                f32StateNew = MLIB_AddSat_F32(f32State, f32Incr);
            }
            else if (bStateHigherTarget)
            {
                f32StateNew = MLIB_SubSat_F32(f32State, f32Incr);
            }
            else
            {   
                f32StateNew = f32State;  
            }
            /* Next state determination */
            if (bTargetHigherState && (f32StateNew > psParam -> f32XT2))
            {
                psParam -> u16AccState = 2;
            }            
            else if (bStateHigherTarget && (f32StateNew < psParam -> f32XT2))
            {
                psParam -> u16AccState = 2;
            }    
        }
    }
    else    /* Current state < X(T1) - acceleration part with constant jerk*/
    {
        /* Increment calculation - trapezoidal integration, i.e. the first value is the half */
        if (f32Incr > 0)
        {
            f32Incr = MLIB_AddSat_F32(f32Incr, psParam -> f32AIncr);
        }
        else
        {
            f32Incr = MLIB_AddSat_F32(f32Incr, psParam -> f32AIncr >> 1);
        }         
        
        /* If desired acceleration achieve, increment will be limited. */
        if (f32Incr > psParam -> f32ADes)
        {
            f32Incr = psParam -> f32ADes;
        }
        /* State (position) increasing or decreasing and next state determination */
        if (bTargetHigherState)
        {
            f32StateNew = MLIB_AddSat_F32(f32State, f32Incr);
            psParam-> u16AccState =(f32StateNew > psParam->f32XT1) + (f32StateNew > psParam->f32XT2);   
        }
        else 
        {   if (bStateHigherTarget)
            {
                f32StateNew = MLIB_SubSat_F32(f32State, f32Incr);
                psParam->u16AccState =(f32StateNew < psParam->f32XT1) + (f32StateNew < psParam->f32XT2);  
            }    
            else 
            {
                f32StateNew = f32State;        
            }
        }
    }    
          
    /* store quantities into structure */
    psParam -> f32Incr = f32Incr;
    psParam -> f32State = f32StateNew;
    psParam -> bReachFlag = (f32StateNew == psParam -> f32Target);
    
    /* return rounded state value */
    return (MLIB_Rnd_F16l(f32StateNew));      
}

/***************************************************************************//*!
*
* @brief  Increment calculation for flex s-ramp
*
* @param  in   frac16_t f16Target  - Desired value in <-1;1) in frac16_t
*              acc32_t a32Duration - Defined time [s] to go from the state to the 
*                                    target value in [0;65535.9] in acc32_t
*
* @param  ptr  GFLIB_FLEXSRAMP_T_F32 *psParam
*           - f32State: State variable keeping the last state x
*           - f32Incr: derivative of x (dX) - acceleration
*           - f32AIncr: derivative of a (dA) - der. of acceleration
*           - f32ADes: desired acceleration - must be <= f32Incrmax
*           - f32Target: Target value x
*           - f32Ts: Sample time [s]
*           - f32IncrMax: Maximum increment of x
*           - f32XT1: X(T1) - x at the instant where the ramp got the desired 
*                     acceleration (f32Incr = const)
*           - f32XT2: X(T2) - x at the instant where the ramp started to decelerate
*           - f16DA: Defined derivative of acceleration: dA = A / T
*           - u16AccState: State of the ramp process 0: f32Incr rising, 
*                          1: f32Incr = const, 2: f32Incr falling
*           - bReachFlag: Flag is set to TRUE if the desired value is achieved*
* @return bool_t - True if discriminant is positive or zero (S ramp can achieve 
*                  target value in desired time), false if not.
*       
* @remarks  The function calculates the acceleration increment based as follows:
* 
*           For the desired acceleration it solves the following quadratic equation:
*
*           f32ADes = {a32Duration * f16DA - sqrt[(a32Duration * f16DA) ^ 2  
*                     - 4 * (f16Target - f32State) * f16DA]} / 2 
*
*           if the discriminant is negative, i.e. 
*           a32Duration * f16DA) ^ 2 < 4 * (f16Target - f32State) * f16DA:
*           then f32ADes = sqrt([f16DA * (f16Target - f32State)]
*           and returns FALSE otherwise TRUE
*           
*           if f32ADes > f32IncrMax, then f32ADes = f32IncrMax 
*
*           Acceleration increment is then:
*           f32AIncr = f32Ts ^ 2 * f16DA
*           
*           The X(T1) until the f32Incr is incremented - as integration 1/2 * f32ADes ^ 2 * f16DA:      
*           f32XT1 = f32State +(-) (f32Ts ^ 2 * f16DA) / 2
*
*           The X(T2) while the f32Incr is const:       
*           f32XT2 = f16Target -(+) (f32Ts ^ 2 * f16DA) / 2
*
*           f32Target = f16Target
*           
*           result = (f16Target - f32State) * (f32Ts / a32Duration)
*           u16AccState = 0
*           The bReachFlag flag is cleared.
*
****************************************************************************/
bool_t GFLIB_FlexSRampCalcIncr_F16_C(frac16_t f16Target, acc32_t a32Duration, 
                                     GFLIB_FLEXSRAMP_T_F32 *psParam)
{
    register frac32_t f32Temp1;
    register frac32_t f32Temp2; 
    register frac32_t f32Temp3; /* f32ADes as well */   
    register frac32_t f32Dx;
    register frac16_t f16XT1;    
    register frac16_t f16D;
    register frac16_t f16DA; 
    register bool_t bStatus;

    /* store more time red quantities to register */
    f16DA = psParam -> f16DA; 
    
    /* Ramping range calculation */ 
    /* Shifted to the right once because the result can be more than 1 */
    f32Dx = MLIB_Sub_F32(MLIB_Conv_F32s(f16Target) >> 1, psParam -> f32State >> 1);
    
    /* Desired acceleration = {T * dA - Sqrt[(T * dA) ^ 2 - 4 * dX * dA]} / 2 */
    /* T * dA calculation */
    f32Temp1 = (a32Duration * f16DA)<<1;
    f16D = f32Temp1 >> 16;
    
    /* Discriminant calculation: Sqrt[(T * dA) ^ 2 - 4 * dX * dA] : */
    /* First portion of the discriminant: (T * dA) ^ 2 */
    f32Temp1 = MLIB_Mul_F32(f32Temp1, f32Temp1);
    /* Second portion of the discriminant: 4 * dX * dA : used three shifts left because
     * f16Dx was shifted to the right once before */
    f32Temp3 = MLIB_MulRnd_F32ls(MLIB_AbsSat_F32(f32Dx), psParam -> f16DA);
    f32Temp2 = MLIB_ShLSat_F32(f32Temp3, 3);
    
    /* Desired acceleration calculation - ADes*/
    /* if the discriminant is >= 0, the ramp is achievable in time (dA = 0) */ 
    if (f32Temp1 >= f32Temp2)
    {
        f32Temp3 = (f16D - GFLIB_Sqrt_F16l(f32Temp1 - f32Temp2)) << 15; /* shifts:15=16(to f32)-1(div by 2)*/
        bStatus = TRUE;
    }
    /* if the discriminant is < 0, the ramp is not achievable in time */
    else
    {
        f32Temp3 = GFLIB_Sqrt_F16l(f32Temp3 << 1) << 16;
        bStatus = FALSE;
    }

    /* read Ts from structure */
    f32Temp1 = psParam -> f32Ts;
    /* Limitation of ADes = DA * Ts^2 to f32IncrMax */
    f32Temp2 = MLIB_MulRnd_F32ls(MLIB_MulRnd_F32(f32Temp1, f32Temp1), f16DA); 
    
    /* if the desired acceleration > maximum allowed increment, 
     * then it is set to the max. increment */
    if (f32Temp2 > psParam ->f32IncrMax)
    {
        f32Temp3 = psParam -> f32IncrMax;
        bStatus = FALSE;
    }
    
    /* Integration of the acceleration to the time where A(T1) = ADes:
     * A(T1) = dA * T1 => T1 = A(T1) / dA = ADes / dA
     * x(T1) = 1/2 * dA * T1 ^ 2 = 1/2 * dA * [ADes / dA] ^ 2 = 1/2 * ADes ^ 2 / dA */    
    f16XT1 = (MLIB_Div1QSat_F16ls(MLIB_MulRnd_F32(f32Temp3, f32Temp3), f16DA)) >> 1;

    /* Calculation instants among constant or zero jerk intervals */
    if (f32Dx < 0)   /* If slope is negative (target < state) cannot overflow */
    {
        psParam -> f32XT1 = MLIB_Sub_F32(psParam -> f32State, MLIB_Conv_F32s(f16XT1));
        psParam -> f32XT2 = MLIB_Conv_F32s(MLIB_Add_F16(f16Target, f16XT1));
    }
    else             /* If slope is positive (target >= state) cannot overflow */
    {
        psParam -> f32XT1 = MLIB_Add_F32(psParam -> f32State, MLIB_Conv_F32s(f16XT1));
        psParam -> f32XT2 = MLIB_Conv_F32s(MLIB_Sub_F16(f16Target, f16XT1));
    }

    /* store quantities into structure */ 
    psParam -> f32AIncr = f32Temp2;  /* ADes = DA * Ts^2 */
    psParam -> f32ADes  = f32Temp3;  /* limited ADes = DA * Ts^2 */  
    psParam -> f32Target = MLIB_Conv_F32s(f16Target);  /* set target value */
    psParam -> f32Incr   = 0;        /* clear S ramp increment */
    psParam -> u16AccState = 0;      /* set firs S ramp interval */
    psParam -> bReachFlag = FALSE;   /* clear bReachFlag */
    /* return bool value of discriminant polarity */
    return (bStatus);
}        
