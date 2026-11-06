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
* @brief  Float flex s-ramp functions 
* 
*******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_FlexSRamp_FLT.h"

/***************************************************************************//*!
*
* @brief  Float flex s-ramp calculation
*
* @param ptr GFLIB_FLEXSRAMP_T_FLT *psParam
*            - fltState: State variable keeping the last state x  
*            - fltIncr: derivative of x (dX) - acceleration 
*            - fltAIncr: derivative of a (dA) - der. of acceleration 
*            - fltADes: desired acceleration - must be <= fltIncrmax 
*            - fltTarget: Target value x 
*            - fltTs: Sample time [s] 
*            - fltIncrMax: Maximum increment of x  
*            - fltXT1: X(T1) - x at the instant where the ramp got the desired  
*               acceleration (fltIncr = const)  
*            - fltXT2: X(T2) - x at the instant where the ramp started to decelerate  
*            - fltDA: Defined derivative of acceleration: dA = A / T  
*            - u16AccState: State of the ramp process 0: fltIncr rising,  
*                           1: fltIncr = const, 2: fltIncr falling  
*            - bReachFlag: Flag is set to TRUE if the desired value is achieved  
*
* @return Ramp output - float_t value 
*       
* @remarks  The function calculations are separated to three intervals:
*           const. jerk interval:jerk is constant, acceleration is proportional to time, 
*                                speed increases by quadratic, position changes with power of 3 
*           const. acceleration: jerk is constant, acceleration is proportional to time, 
*                                speed increases by quadratic, position rise up with power of 3 
*           const. jerk(inverse):jerk is constant, acceleration is proportional to time, 
*                                speed decreases by quadratic, position fall down with power of 3 
*
****************************************************************************/
float_t GFLIB_FlexSRamp_FLT_FC(GFLIB_FLEXSRAMP_T_FLT *psParam)
{
    register float_t fltState;
    register float_t fltStateNew;
    register float_t fltIncr;    
    register float_t fltTarget;   
    register float_t fltTemp;
    register bool_t bTargetHigherState;
    register bool_t bStateHigherTarget;
    
    /* Store more times red quantities to registers */ 
    fltState = psParam -> fltState;
    fltTarget= psParam -> fltTarget;
    fltIncr  = psParam -> fltIncr; 
    bTargetHigherState = fltTarget > fltState;
    bStateHigherTarget = fltState > fltTarget;
        
    /* Increment calculation */
    /* Current state > X(T2) - deacceleration part with constant inverse jerk*/
    if (psParam -> u16AccState >= 1)
    {   
        if (psParam -> u16AccState > 1)
        {   
            /* Increment calculation - trapezoidal integration, i.e. the first value is the half */
            fltIncr = MLIB_Sub_FLT(fltIncr, psParam -> fltAIncr);
            fltTemp = MLIB_Mul_FLT(psParam -> fltAIncr, 0.5F);
            if (fltIncr < fltTemp)
            {
                fltIncr = fltTemp;
            }
            /* State calculations */
            if (bTargetHigherState)
            {
                fltStateNew = MLIB_Add_FLT(fltState, fltIncr);
            }
            else 
            {   
                if (bStateHigherTarget)
                {
                    fltStateNew = MLIB_Sub_FLT(fltState, fltIncr);
                } 
                else 
                {   
                    fltStateNew = fltState;
                }
            }    
            /* State limitation if target achieved */
            if (bTargetHigherState && (fltStateNew > fltTarget))
            {
                fltStateNew = fltTarget;
            }
            else if (bStateHigherTarget && (fltStateNew < fltTarget))
            {
                fltStateNew = fltTarget;
            }  
        }                
        else  /* X(T1) < current state < X(T2) - const. accel. part with zero jerk*/
        {
            /* State calculations */
            if (bTargetHigherState)
            {
                fltStateNew = MLIB_Add_FLT(fltState, fltIncr);
            }
            else if (bStateHigherTarget)
            {
                fltStateNew = MLIB_Sub_FLT(fltState, fltIncr);
            }
            else
            {   
                fltStateNew = fltState;  
            }
            /* Next state determination */
            if (bTargetHigherState && (fltStateNew > psParam -> fltXT2))
            {
                psParam -> u16AccState = 2;
            }            
            else if (bStateHigherTarget && (fltStateNew < psParam -> fltXT2))
            {
                psParam -> u16AccState = 2;
            }    
        }    
    }
    else    /* Current state < X(T1) - acceleration part with constant jerk*/
    {
        /* Increment calculation - trapezoidal integration, i.e. the first value is the half */
        if (fltIncr > 0)
        {
            fltIncr = MLIB_Add_FLT(fltIncr, psParam -> fltAIncr);
        }
        else
        {
            fltIncr = MLIB_Add_FLT(fltIncr, MLIB_Mul_FLT(psParam -> fltAIncr, 0.5F));
        }         
        
        /* If desired acceleration achieve, increment will be limited. */
        if (fltIncr > psParam -> fltADes)
        {
            fltIncr = psParam -> fltADes;
        }
        /* State (position) increasing or decreasing and next state determination */
        if (bTargetHigherState)
        {
            fltStateNew = MLIB_Add_FLT(fltState, fltIncr);
            psParam-> u16AccState =(fltStateNew > psParam->fltXT1) + (fltStateNew > psParam->fltXT2);    
        }
        else 
        {   if (bStateHigherTarget)
            {
                fltStateNew = MLIB_Sub_FLT(fltState, fltIncr);
                psParam->u16AccState =(fltStateNew < psParam->fltXT1) + (fltStateNew < psParam->fltXT2);  
            } 
            else 
            {
                fltStateNew = fltState;        
            }
        }
    }    
    
    /* store quantities into structure */
    psParam -> fltIncr = fltIncr;
    psParam -> fltState = fltStateNew;
    psParam -> bReachFlag = (fltStateNew == psParam -> fltTarget);
    
    /* return rounded state value */
    return (fltStateNew);      
}

/***************************************************************************//*!
*
* @brief  Increment calculation for float flex s-ramp
*
* @param  in   float_t fltTarget   - Desired value in float_t
*              float_t fltDuration - Defined time [s] to go from the state to the 
*                                    target value in float_t
*
* @param ptr GFLIB_FLEXSRAMP_T_FLT *psParam
*            - fltState: State variable keeping the last state x  
*            - fltIncr: derivative of x (dX) - acceleration 
*            - fltAIncr: derivative of a (dA) - der. of acceleration 
*            - fltADes: desired acceleration - must be <= fltIncrmax 
*            - fltTarget: Target value x 
*            - fltTs: Sample time [s] 
*            - fltIncrMax: Maximum increment of x  
*            - fltXT1: X(T1) - x at the instant where the ramp got the desired  
*               acceleration (fltIncr = const)  
*            - fltXT2: X(T2) - x at the instant where the ramp started to decelerate  
*            - fltDA: Defined derivative of acceleration: dA = A / T  
*            - u16AccState: State of the ramp process 0: fltIncr rising,  
*                           1: fltIncr = const, 2: fltIncr falling  
*            - bReachFlag: Flag is set to TRUE if the desired value is achieved  
*
* @return bool_t - True if discriminant is positives or zero (S ramp can achieve 
*                  target value in desired time), false if not.
*       
* @remarks  The function calculates the acceleration increment based as follows:
* 
*           For the desired acceleration it solves the following quadratic equation:
*
*           fltADes = {fltDuration * fltDA - sqrt[(fltDuration * fltDA) ^ 2  
*                     - 4 * (fltTarget - fltState) * fltDA]} / 2 
*
*           if the discriminant is negative, i.e. 
*           fltDuration * fltDA) ^ 2 < 4 * (fltTarget - fltState) * fltDA:
*           then fltADes = sqrt([fltDA * (fltTarget - fltState)]
*           and returns FALSE otherwise TRUE
*           
*           if fltADes > fltIncrMax, then fltADes = fltIncrMax 
*
*           Acceleration increment is then:
*           fltAIncr = fltTs ^ 2 * fltDA
*           
*           The X(T1) until the fltIncr is incremented - as integration 1/2 * fltADes ^ 2 * fltDA:      
*           fltXT1 = fltState +(-) (fltTs ^ 2 * fltDA) / 2
*
*           The X(T2) while the fltIncr is const:       
*           fltXT2 = fltTarget -(+) (fltTs ^ 2 * fltDA) / 2
*
*           fltTarget = fltTarget
*           
*           result = (fltTarget - fltState) * (fltTs / fltDuration)
*           u16AccState = 0
*           The bReachFlag flag is cleared.
*
****************************************************************************/
bool_t GFLIB_FlexSRampCalcIncr_FLT_FC(float_t fltTarget, float_t fltDuration, 
                                      GFLIB_FLEXSRAMP_T_FLT *psParam)
{
    register float_t fltD;
    register float_t fltDiscr1;
    register float_t fltDiscr2;
    register float_t fltADes;
    register float_t fltXT1;
    register float_t fltDx;
    register float_t fltDA;
    register float_t fltTemp;
    register bool_t bStatus;

    /* store more time red quantities to register */
    fltDA = psParam -> fltDA; 
    bStatus = FALSE;
    
    /* Ramping range calculation */ 
    fltDx = MLIB_Sub_FLT(fltTarget, psParam -> fltState);
    
    /* Desired acceleration = {T * dA - Sqrt[(T * dA) ^ 2 - 4 * dX * dA]} / 2 */
    /* T * dA calculation */
    fltD = MLIB_Mul_FLT(fltDuration, fltDA);
    
    /* Discriminant calculation: Sqrt[(T * dA) ^ 2 - 4 * dX * dA] : */
    /* First portion of the discriminant: (T * dA) ^ 2 */
    fltDiscr1 = MLIB_Mul_FLT(fltD, fltD);
    /* Second portion of the discriminant: 4 * dX * dA : used three shifts left because
     * fltDx was shifted to the right once before */
    fltDiscr2 = MLIB_Mul_FLT(MLIB_Mul_FLT(MLIB_Abs_FLT(fltDx), fltDA), 4.0);
    
    /* Desired acceleration calculation */
    /* if the discriminant is >= 0, the ramp is achievable in time (dA = 0) */ 
    if (fltDiscr1 >= fltDiscr2)
    {
        fltADes = MLIB_Mul_FLT(fltD - GFLIB_Sqrt_FLT(fltDiscr1 - fltDiscr2), 0.5F);
        bStatus = TRUE;
    }
    /* if the discriminant is < 0, the ramp is not achievable in time */
    else
    {
        fltADes = GFLIB_Sqrt_FLT(MLIB_Mul_FLT(fltDiscr2, 0.25F));
        bStatus = FALSE;
    }

    /* Limitation of ADes = DA * Ts^2 to f32IncrMax */
    fltTemp = MLIB_Mul_FLT(MLIB_Mul_FLT(psParam -> fltTs, psParam -> fltTs),psParam -> fltDA);
    /* if the desired acceleration > maximum allowed increment, 
     * then it is set to the max. increment */
    if (fltTemp > psParam -> fltIncrMax)
    {
        fltADes = psParam -> fltIncrMax;
        bStatus = FALSE;
    }
    
    /* Integration of the acceleration to the time where A(T1) = ADes:
     * A(T1) = dA * T1 => T1 = A(T1) / dA = ADes / dA
     * x(T1) = 1/2 * dA * T1 ^ 2 = 1/2 * dA * [ADes / dA] ^ 2 = 1/2 * ADes ^ 2 / dA */    
    fltXT1 = MLIB_Div_FLT(MLIB_Mul_FLT(fltADes, fltADes), MLIB_Mul_FLT (fltDA, 2.0F));

    /* Calculation instants among constant or zero jerk intervals */
    if (fltDx < 0)   /* If slope is negative */
    {
        psParam -> fltXT1 = MLIB_Sub_FLT(psParam -> fltState, fltXT1);
        psParam -> fltXT2 = MLIB_Add_FLT(fltTarget, fltXT1);
    }
    else             /* If slope is positive */
    {
        psParam -> fltXT1 = MLIB_Add_FLT(psParam -> fltState, fltXT1);
        psParam -> fltXT2 = MLIB_Sub_FLT(fltTarget, fltXT1);
    }

    /* store quantities into structure */ 
    psParam -> fltAIncr = fltTemp;    /* ADes = DA * Ts^2 */
    psParam -> fltADes  = fltADes;    /* limited ADes = DA * Ts^2 */  
    psParam -> fltTarget= fltTarget;  /* set target value */
    psParam -> fltIncr  = 0.0F;       /* clear S ramp increment */
    psParam -> u16AccState = 0;       /* set firs S ramp interval */
    psParam -> bReachFlag  = FALSE;   /* clear bReachFlag */
    /* return bool value of discriminant polarity */
    return (bStatus);
}  
