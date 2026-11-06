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
* @brief  Flex ramp functions with 16-bit fractional output 
* 
*******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_FlexRamp_A32.h"

/***************************************************************************//*!
*
* @brief  Flex ramp calculation
*
* @param  ptr  GFLIB_FLEXRAMP_T_F32 *psParam
*                - f32State: State variable keeping the last state, assigned by GFLIB_FlexRampInit_F16
*                - f32Incr: Increment - calculated by GFLIB_FlexRampCalcIncr_F16 function
*                - f32Target: Target value - assigned by GFLIB_FlexRampCalcIncr_F16 function
*                - f32Ts: Sample time [s] - assigned by user(positive value)
*                - f32IncrMax: Maximum increment - assigned by user(positive value)
*                - bReachFlag: Flag is set to TRUE if the desired value is achieved
*
* @return Ramp output - frac16_t value <-1;1)
*       
* @remarks  The functions adds f32Incr to the f32State value.
*           For positive increment : if f32State + f32Incr >= f32Target then 
*               the output is trimmed to the f32Target value and the bReachFlag is set
*           For negative increment : if f32State + f32Incr <= f32Target then 
*               the output is trimmed to the f32Target value and the bReachFlag is set
*           The output is stored into the f32State variable.
*
****************************************************************************/
frac16_t GFLIB_FlexRamp_F16_FC(GFLIB_FLEXRAMP_T_F32 *psParam)
{
    register frac32_t f32State;
    register frac32_t f32Incr;    
    register frac32_t f32Target;   
    
    /* Store more times red quantities to registers */ 
    f32State = psParam -> f32State;
    f32Incr  = psParam -> f32Incr; 
    f32Target= psParam -> f32Target; 
    
    /* Ramp calculation */
    f32State = MLIB_AddSat_F32(f32State, f32Incr);
    if (((f32Target > f32State) && (f32Incr > 0)) ||
        ((f32Target < f32State) && (f32Incr < 0))) 
    {   
        psParam -> bReachFlag = FALSE; 
    }
    else 
    {
        f32State = f32Target;   
        psParam -> bReachFlag = TRUE;             
    }        
    
    /* store quantities into structure */
    psParam -> f32State = f32State;   
    /* return state value */
    return((frac16_t)(f32State >> 16));    
}

/***************************************************************************//*!
*
* @brief  Increment calculation for Flex ramp
*
* @param  in   frac16_t f16Target  - Desired value in <-1;1) in frac16_t
*              acc32_t a32Duration - Defined time [s] to go from the state to the 
*                                    target value in [0;65535.9] in acc32_t
*
* @param  ptr  GFLIB_FLEXRAMP_T_F32 *psParam
*                - f32State: State variable keeping the last state, assigned by GFLIB_FlexRampInit_F16
*                - f32Incr: Increment - calculated by GFLIB_FlexRampCalcIncr_F16 function
*                - f32Target: Target value - assigned by GFLIB_FlexRampCalcIncr_F16 function
*                - f32Ts: Sample time [s] - assigned by user(positive value)
*                - f32IncrMax: Maximum increment - assigned by user(positive value)
*                - bReachFlag: Flag is set to TRUE if the desired value is achieved
*
* @return void
*       
* @remarks  The function calculates the increment based as follows:
*           result = (f16Target - f32State) * (f32Ts / a32Duration)
*           if |result| > |f32IncrMax| then f32Incr = f32IncrMax * sign(result)
*           otherwise f32Incr = result
*           The bReachFlag flag is cleared.
*
****************************************************************************/
void GFLIB_FlexRampCalcIncr_F16_FC(frac16_t f16Target,
                                   acc32_t a32Duration,
                                   GFLIB_FLEXRAMP_T_F32 *psParam)
{
    register frac32_t f32Temp;      
    register frac32_t f32Result;     
    register frac32_t f32Target;   
    register frac32_t f32TsNorm;
    register frac32_t f32Ts;
    register acc32_t  a32DurationNorm;
    register int16_t  s16DurationSh;
    register int16_t  s16SumSh;  
   
    /* Store more times red quantities to registers */   
    f32Target= f16Target << 15;  /* 16-1=15 because of ramping range can be 2 */  
    f32Ts    = psParam -> f32Ts;        
    
    /* Ramping range calculation */ 
    /* Shifted to the right once because the result can be more than 1 */
    f32Temp = MLIB_Sub_F32 (f32Target, psParam -> f32State>>1); 
    
    /* normalization */
    s16DurationSh = MLIB_Clb_U16l (a32Duration);    /* Normalization of a32Duration to the range 0.5 to 1.0 */ 
    a32DurationNorm = a32Duration << s16DurationSh;  
    s16SumSh = (int16_t)MLIB_Clb_U16l (f32Ts) - 1;  /* Normalization of Ts to range 0.25 to 0.5 for division */  
    f32TsNorm = MLIB_ShLBiSat_F32(f32Ts, s16SumSh);     
    s16SumSh += 16 - s16DurationSh - 1;       /* final shift = TsShift + 16 (accumulator division) - DurationShift */
                                              /*               - 1 (because it was shifted right before) */
    /* Division */
    /* Increment = (f32Ts / a32Duration) * SubSat(f16Target << 16, psParam -> f32State) */
    f32Result = MLIB_Div_F32(f32TsNorm, a32DurationNorm);
    f32Result = MLIB_MulRnd_F32(f32Result, f32Temp);   
    f32Result = MLIB_ShRBiSat_F32 (f32Result, s16SumSh);  

    /* Final limitation:  if |result| > |f32IncrMax| then f32Incr = f32IncrMax * sign(result)
       otherwise f32Incr = result */
    if (MLIB_AbsSat_F32(f32Result) > psParam -> f32IncrMax)
        psParam -> f32Incr = (f32Result >= 0) ? psParam -> f32IncrMax : -psParam -> f32IncrMax;   
    else 
        psParam -> f32Incr = f32Result;
    
    /* clear bReachFlag */
    psParam -> bReachFlag = FALSE;
    /* store quantities into structure */
    psParam -> f32Target = MLIB_Conv_F32s(f16Target);    
}
