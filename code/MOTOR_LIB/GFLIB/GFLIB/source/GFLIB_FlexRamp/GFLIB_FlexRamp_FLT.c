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
* @brief  Float flex ramp functions 
* 
*******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_FlexRamp_FLT.h"

/***************************************************************************//*!
*
* @brief  Float flex ramp calculation
*
* @param  ptr  GFLIB_FLEXRAMP_T_FLT *psParam
*                - fltState: State variable keeping the last state, assigned by 
*                            GFLIB_FlexRampInit_FLT
*                - fltIncr: Increment - calculated by GFLIB_FlexRampCalcIncr_FLT function
*                - fltTarget: Target value - assigned by GFLIB_FlexRampCalcIncr_FLT function
*                - fltTs: Sample time [s] - assigned by user(positive value)
*                - fltIncrMax: Maximum increment - assigned by user(positive value)
*                - bReachFlag: Flag is set to TRUE if the desired value is achieved
*
* @return Ramp output - float_t value 
*       
* @remarks  The functions adds fltIncr to the fltState value.
*           For positive increment : if fltState + fltIncr >= fltTarget then 
*               the output is trimmed to the fltTarget value and the bReachFlag is set
*           For negative increment : if fltState + fltIncr <= fltTarget then 
*               the output is trimmed to the fltTarget value and the bReachFlag is set
*           The output is stored into the fltState variable.
*
****************************************************************************/
float_t GFLIB_FlexRamp_FLT_FC(GFLIB_FLEXRAMP_T_FLT *psParam)
{
    register float_t fltState;
    register float_t fltIncr;    
    register float_t fltTarget;   
    
    /* Store more times red quantities to registers */ 
    fltState = psParam -> fltState;
    fltIncr  = psParam -> fltIncr; 
    fltTarget= psParam -> fltTarget; 
    
    /* Ramp calculation */
    fltState = MLIB_Add_FLT(fltState, fltIncr);
    if (((fltTarget > fltState) && (fltIncr > 0)) ||
        ((fltTarget < fltState) && (fltIncr < 0))) 
    {   
        psParam -> bReachFlag = FALSE; 
    }
    else 
    {
        fltState = fltTarget;   
        psParam -> bReachFlag = TRUE;             
    }        
    
    /* store quantities into structure */
    psParam -> fltState = fltState;   
    /* return casted state value */
    return((float_t)fltState );                        
}

/***************************************************************************//*!
*
* @brief  Increment calculation for float flex ramp
*
* @param  in   frac16_t fltTarget  - Desired value in frac16_t
*              float_t fltDuration - Defined time [s] to go from the state to the 
*                                    target value in float_t
*
* @param ptr GFLIB_FLEXRAMP_T_FLT *psParam
*              - fltState: State variable keeping the last state, assigned by GFLIB_FlexRampInit_FLT
*              - fltIncr: Increment - calculated by GFLIB_FlexRampCalcIncr_FLT function
*              - fltTarget: Target value - assigned by GFLIB_FlexRampCalcIncr_FLT function
*              - fltTs: Sample time [s] - assigned by user(positive value)
*              - fltIncrMax: Maximum increment - assigned by user(positive value)
*              - bReachFlag: Flag is set to TRUE if the desired value is achieved
*
* @return void
*       
* @remarks  The function calculates the increment based as follows:
*           result = (fltTarget - fltState) * (fltTs / fltDuration)
*           if |result| > |fltIncrMax| then fltIncr = fltIncrMax * sign(result)
*           otherwise fltIncr = result
*           The bReachFlag flag is cleared.
*
****************************************************************************/
void GFLIB_FlexRampCalcIncr_FLT_FC(float_t fltTarget,
                                   float_t fltDuration,
                                   GFLIB_FLEXRAMP_T_FLT *psParam)
{
    register float_t fltTemp;               
    
    /* Ramping quantity range calculation */ 
    fltTemp = MLIB_Sub_FLT (fltTarget, psParam -> fltState); 
    
    /* Division */
    /* Increment = (fltTs / fltDuration) * (fltTarget - psParam->fltState) */
    fltTemp = MLIB_Mul_FLT(fltTemp, psParam -> fltTs); 
    fltTemp = MLIB_Div_FLT(fltTemp, fltDuration);    

    /* Final limitation:  if |result| > |fltIncrMax| then fltIncr = fltIncrMax * sign(result)
       otherwise fltIncr = result */
    if (MLIB_Abs_FLT(fltTemp) > psParam -> fltIncrMax)
        psParam -> fltIncr = (fltTemp >= 0) ? psParam -> fltIncrMax : -psParam -> fltIncrMax;   
    else 
        psParam -> fltIncr = fltTemp;
    
    /* clear bReachFlag */
    psParam -> bReachFlag = FALSE;
    /* store quantities into structure */
    psParam -> fltTarget = fltTarget;    
}
