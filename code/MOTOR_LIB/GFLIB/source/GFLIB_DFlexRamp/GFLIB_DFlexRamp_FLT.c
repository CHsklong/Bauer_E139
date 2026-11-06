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
* @brief  Float dynamic flex ramp functions
* 
*******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_DFlexRamp_FLT.h"

/***************************************************************************//*!
*
* @brief  Float dynamic flex ramp calculation
*
* @param  in    float_t fltInstant - Measured instant value in float_t
*
* @param  ptr   const bool_t *bStopFlagMot - If true the motor mode saturation mode is on
*               const bool_t *bStopFlagGen - If true the generator mode saturation mode is on    
*               GFLIB_DFLEXRAMP_T_FLT *psParam
*                 - fltState: State variable keeping the last state, assigned by GFLIB_DFlexRampInit_FLT
*                 - fltIncr: Increment - calculated by GFLIB_DFlexRampCalcIncr_FLT function
*                 - fltIncrSatMot: Motor mode saturation mode increment - assigned by user(positive value), 
*                                  sign can be changed by GFLIB_DFlexRampCalcIncr_FLT
*                 - fltIncrSatGen: Generator mode saturation mode increment - assigned by user(positive value),
*                                  sign can be changed by GFLIB_DFlexRampCalcIncr_FLT
*                 - fltTarget: Target value - assigned by GFLIB_DFlexRampCalcIncr_FLT function
*                 - fltTs: Sample time [s] - assigned by user(positive value)
*                 - fltIncrMax: Maximum increment - assigned by user(positive value)
*                 - bReachFlag: Flag is set to TRUE if the desired value is achieved
*
* @return Ramp output - float_t value
*       
* @remarks  If both flags are cleared the functions adds fltIncr to the fltState value. 
*           For positive increment : if fltState + fltIncr >= fltTarget then 
*               the output is trimmed to the fltTarget value and the bReachFlag is set
*           For negative increment : if fltState + fltIncr <= fltTarget then 
*               the output is trimmed to the fltTarget value and the bReachFlag is set
*
*           If *bStopFlagMot & (|fltTarget| > |fltState|) then then the function adds
*           fltIncrSatMot to fltState returning to the instant value.
*           If |fltState + fltIncrSatMot| < |fltInstant| then it is trimmed to the fltInstant value.
*
*           If *bStopFlagGen & (|fltTarget| < |fltState|) then then the function adds
*           fltIncrSatGen to fltState returning to the instant value.     
*           If |fltState + fltIncrSatGen| > |fltInstant| then it is trimmed to the fltInstant value.                 
*
*           The output is stored into the fltState variable.
*
***************************************************************************/
float_t GFLIB_DFlexRamp_FLT_FC(float_t fltInstant,
                               const bool_t *pbStopFlagMot,
                               const bool_t *pbStopFlagGen,
                               GFLIB_DFLEXRAMP_T_FLT *psParam)
{
    register float_t fltState;
    register float_t fltIncr;    
    register float_t fltIncrSatMot;    
    register float_t fltIncrSatGen;    
    register float_t fltTarget;         
    register float_t fltTargetAbs;
    register float_t fltStateAbs; 
    
    /* Store more times red quantities to registers */     
    fltState = psParam -> fltState;
    fltIncr  = psParam -> fltIncr;
    fltIncrSatMot = psParam -> fltIncrSatMot;
    fltIncrSatGen = psParam -> fltIncrSatGen;    
    fltTarget   = psParam -> fltTarget;       
    fltStateAbs = MLIB_Abs_FLT(fltState);
    fltTargetAbs= MLIB_Abs_FLT(fltTarget); 
    
    /* Ramp calculation with motor flag */
    if ((*pbStopFlagMot) && (fltTargetAbs > fltStateAbs))
    {
        fltState = MLIB_Add_FLT(fltState, fltIncrSatMot);
        if (fltIncrSatMot > 0) 
        {   if (fltState > fltInstant) 
            {   
                fltState = fltInstant;    
            }
        }    
        else
        {   if (fltState < fltInstant) 
            {   
                fltState = fltInstant;
            }
        } 
    } /* Ramp calculation with generator flag */ 
    else if ((*pbStopFlagGen) && (fltTargetAbs < fltStateAbs))
    {
        fltState = MLIB_Add_FLT(fltState, fltIncrSatGen);
        if (fltIncrSatGen > 0) 
        {   if (fltState > fltInstant) 
            {   
                fltState = fltInstant;    
            }
        }    
        else
        {   if (fltState < fltInstant) 
            {   
                fltState = fltInstant;
            }
        } 
    } /* Normal ramp calculation without any flags */    
    else  
    {
        fltState = MLIB_Add_FLT(fltState, fltIncr);
        /* If ramp crossed the target value */
        if (((fltTarget < fltState) && (fltIncr > 0)) ||  
            ((fltTarget > fltState) && (fltIncr < 0))) 
        {   
            fltState = fltTarget;
        }   
        /* Set flag if ramp achieved target value */
        psParam -> bReachFlag = (fltState == fltTarget) ? TRUE : FALSE;  
    } 
    
    /* store quantities into structure */
    psParam -> fltState = fltState;   
    /* return state value */
    return((float_t)fltState);   
}


/***************************************************************************//*!
*
* @brief  Increment calculation for float dynamic flex ramp
*
* @param  in    float_t fltTarget   - Desired value in float_t
*               float_t  fltDuration - Defined time [s] to go from the state to the target value in float_t
*               float_t fltIncrSatMot - Increment for the motor mode when the saturation flag is set
*               float_t fltIncrSatGen - Increment for the generator mode when the saturation flag is set                          
*
* @param   ptr  GFLIB_DFLEXRAMP_T_FLT *psParam
*                 - fltState: State variable keeping the last state, assigned by GFLIB_DFlexRampInit_FLT
*                 - fltIncr: Increment - calculated by GFLIB_DFlexRampCalcIncr_FLT function
*                 - fltIncrSatMot: Motor mode saturation mode increment - assigned by user(positive value), 
*                                  sign can be changed by GFLIB_DFlexRampCalcIncr_FLT
*                 - fltIncrSatGen: Generator mode saturation mode increment - assigned by user(positive value),
*                                  sign can be changed by GFLIB_DFlexRampCalcIncr_FLT
*                 - fltTarget: Target value - assigned by GFLIB_DFlexRampCalcIncr_FLT function
*                 - fltTs: Sample time [s] - assigned by user(positive value)
*                 - fltIncrMax: Maximum increment - assigned by user(positive value)
*                 - bReachFlag: Flag is set to TRUE if the desired value is achieved
*
* @return  void
*       
* @remarks  The function calculates the increment based as follows:
*           result = (fltTarget - fltState) * (fltTs / fltDuration)
*           if |result| > |fltIncrMax| then fltIncr = fltIncrMax * sign(result)
*           otherwise fltIncr = result
*           fltIncrSatMot = fltIncrSatMot * sign(-result)
*           fltIncrSatGen = fltIncrSatGen * sign(-result)
*           The bReachFlag flag is cleared.
*
****************************************************************************/
void GFLIB_DFlexRampCalcIncr_FLT_FC(float_t fltTarget,
                                    float_t fltDuration,
                                    float_t fltIncrSatMot,
                                    float_t fltIncrSatGen,
                                    GFLIB_DFLEXRAMP_T_FLT *psParam)
{
    register float_t fltTemp;               
    
    /* Ramping quantity range calculation */ 
    fltTemp = MLIB_Sub_FLT (fltTarget, psParam -> fltState); 
    
    /* Division */
    /* Increment = (fltTs / fltDuration) * (fltTarget - psParam->fltState) */
    fltTemp = MLIB_Mul_FLT(fltTemp, psParam -> fltTs); 
    fltTemp = MLIB_Div_FLT(fltTemp, fltDuration);    

    /* Final limitation:  if |result| > |fltIncrMax| then fltIncr = fltIncrMax * sign(result)
       otherwise fltIncr = result, fltIncrSatMot = fltIncrSatMot * sign(-result),
       fltIncrSatGen = fltIncrSatGen * sign(-result) */
    if (MLIB_Abs_FLT(fltTemp) > psParam -> fltIncrMax)
        psParam -> fltIncr = (fltTemp >= 0) ? psParam -> fltIncrMax : -psParam -> fltIncrMax;   
    else 
        psParam -> fltIncr = fltTemp;
    
    /* clear bReachFlag */
    psParam -> bReachFlag = FALSE;
    /* store quantities into structure */
    psParam -> fltTarget = fltTarget;      
    /* Saturation motor increment is always opposite against result(calculated increment) but 
       saturation generator increment has the same sign as result(calculated increment) */
    psParam -> fltIncrSatMot = (fltTemp >= 0.0F) ? -fltIncrSatMot :  fltIncrSatMot;
    psParam -> fltIncrSatGen = (fltTemp >= 0.0F) ?  fltIncrSatGen : -fltIncrSatGen;      
}    

