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
* @brief  Dynamic ramp
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_DRamp_FLT.h"

/***************************************************************************//*!
*
* @brief    Ramp function
*
* @param    ptr  GFLIB_DRAMP_T_F32 *psParam
*                  - f32RampUp: Ramp-up increment
*                  - f32RampDown: Ramp-down increment
*                  - f32RampUpSat: Ramp-up increment used in case of saturation
*                  - f32RampDownSat: Ramp-down increment used in case of saturation
*                  - f32State: Previous ramp value
*                  - bReachFlag: Flag is set to 1 if the desired value is achieved
* @param    in   frac32_t f32Target - Desired value in <-1;1) in frac32_t
*                frac32_t f32Instant- Instant value in <-1;1) in frac32_t
*                uint32_t pbStopFlag- determines the saturation mode: 0 non-saturation
*
* @return   This function returns - frac32_t value <-1;1)
*       
* @remarks  This function ramps the value from the actual value up/down to
*           the f32Target value using the up/down increments defined in
*           the pParam structure. In case of saturation (pbStopFlag != 0)
*           the function uses the saturation up/down increments and ramps
*           the value toward the f32Instant value.
*
*******************************************************************************/
float_t GFLIB_DRamp_FLT_FC(float_t fltTarget, float_t fltInstant, const bool_t *pbStopFlag, GFLIB_DRAMP_T_FLT *psParam)
{
    float_t fltResult;          /* Result of the ramp */
    float_t fltRampUp;          /* Ramp-up increment */
    float_t fltRampDown;        /* Ramp-down decrement */
    float_t fltDesiredValue;    /* Actual ramp value */
    bool_t  bTemp;
    
    /* take Target as an input, when StopFlag is clear or 
    when Desired value is lower than Instant value */
    bTemp = (MLIB_Abs_FLT(fltTarget) > MLIB_Abs_FLT(psParam->fltState));
    if (*pbStopFlag && bTemp)  
    {                                                                               
        fltRampUp = psParam->fltRampUpSat;
        fltRampDown = psParam->fltRampDownSat;
        fltDesiredValue = fltInstant;
    }
    else
    {
        fltRampUp = psParam->fltRampUp;
        fltRampDown = psParam->fltRampDown;
        fltDesiredValue = fltTarget;
    }
    fltResult = fltDesiredValue;  /* Copies fwDesiredValue to fwResult */

    if (fltDesiredValue > psParam->fltState) /* If desired > actual */
    {   /* The algorithm adds the rampUp to actual */
        fltResult = MLIB_Add_FLT(psParam->fltState, fltRampUp); 
        if (fltResult > fltDesiredValue) /* If it overshoots */
        {
            fltResult = fltDesiredValue; /* it uses the desired */
        }
    }
    else if (fltDesiredValue < psParam->fltState)   /* If desired < actual */
    {   /* The algorithm subtracts the rampDown from actual */
        fltResult = MLIB_Sub_FLT(psParam->fltState, fltRampDown);
        if (fltResult < fltDesiredValue) /* If it undershoots */
        {
            fltResult = fltDesiredValue;  /* it uses the desired */
        }
    }

    if (fltResult == fltTarget)
        psParam->bReachFlag = 1;
    else
        psParam->bReachFlag = 0;
    psParam->fltState = fltResult;  /* Saves the actual ramp value */
    return(fltResult);
}
