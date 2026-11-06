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
* @brief  Arcus cosine function
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_Acos_F16.h"
  
/*******************************************************************************
* Global variables
*******************************************************************************/
GFLIB_CONST GFLIB_ACOS_T_F16 f16gflibAcosCoef = {{
   {{1403,  1012,   148,   10419,      1}},  /* Coef.  [0;0.5) */
   {{-800,  -560,  -231,  -14718,  16384}}   /* Coef.  [0.5;1) */
}};

/***************************************************************************//*!
*
* @brief    The GFLIB_Acos function computes the Acos(pi*x) using    
*           piece-wise polynomial approximation.
*
* @param    ptr  *psParam - Pointer to the table
*                *psParam - Pointer to the SQRT function table
*
* @param    in   f16Val - The input data value is in the range of [-1,1), which
*                         corresponds to the angle in the range of [-pi,pi).
*
* @return   The function returns Acos(pi*x).
*       
* @remarks  The GFLIB_Acos function computes the Acos(pi*x) using     
*           Asin(pi*x) piece-wise polynomial approximation. All Asin values
*           falling beyond [-1, 1), are truncated to -1 and 1      
*           respectively.calculates Asin of the input fractional
*           argument   
*
*           The the [0,1) interval is divided into two sub-intervals [0,0.5) and
*           (0.5,1], and polynomial approximation is done for each interval respectively.
*           Calculates Asin of the input fractional argument                                             
*
*           Output = Acos(x)
*           x(f16Val) = |  |f16Val|            if x(|f16Val| < 0.5) 
*                       |                      
*                       |  Sqrt(1-|f16Val|)    if x(|f16Val| >= 0.5)
*
*           Asin(x) = ((((A1*x + A2)*x) + A3)*x + A4)*x + A5 
*
*           Output  =   | -1 * (- 0.5 + Asin(f16Val))     if(f16Val < 0)
*                       |
*                       |  -1 * (- 0.5 - Asin(f16Val))    if(f16Val >= 0)
*
****************************************************************************/
frac16_t GFLIB_Acos_F16_FC(frac16_t f16Val,GFLIB_CONST GFLIB_ACOS_T_F16 *const psParam)
{
    frac16_t f16Sign,f16Dump;
    int16_t  s16nSector = 0;

    /* For special case, where the input value is -1, 
    set the output value to eliminate the calculation error */
    if(f16Val == (frac16_t)INT16_MIN)
    {
        return((frac16_t)INT16_MAX);    
    }
    else
    {
        f16Sign = f16Val;
        f16Val = MLIB_AbsSat_F16(f16Val);        /* x = |f16Val| */
        if (f16Val >= (frac16_t)(INT16_MAX/2))   /* If x >= 0.5 => x = Sqrt(1-|f16Val|) */
        {
            f16Val = MLIB_NegSat_F16(f16Val);
            f16Val = (f16Val ^ (frac16_t)(INT16_MIN));
            f16Val = GFLIB_Sqrt_F16(f16Val);
            s16nSector = 1;
        }
        /* (a1 * x) + a2 */
        f16Dump = MLIB_Mac_F16(psParam->GFLIB_ACOS_SECTOR_F16[s16nSector].f16A[1], 
                               psParam->GFLIB_ACOS_SECTOR_F16[s16nSector].f16A[0], f16Val );

        /* ((a1 * x) + a2) * x + a3 */
        f16Dump = MLIB_Mac_F16(psParam->GFLIB_ACOS_SECTOR_F16[s16nSector].f16A[2], f16Dump, f16Val);
        
        /* (((a1 * x) + a2) * x + a3) * x + a4 */
        f16Dump = MLIB_Mac_F16(psParam->GFLIB_ACOS_SECTOR_F16[s16nSector].f16A[3], f16Dump, f16Val);
        
        /* ((((a1 * x) + a2) * x + a3) * x + a4) * x + a5 */
        f16Dump = MLIB_Mac_F16(psParam->GFLIB_ACOS_SECTOR_F16[s16nSector].f16A[4], f16Dump, f16Val);
        
        /* Sign is added to the result value */
        f16Dump = (f16Sign > 0) ? f16Dump : MLIB_Neg_F16(f16Dump);
        
        return(MLIB_Sub_F16((frac16_t)((INT16_MAX/2)+1), f16Dump));    
    }
}
