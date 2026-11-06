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
#include "GFLIB_Acos_FLT.h"
 
/*******************************************************************************
* Global variables
*******************************************************************************/
GFLIB_CONST GFLIB_ACOS_T_FLT fltgflibAcosCoef = {{
    -1.87253000000E-2, 7.42610000000E-2, -2.121143800000E-1, 
     1.5707324210000F, 1.5707963267949F  }};

/***************************************************************************//*!
*
* @brief    The GFLIB_Acos function computes the Acos(x) using    
*           piece-wise polynomial approximation.
*
* @param    ptr  *psParam - Pointer to the table
*
* @param    in   fltVal- The input data value is in the range of float, which
*                        corresponds to the angle in the range of [-pi,pi).
*
* @return   The function returns Acos(x).
*     
*       
* @remarks  The GFLIB_Acos function computes the Acos(x) using     
*           Asin(x) piece-wise polynomial approximation. 
*    
*           Output = Acos(x)
*           x(fltVal) =  Sqrt(1-|fltVal|)   
*
*           Asin(x) = (a4 - SQRT(1 - x)(a3 + x(a2 + x(a1 + x * a0)))) 
*
*           Output  =   | -Asin(fltVal)     if(fltVal < 0)
*                       |
*                       |  Asin(flt6Val)    if(fltVal >= 0)
*
****************************************************************************/
float_t GFLIB_Acos_FLT_FC(float_t fltVal, GFLIB_CONST GFLIB_ACOS_T_FLT *const psParam)
{
    float_t fltX;
    float_t fltY;
    float_t fltZ;
    float_t fltW;

    /* Absolute value */
    fltX = MLIB_Abs_FLT(fltVal);

    /* 1-|fltIn| */
    fltY = MLIB_Sub_FLT(1.0F, fltX);

    /* Square root of (1 - |fltIn|) */
    fltY = GFLIB_Sqrt_FLT(fltY);

    /*------------------------------------------------------------------------
    / (a4 - SQRT(1 - x)(a3 + x(a2 + x(a1 + x * a0))))
    /------------------------------------------------------------------------*/
    /* a1 + x * a0 */
    fltZ = MLIB_Mac_FLT(psParam->fltA[1], fltX, psParam->fltA[0]);

    /* a2 + x(a1 + x * a0) */
    fltZ = MLIB_Mac_FLT(psParam->fltA[2], fltX, fltZ);

    /* a3 + x(a2 + x(a1 + x * a0)) */
    fltZ = MLIB_Mac_FLT(psParam->fltA[3], fltX, fltZ);

    /* (a4 - SQRT(1 - x)(a3 + x(a2 + x(a1 + x * a0)))) */
    fltW = MLIB_Mul_FLT(fltY, fltZ);
    fltZ = MLIB_Sub_FLT(psParam->fltA[4], fltW);

    if (fltVal < 0.0F)
    {
        fltZ = MLIB_Neg_FLT(fltZ);
    }
    fltZ = MLIB_Sub_FLT(1.57079632679490F, fltZ);   
    return(fltZ);
}
