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
* @brief  Arcus sine function
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_Asin_FLT.h"

/*******************************************************************************
* Global variables
*******************************************************************************/  
GFLIB_CONST GFLIB_ASIN_T_FLT fltgflibAsinCoef = {{
    -1.872130E-2,      7.42610E-2,        -2.120944E-1,        1.57075239F, 
     1.5707963267949F, 0.082733681152843F, 0.166350537976503F, 1.000001892183630F }};

/***************************************************************************//*!
*
* @brief    The GFLIB_Asin function computes the Asin(x) using    
*           piece-wise polynomial approximation.
*
* @param    ptr  *psParam - Pointer to the table
*
* @param    in   fltVal - The input data value is in the range of float, which
*                         corresponds to the angle in the range of <-pi,pi).                         
*
* @return   The function returns Asin(x).     
*       
* @remarks  The GMCLIB_Asin function computes the Asin(x) using piece-wise 
*           polynomial approximation.
*
*           Output = Asin(x)
*
*           fltX = |fltVal|
*           Asin(x) = | (a4 - SQRT(1 - x)(a3 + x(a2 + x(a1 + x * a0))))  if (fltX >= 0.325)
*                     | 
*                     | (x * (a7 + x^2(a6 + x^2 * a5))))                 if (fltX < 0.325)
*
*           Output  = | -Asin(fltVal)     if(fltVal < 0)
*                     |
*                     |  Asin(flt6Val)    if(fltVal >= 0)
*
****************************************************************************/
float_t GFLIB_Asin_FLT_FC(float_t fltVal, GFLIB_CONST GFLIB_ASIN_T_FLT *const psParam)
{
    float_t fltX;
    float_t fltY;
    float_t fltZ;
    float_t fltW;

    /* Absolute value */
    fltX = MLIB_Abs_FLT(fltVal);

    if (fltX >= 0.325F)
    {
        /* Square root based approximation for interval [-1, -0.325] and interval [0.325, 1] */

        /* 1-|x| */
        fltY = MLIB_Sub_FLT(1.0F, fltX);

        /* Square root of (1-|f32In|) */
        fltY = GFLIB_Sqrt_FLT(fltY);

        /*------------------------------------------------------------------------
        * (a4 - SQRT(1 - x)(a3 + x(a2 + x(a1 + x * a0))))
        *------------------------------------------------------------------------*/
        /* a1 + x * a0 */
        fltZ = MLIB_Mac_FLT(psParam->fltA[1], fltX, psParam->fltA[0]);

        /* a2 + x(a1 + x * a0) */
        fltZ = MLIB_Mac_FLT(psParam->fltA[2], fltX, fltZ);

        /* a3 + x(a2 + x(a1 + x * a0)) */
        fltZ = MLIB_Mac_FLT(psParam->fltA[3], fltX, fltZ);

        /* (a4 - SQRT(1 - x)(a3 + x(a2 + x(a1 + x * a0)))) */
        fltW = MLIB_Mul_FLT(fltY, fltZ);
        fltZ = MLIB_Sub_FLT(psParam->fltA[4], fltW);
    }
    else
    {
        /* Minimax approximation for interval (-0.325, 0.325) */

        /* |f32In|^2 */
        fltY = MLIB_Mul_FLT(fltX, fltX);

        /*------------------------------------------------------------------------
        * (x * (a7 + x^2(a6 + x^2 * a5))))
        ------------------------------------------------------------------------*/
        /* a1 +  x^2 * a2 */
        fltZ = MLIB_Mac_FLT(psParam->fltA[6], fltY, psParam->fltA[5]);

        /* a0 + x^2(a1 + x^2 * a2) */
        fltZ = MLIB_Mac_FLT(psParam->fltA[7], fltY, fltZ);

        /* x * (a0 + x^2(a1 + x^2 * a2))) */
        fltZ = MLIB_Mul_FLT(fltZ, fltX);
    }

    if (fltVal < 0.0F)
    {
        fltZ = MLIB_Neg_FLT(fltZ);
    }
    return(fltZ);
}
