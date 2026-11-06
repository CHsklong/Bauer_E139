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
* @brief  Vector limit functions with 16-bit and 32-bit fractional output
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_VectorLimit_F16.h"

/***************************************************************************//*!
*
* @brief    16-bit vector magnitude limit function
*
* @param    ptr  *psVectorIn  - pointer to the input vector
*                *psVectorOut - pointer to the output vector
*   
* @param    in   frac16_t f16Lim - Limit of vector magnitude in <0;1) in frac16_t
*
* @return   This function returns - bool_t value
*       
* @remarks  This function limits the magnitude of the input vector. The function will 
*           return true "TRUE" if the input vector is being limited, or false
*           "FALSE" otherwise.
*           
*             |y_In * Limit/sqrt(y_In^2 + x_In^2), if sqrt(y_In^2 + x_In^2) > Limit
*   y_out  =  |
*             |y_In, if sqrt(y_In^2 + x_In^2) <= Limit
*
*             |x_In * Limit/sqrt(y_In^2 + x_In^2), if sqrt(y_In^2 + x_In^2) > Limit
*   x_out  =  |
*             |x_In, if sqrt(y_In^2 + x_In^2) <= Limit
*
****************************************************************************/
bool_t GFLIB_VectorLimit_F16_FC(const GFLIB_VECTORLIMIT_T_F16 *psVectorIn,
                                frac16_t f16Lim, 
                                GFLIB_VECTORLIMIT_T_F16 *psVectorOut)
{
    register frac32_t f32InSquare;
    register frac32_t f32LimitSquare;
    register frac16_t f16LenQuotient, f16InLength;

    f32InSquare = MLIB_AddSat_F32(((frac32_t)psVectorIn->f16A * (frac32_t)psVectorIn->f16A),
                                  ((frac32_t)psVectorIn->f16B * (frac32_t)psVectorIn->f16B)); 
    f32LimitSquare = (frac32_t)f16Lim * (frac32_t)f16Lim; 

    if (f32InSquare > f32LimitSquare)
    {
        f16InLength = (GFLIB_Sqrt_F16l(f32InSquare));
        /* count the length quotient */
        f16LenQuotient = (frac16_t)(MLIB_Div1QSat_F16ls((MLIB_Mul_F32ss((frac16_t)0x5A82,(f16Lim))),f16InLength)); 
        /* count the limited vector, X,Y coordinate */
        psVectorOut->f16A = MLIB_Conv_F16l(MLIB_Mul_F32ss(psVectorIn->f16A,f16LenQuotient));
        psVectorOut->f16B = MLIB_Conv_F16l(MLIB_Mul_F32ss(psVectorIn->f16B,f16LenQuotient)); 

        return(TRUE); /* limitation has occurred, return logical 1 */
    }
    else
    {
        psVectorOut->f16A = psVectorIn->f16A;
        psVectorOut->f16B = psVectorIn->f16B;
        return(FALSE); /* limitation has not occurred, return logical 0 */
    }
} 

/***************************************************************************//*!
*
* @brief    16-bit vector B component limit function
*
* @param    ptr   *psVectorIn  - pointer to the input vector
*                 *psVectorOut - pointer to the output vector
*   
* @param    in    frac16_t f16Lim - Limit of vector magnitude in <0;1) in frac16_t
*
* @return   This function returns - bool_t value
*       
* @remarks  This function limits the magnitude of the input vector. The function will 
*           return true "TRUE" if the input vector is being limited, or false
*           "FALSE" otherwise.
*           
*                    |f16A, if |f16A| <= f16Lim
*    out.f16A   =    |
*                    |f16Lim * sign(f16A), if |f16A| > f16Lim
*
*                    |f16B, if |f16B| <= sqrt(f16Lim ^ 2 - out.f16A ^ 2)
*    out.f16B   =    |
*                    |sqrt(f16Lim ^ 2 - out.f16A ^ 2) * sign(f16B), if |f16B| > sqrt(f16Lim ^ 2 - out.f16A ^ 2)
*
****************************************************************************/
bool_t GFLIB_VectorLimit1_F16_FC(const GFLIB_VECTORLIMIT_T_F16 *psVectorIn,
                                frac16_t f16Lim, 
                                GFLIB_VECTORLIMIT_T_F16 *psVectorOut)
{
    register frac32_t f32A;
    register frac32_t f32B;
    register frac32_t f32A_Abs;
    register frac32_t f32B_Abs;
    register frac32_t f32B_Lim;  
    register frac32_t f32Temp;
    bool_t bFlagOut;
    
    /* Set init quantities values */ 
    bFlagOut = FALSE; 
    f32A = psVectorIn->f16A;
    f32B = psVectorIn->f16B;

    /* Calculating absolute value of A component */
    f32A_Abs = MLIB_AbsSat_F16(f32A);
    /* A component limitation */
    if (f32A_Abs > f16Lim)
    {   f32A = (f32A < 0) ? -f16Lim : f16Lim;  
        bFlagOut = TRUE;        
    }
    
    /* Calculating absolute value of B component */
    f32B_Abs = MLIB_AbsSat_F16(f32B);
    /* Calculating limitation for B components from assigned limit and A component */  
    f32Temp = MLIB_Msu4_F32ssss(f16Lim, f16Lim, f32A, f32A);  
    f32B_Lim = GFLIB_Sqrt_F16l(f32Temp);   
    /* Store results to output structure and return flag */
    if (f32B_Abs > f32B_Lim)
    {   f32B = (f32B < 0) ? (-f32B_Lim) : f32B_Lim;  
        bFlagOut = TRUE;        
    }   
    
    /* Store results to output structure and return flag */
    psVectorOut->f16A = (frac16_t)f32A;
    psVectorOut->f16B = (frac16_t)f32B;
    return(bFlagOut);
}
