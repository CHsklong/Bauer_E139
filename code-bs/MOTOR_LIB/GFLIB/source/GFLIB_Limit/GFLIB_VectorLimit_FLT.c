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
* @brief  Vector limit functions with float output 
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_VectorLimit_FLT.h"

/***************************************************************************//*!
*
* @brief    float vector magnitude limit function
*
* @param    ptr   *psVectorIn  - Pointer to the input vector
*                 *psVectorOut - Pointer to the output vector
*   
* @param    in    float_t fltLim - Limit of vector magnitude in float_t
*
* @return   This function returns - bool_t value
*       
* @remarks  This function limits the magnitude of the input vector. The function will 
*           return true "TRUE" if the input vector is being limited, or false
*           "FALSE" otherwise.
*   
*           |y_In * Limit/sqrt(y_In^2 + x_In^2), if sqrt(y_In^2 + x_In^2) > Limit
*   y_out = |
*           |y_In, if sqrt(y_In^2 + x_In^2) <= Limit
*
*           |x_In * Limit/sqrt(y_In^2 + x_In^2), if sqrt(y_In^2 + x_In^2) > Limit
*   x_out = |
*           |x_In, if sqrt(y_In^2 + x_In^2) <= Limit
*
****************************************************************************/
bool_t GFLIB_VectorLimit_FLT_FC(const GFLIB_VECTORLIMIT_T_FLT *psVectorIn, 
                                float_t fltLim,
                                GFLIB_VECTORLIMIT_T_FLT *psVectorOut)
{
    register float_t fltInLength;    /* length of the input vector */
    register float_t fltLenQuotient; /* the quotient length */
    register float_t fltA, fltB;
    fltA = psVectorIn->fltA;
    fltB = psVectorIn->fltB;

    /* Count the length of the input vector */
    fltInLength = GFLIB_Sqrt_FLT(MLIB_Mac4_FLT(fltB,fltB,fltA,fltA));
    /* Vector limitation has occurred */
    if((fltInLength) > fltLim)
    {   /* count the length quotient */
        fltLenQuotient = MLIB_Div_FLT(fltLim,fltInLength); 
        /* count the limited vector X,Y coordinate */
        psVectorOut->fltA = MLIB_Mul_FLT(fltA,fltLenQuotient); 
        psVectorOut->fltB = MLIB_Mul_FLT(fltB,fltLenQuotient); 

        return(TRUE); /* limitation has occurred, return logical 1 */    
    }
    else 
    {
        psVectorOut->fltA = fltA;
        psVectorOut->fltB = fltB;
        return(FALSE); /* limitation has not occurred, return logical 0 */
    }
}

/***************************************************************************//*!
*
* @brief    float vector B component limit function
*
* @param    ptr   *psVectorIn  - Pointer to the input vector
*                 *psVectorOut - Pointer to the output vector
*   
* @param    in    float_t fltLim - Limit of vector magnitude in float_t
*
* @return   This function returns - bool_t value
*       
* @remarks  This function limits the magnitude of the input vector. The function will 
*           return true "TRUE" if the input vector is being limited, or false
*           "FALSE" otherwise.
*           
*                 |fltA, if |fltA| <= fltLim
*     out.fltA  = |
*                 |fltLim * sign(fltA), if |fltA| > fltLim
*
*                 |fltB, if |fltB| <= sqrt(fltLim ^ 2 - out.fltA ^ 2)
*     out.fltB  = |
*                 |sqrt(fltLim ^ 2 - out.fltA ^ 2) * sign(fltB), if |fltB| > sqrt(fltLim ^ 2 - out.fltA ^ 2)
*
****************************************************************************/
bool_t GFLIB_VectorLimit1_FLT_FC(const GFLIB_VECTORLIMIT_T_FLT *psVectorIn, 
                                 float_t fltLim,
                                 GFLIB_VECTORLIMIT_T_FLT *psVectorOut)
{
    register float_t fltA;
    register float_t fltB;
    register float_t fltA_Abs;
    register float_t fltB_Abs;    
    register float_t fltB_Lim;      
    bool_t bFlagOut;
    
    /* Set init quantities values */ 
    bFlagOut = FALSE; 
    fltA = psVectorIn->fltA;
    fltB = psVectorIn->fltB;
    
    /* Calculating absolute value of A component */
    fltA_Abs = MLIB_Abs_FLT (fltA);
    /* A component limitation */
    if (fltA_Abs > fltLim)
    {   fltA = (fltA < 0.0F) ? -fltLim : fltLim;  
        bFlagOut = TRUE;        
    }
    
    /* Calculating absolute value of B component */
    fltB_Abs = MLIB_Abs_FLT (fltB);
    /* Calculating limitation for B components from assigned limit and A component */
    fltB_Lim = GFLIB_Sqrt_FLT(MLIB_Msu4_FLT(fltLim, fltLim, fltA, fltA));    
    /* B component limitation */
    if (fltB_Abs > fltB_Lim)
    {   fltB = (fltB < 0.0F) ? -fltB_Lim : fltB_Lim;  
        bFlagOut = TRUE;        
    }
    
    /* Store results to output structure and return flag */
    psVectorOut->fltB = fltB;
    psVectorOut->fltA = fltA;
    return(bFlagOut);
}
