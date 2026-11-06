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
* @brief  Arcus tangent algorithms using piece-wise polynomial approximation
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_Atan_FLT.h"

/*******************************************************************************
* Global variables
*******************************************************************************/
GFLIB_CONST GFLIB_ATAN_T_FLT gfltAtanCoef = {{
    49.53263287722540F, 9.4060435423162400F, 48.70107004404890F, 
    21.58787016702020F, 65.76632705089560F, 48.70107004404990F
}};

/***************************************************************************//*!
*
* @brief    The GFLIB_Atan function computes the Atan(x)
*           using piece-wise polynomial approximation.          
*
* @param    ptr  *psParam - Pointer to the table
*
* @param    in   fltVal - The input data value is 32-bit single precision 
*                         floating-point value within the full type's range.                       
*
* @return   The function returns arctangent of the input as a 32-bit single
*           precision floating-point value in radians.              
*       
* @remarks  Output value is:
*
*           Atan(x) =  sign(x) * ((A1*|x| + A2)*|x| + A3)
*
*******************************************************************************/
float_t GFLIB_Atan_FLT_FC(float_t fltVal, GFLIB_CONST GFLIB_ATAN_T_FLT *const psParam)
{   
    float_t fltX;
    float_t fltY;
    float_t fltZ;
    float_t fltW;
    float_t fltV;
    bool_t  bRange_flag; 

    fltW = (float_t)(0);
    bRange_flag = FALSE;

    fltY = MLIB_Abs_FLT(fltVal);  /* Prepare absolute value of fltIn */
    fltV = fltY;

    if (fltY > (float_t)(1))      /* Use the reciprocal value for |fltIn| > 1 */
    {
        bRange_flag = TRUE;
        /* Division by 0 handled by if condition, MLIB_Div not used due to efficiency reason */
        fltV = (MLIB_Div_FLT(1.0F,fltV));

        /* Prepare final addition */
        fltW = 1.57079632679490F;       
    }

    if(fltV > 0.267949192431123000F)     /* Reduction to interval [0;Pi/12] */
    {           
        fltV = MLIB_Div_FLT(MLIB_Sub_FLT(fltV, 0.577350269189626000F), 
               MLIB_Mac_FLT((float_t)1, 0.577350269189626000F, fltV));
        fltW = MLIB_Sub_FLT(fltW, 0.523598775598299000F);  
    }    
    fltX = MLIB_Mul_FLT(fltV, fltV);     /* x =  fltIn^2 */

    /*----------------------------------------------------------------------
    x * (a1 + x^2(a2 + a3 * x^2)) / (a4 + x^2(a5 + x^2(a6 + x^2)))
    /---------------------------------------------------------------------*/

    /* (a2 + a3 * x^2) */
    fltY = MLIB_Mac_FLT(psParam->fltA[0], psParam->fltA[1], fltX);

    /* (a1 + x^2(a2 + a3 * x^2)) */
    fltY = MLIB_Mac_FLT(psParam->fltA[2], fltX, fltY);

    /* x * (a1 + x^2(a2 + a3 * x^2)) */
    fltY = MLIB_Mul_FLT(fltV, fltY);

    /* (a6 + x^2) */
    fltZ = MLIB_Add_FLT(psParam->fltA[3], fltX);

    /* (a5 + x^2(a6 + x^2)) */
    fltZ = MLIB_Mac_FLT(psParam->fltA[4], fltX, fltZ);

    /* (a4 + x^2(a5 + x^2(a6 + x^2))) */
    fltZ = MLIB_Mac_FLT(psParam->fltA[5], fltX, fltZ);

    /* x * (a1 + x^2(a2 + a3 * x^2)) / (a4 + x^2(a5 + x^2(a6 + x^2))) */
    fltY = MLIB_Div_FLT(fltY, fltZ);

    /* For |fltIn| > 1 subtract the fltY from PI/2 */
    if (bRange_flag == TRUE)
    {
        fltY = MLIB_Sub_FLT(fltW, fltY);    
    }
    else
    {                               
        fltY = MLIB_Sub_FLT(fltY, fltW);        
    }

    if (fltVal < (float_t)0)
    {             
        fltY = MLIB_Neg_FLT(fltY);        
    }
    return(fltY);
}

/***************************************************************************//*!
*
* @brief    The GFLIB_Atan function computes the Atan(x)/pi
*           using piece-wise polynomial approximation.
*
* @param    in  fltVal - The input data value is 32-bit single precision
*                        floating-point value within the full type's range.
*
* @return   The function returns atan(x)/pi.The output data value
*           is in range (-0.5, 0.5), which corresponds to the angle
*           in the range of (-pi/2,pi/2).
*
* @remarks  Output value is:
*
*           Atan(x) =  sign(x) * ((A1*|x| + A2)*|x| + A3)
*
*******************************************************************************/
acc32_t GFLIB_Atan_A32f_FC(float_t fltVal)
{
    acc32_t a32Out; 

    fltVal = GFLIB_Atan_FLT_FC(fltVal, &gfltAtanCoef);
    a32Out = (acc32_t)((float_t)fltVal * ((float_t)32768.0F / (float_t)FLOAT_PI));
    return (a32Out);
}
