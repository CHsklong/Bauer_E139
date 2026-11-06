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
* @brief  Sine 
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_SinCos_FLT.h"

/*******************************************************************************
* Global variables
*******************************************************************************/  
GFLIB_CONST GFLIB_SIN_T_FLT gfltSinCoef = {{
    -0.000184881402886F, 0.008311899801389F, -0.166655540927577F
}};
GFLIB_CONST GFLIB_COS_T_FLT gfltCosCoef = {{
    -0.000184881402886F, 0.008311899801389F, -0.166655540927577F
}};

/***************************************************************************//*!
* @brief  Calculates the sine of the given argument using Taylor 9th order
*         polynomial.
*
* @param  ptr  float_t *psParam - Pointer to the Taylor polynomial coefficients
* @param  in   float_t fltAngle - Argument in float_t range 
*
* @return This function returns - float_t value 
*       
* @remarks  
*   This function calculates sin(x) using Taylor 9th order polynomial:
*   x=[-pi; pi]:
*       sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*       sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*       sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*       where:
*       d1 =  1
*       d3 = -1 / 3!
*       d5 =  1 / 5!
*       d7 = -1 / 7!
*       d9 =  1 / 9!
*
*******************************************************************************/
float_t GFLIB_Sin_FLT_FC(float_t fltAngle, GFLIB_CONST GFLIB_SIN_T_FLT *const psParam)
{
    float_t fltX;
    float_t fltZ;
    float_t fltW;

    /* Absolute value */
    fltX = MLIB_Abs_FLT(fltAngle);

    if (fltX > 1.57079632679490F)
    {               
        /* Use the PI - |fltIn| for calculation */
        fltX = MLIB_Sub_FLT(FLOAT_PI, fltX);              
        fltX = MLIB_Sub_FLT(fltX, 8.74204136E-8);           
    }

    /* Prepare second power of fltIn */
    fltZ = MLIB_Mul_FLT(fltX, fltX);

    /*--------------------------------------------------------------------------
    * x + x^3*(a3 + x^2(a2 + x^2*a1))
    --------------------------------------------------------------------------*/
    /* (a2 + x^2*a1) */
    fltW = MLIB_Mac_FLT(psParam->fltA[1], fltZ, psParam->fltA[0]);

    /* a3 + x^2(a2 + x^2*a1) */
    fltW = MLIB_Mac_FLT(psParam->fltA[2], fltW, fltZ);

    /* x + x^3*(a3 + x^2(a2 + x^2*a1)) */
    fltW = MLIB_Mul_FLT(fltZ, fltW);
    fltW = MLIB_Mac_FLT(fltX, fltX, fltW);

    /* add sign into the result */
    if (fltAngle < 0.0F)
    {
        fltW = MLIB_Neg_FLT(fltW);
    }
    return(fltW);
}

/***************************************************************************//*!
* @brief  Calculates the sine of the given argument using Taylor 9th order
*         polynomial.
*
* @param  in   acc32 a32AngleExt - Argument in range in acc32_t, fractional part 
*                                  of which corresponds to [-pi; pi)
*
* @return This function returns - float_t value
*
* @remarks
*   This function calculates sin(x),
*   x = fractional part of a32AngleExt * pi / 2^15
*
*******************************************************************************/
float_t GFLIB_Sin_FLTa_FC(acc32_t a32AngleExt)
{
    float_t fltOut;
    
    a32AngleExt = (acc32_t)((frac16_t)a32AngleExt); /* Clears integer part of acc32_t input */
    fltOut = (float_t)((frac16_t)a32AngleExt * 
             ((float_t)FLOAT_PI/(float_t)32768.0F)); /* Converts fractional to float */
    return(GFLIB_Sin_FLT_FC(fltOut,&gfltSinCoef));
}

/***************************************************************************//*!
* @brief  Calculates the cosine of the given argument using Taylor 9th order
*         polynomial.
*
* @param  ptr float_t *psParam - Pointer to the Taylor polynomial coefficients
* @param  in  float_t fltAngle - Argument in float_t range 
*
* @return This function returns - float_t value
*
* @remarks
*   This function calculates cosin(x) using Taylor 9th order polynomial
*
*******************************************************************************/
float_t GFLIB_Cos_FLT_FC(float_t fltAngle, GFLIB_CONST GFLIB_COS_T_FLT * psParam)
{
    float_t fltX;
    float_t fltZ;
    float_t fltW;

    if (fltAngle < 0.0F)
    {
        fltW = 4.37102068E-8;
        fltX = MLIB_Add_FLT(fltAngle,1.57079632679490F );    
        fltX = MLIB_Sub_FLT(fltX, 4.37102068E-8 );   
    }
    else
    {
        fltX = MLIB_Sub_FLT(1.57079632679490F, fltAngle);   
        fltX = MLIB_Sub_FLT(fltX, 4.37102068E-8 );    
    }
    /* Prepare second power of fltIn */
    fltZ = MLIB_Mul_FLT(fltX, fltX);

    /*------------------------------------------------------------------------
    * x + x^3*(a3 + x^2(a2 + x^2*a1))
    *------------------------------------------------------------------------*/
    /* (a2 + x^2*a1) */
    fltW = MLIB_Mac_FLT(psParam->fltA[1],fltZ, psParam->fltA[0]);

    /* a3 + x^2(a2 + x^2*a1) */
    fltW = MLIB_Mac_FLT(psParam->fltA[2], fltW, fltZ);

    /* x + x^3*(a3 + x^2(a2 + x^2*a1)) */
    fltZ = MLIB_Mul_FLT(fltZ, fltW);
    fltW = MLIB_Mac_FLT(fltX, fltX, fltZ);

    return(fltW);
}

/***************************************************************************//*!
* @brief  Calculates the cosine of the given argument using Taylor 9th order
*         polynomial.
*
* @param  in acc32_t fltAngle - Argument in range in acc32_t, fractional part of which
*                               corresponds to [-pi; pi)
*
* @return This function returns - float_t value
*
* @remarks
*   This function calculates cosin(x),
*   x = fractional part of a32AngleExt * pi / 2^15
*
*******************************************************************************/
float_t GFLIB_Cos_FLTa_FC(acc32_t a32AngleExt)
{
    float_t fltOut;
    
    a32AngleExt = (acc32_t)((frac16_t)a32AngleExt);   /* Clears integer part of acc32_t input */
    fltOut = (float_t)((frac16_t)a32AngleExt * 
             ((float_t)FLOAT_PI/(float_t)32768.0F));  /* Converts fractional to float */
    return(GFLIB_Cos_FLT_FC(fltOut,&gfltCosCoef));
}
