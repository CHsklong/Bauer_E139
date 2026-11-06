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
* @brief  Tangent 
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_Tan_FLT.h"

/*******************************************************************************
* Global variables
*******************************************************************************/  
GFLIB_CONST GFLIB_TAN_T_FLT fltgflibTanCoef = {{
    211.8493696641210000F, -12.5288887278448000F, 
    269.7350131214120000F, -71.4145309347748000F}};
                                          
/***************************************************************************//*!
*
* @brief    The GFLIB_Tan function computes the tan(x) using    
*           piece-wise polynomial approximation.
*
* @param  ptr  *psParam - Pointer to the table
*
* @param  in   fltAngle  -The input data value is in the range float_t, which
*                         corresponds to the angle in the range of [-pi,pi).   
*                       
* @return The function returns tan(x).
*       
* @remarks  The GLIB_Tan function computes the tan(x) using     
*           piece-wise polynomial approximation.                                           
*
*           The approximation of the tangent function over the entire defined
*           range of input angles is simplified to an approximation for angles in 
*           the range [0, pi/4), and then, depending on the input angle, the result 
*           will be negated. Interval [0, pi/4) is further divided into eight equally 
*           spaced sub intervals, and polynomial approximation is done for each 
*           interval respectively. 
*
*           Tan(x) = ((A1*x + A2)*x + A3)*x + A4
*
****************************************************************************/
float_t GFLIB_Tan_FLT_FC(float_t fltAngle,GFLIB_CONST GFLIB_TAN_T_FLT *const psParam)
{
    float_t fltY;
    float_t fltZ;
    float_t fltW;
    float_t fltP1;
    float_t fltP2;
    float_t fltP1Tmp;

    /* Absolute value */
    fltY = fltAngle;
    if (fltAngle < 0.0F)
    {
        fltY = MLIB_Neg_FLT(fltY);
    }

    /* fltIn * 4 / PI */
    fltY = MLIB_Mul_FLT(fltY, 1.2732395447351600F);  

    if (fltY >= 3.0F)
    {
        /* For interval <|PI|; |3 * PI / 4|) transformation to interval (0; |PI / 4|> */
        fltY = MLIB_Sub_FLT(4.0F, fltY);
        
        /* fltY ^ 2 */
        fltZ = MLIB_Mul_FLT(fltY, fltY);
        
        /*------------------------------------------------------------------------
        *      x(a0 + a1 * x^2)
        * - ---------------------
        *    a2 + x^2(a3 + x^2)
        *------------------------------------------------------------------------*/
        
        /* (a0 + a1 * x^2) */
        fltP1 = MLIB_Mac_FLT(psParam->fltA[0],psParam->fltA[1], fltZ);
        
        /*  x(a0 + a1 * x^2) */
        fltP1 = MLIB_Mul_FLT(fltY, fltP1);
        
        /* (a3 + x^2) */
        fltP2 = MLIB_Add_FLT(psParam->fltA[3], fltZ);
        
        /* a2 + x^2(a3 + x^2) */
        fltP2 = MLIB_Mac_FLT(psParam->fltA[2], fltZ, fltP2);
        
        /*  (-x(a0 + a1 * x^2))/(a2 + x^2(a3 + x^2)) */
        fltP1Tmp = MLIB_Neg_FLT(fltP1);
        fltW = MLIB_Div_FLT(fltP1Tmp, fltP2);
    }
    else
    {
        if(fltY >= 1.0F)
        {
            /* For interval <|3 * PI / 4|; |PI / 4|) transformation to interval 1/((- PI / 4 ; PI / 4))
               FLOAT_PI_CORRECTION constant add to compensate the error in 
               enumeration of PI in single precision float */
            fltY = MLIB_Sub_FLT(MLIB_Sub_FLT(2.0F, fltY), (float_t) 8.74204136E-8); 
            /* fltY ^ 2 */
            fltZ = MLIB_Mul_FLT(fltY, fltY);
            /*------------------------------------------------------------------------
            *   a2 + x^2(a3 + x^2)
            *  --------------------
            *    x(a0 + a1 * x^2)
            *------------------------------------------------------------------------*/
            /* (a0 + a1 * x^2) */
            fltP1 = MLIB_Mac_FLT(psParam->fltA[0], psParam->fltA[1], fltZ);
            
            /*  x(a0 + a1 * x^2) */
            fltP1 = MLIB_Mul_FLT(fltY, fltP1);
            
            /* (a3 + x^2) */
            fltP2 = MLIB_Add_FLT(psParam->fltA[3], fltZ);
            
            /* a2 + x^2(a3 + x^2) */
            fltP2 = MLIB_Mac_FLT(psParam->fltA[2], fltZ, fltP2);
            
            /* Protection of divide by zero */
            if (fltP1 < 0.1F)
            {
                if((fltP1 >= 0.0F))
                {
                    fltP1 = (float_t)(1E-9);
                }
                else
                {
                    if(fltP1 >  -0.1F)
                    {
                        fltP1 = (float_t)(-1E-9);
                    }
                }
            }
            /*  (a2 + x^2(a3 + x^2))/(x(a0 + a1 * x^2)) */
            /* MLIB_Div not used due to efficiency reason, the protection of division by zero done in previous step */
            fltW = (fltP2 / fltP1);
        }
        else
        {
            /* Interval <0; PI / 4> */
            /* fltY ^ 2 */
            fltZ = MLIB_Mul_FLT(fltY, fltY);
            /*------------------------------------------------------------------------
            *    x(a0 + a1 * x^2)
            *  ---------------------
            *   a2 + x^2(a3 + x^2)
            *------------------------------------------------------------------------*/
            /* (a0 + a1 * x^2) */
            fltP1 = MLIB_Mac_FLT(psParam->fltA[0], psParam->fltA[1], fltZ);
            
            /*  x(a0 + a1 * x^2) */
            fltP1 = MLIB_Mul_FLT(fltY, fltP1);
            
            /* (a3 + x^2) */
            fltP2 = MLIB_Add_FLT(psParam->fltA[3], fltZ);
            
            /* a2 + x^2(a3 + x^2) */
            fltP2 = MLIB_Mac_FLT(psParam->fltA[2], fltZ, fltP2);
            
            /*  (x(a0 + a1 * x^2))/(a2 + x^2(a3 + x^2)) */
            /* MLIB_Div not used due to efficiency reason, the protection of division by zero eliminated by previous conditions */
            fltW = MLIB_Div_FLT(fltP1,fltP2);
        }
    }

    /* Add the sign for x < 0 */
    if (fltAngle < 0.0F)
    {
        fltW = MLIB_Neg_FLT(fltW);
    }
    return(fltW);
  }
  
/***************************************************************************//*!
*
* @brief      The GFLIB_Tan function computes the tan(x) using
*             piece-wise polynomial approximation.
*
* @param  in  a32AngleExt - The input data value is in the range acc32_t,
*                           fractional part of which corresponds to [-pi; pi)
*
* @return      The function returns tan(x).
*
* @remarks     The GLIB_Tan function computes the tan(x),
*              x = fractional part of a32AngleExt * pi / 2^15
*
****************************************************************************/
float_t GFLIB_Tan_FLTa_FC(acc32_t a32AngleExt)
{
    float_t fltOut;

    a32AngleExt = (acc32_t)((frac16_t)a32AngleExt);   /* Clears integer part of acc32_t input */
    fltOut = (float_t)((frac16_t)a32AngleExt * 
             ((float_t)FLOAT_PI/(float_t)32768.0F));    /* Converts fractional to float */
    return(GFLIB_Tan_FLT_FC(fltOut,&fltgflibTanCoef));
}
