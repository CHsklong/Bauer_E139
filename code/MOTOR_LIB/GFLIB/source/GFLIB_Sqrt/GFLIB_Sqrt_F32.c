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
* @brief  Square root 
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_Sqrt_F32.h"
#include "GFLIB_Sqrt_F16.h"

/***************************************************************************//*!
*
* @brief  Calculates the square root of the argument.
*
* @param    ptr  GFLIB_SQRT_TABLE_T_F32 *psParam - Pointer to the polynomial table 
* @param    in   frac32_t f32Val - Argument in <0;1) in frac32_t
*
* @return This function returns - frac16_t value <0;1)
*		
* @remarks 	This function calculates square root. The argument must be in the
*			range [0;1]. The algorithm calculates the raw result using a polynomial
*			of the 4th order using 3 intervals. Then the raw result is iterated
*			in 3 steps to get a more precise result.
*			If the value is negative, it is negated and then calculated as it was
*			positive. But the result keeps positive.
*
****************************************************************************/
frac16_t GFLIB_Sqrt_F16l_FC(frac32_t f32Val,
                            const GFLIB_SQRT_TABLE_T_F32 *psParam)
{
    GFLIB_SQRT_ROW_T_F32 const *psSqrtRow;   
    register frac32_t f32X;    
    register frac32_t f32Temp; 
    register frac32_t f32Temp1;  
    register frac32_t f32Temp2;      
    register frac32_t f32TempRes;     
    register frac32_t f32Temp1Res; 
    register frac32_t f32Temp2Res;    
    register uint16_t u16ValNormMSB;    
    register uint16_t u16ClbNum;
    register uint16_t u16ClbNumHalf; 
    
    if(f32Val > 0)    
    {   
        /* Normalization */   
        u16ClbNum = (MLIB_Clb_U16l (f32Val)) & 0x1E ;  /* Keeps the pair number of leading bits */      
        u16ClbNumHalf = u16ClbNum >> 1;                /* Store the half of pair number of leading bits */
        f32X = f32Val << u16ClbNum;                    /* Normalize the number */ 
        
        /* Separating two MSB bits*/
        u16ValNormMSB = f32X >> (32-2-1);              /* Right shift normed value to keep two MSB bits */
        u16ValNormMSB--;                               /* The result is 1,2 or 3, so it subtracts 1 */

        /* Pointer to the row of table according two MSB bits */   
        f32Temp = (uint32_t)psParam +12+(u16ValNormMSB*20);  /* skip sLineX meambers (12 = 4 x 3), 20 is row lenght */    
        psSqrtRow = (GFLIB_SQRT_ROW_T_F32*)f32Temp;
            
        /* Polynom calculation selected by two MSB bits */
        /* x = (flX - XkOffset) << 3 */
        /* flResult = YkOffset + (Coef5 + x * (Coef4 + x * (Coef3 + x * (Coef2 + x * Coef1))) >> NYScl  */
        /*				or 			*/	
        /* flResult = YkOffset + (Coef5 + Coef4 * x + Coef3 * x ^ 2 + Coef2 * x ^ 3 + Coef1 * x ^ 4) >> NYScl */
   
        f32X    = ((frac16_t)(f32X>>16) - psSqrtRow->f16XkOffset) << 3;
        f32Temp = MLIB_Mac_F16 (psSqrtRow->f16PolyCoef[1], f32X, psSqrtRow->f16PolyCoef[0]);    
        f32Temp = MLIB_Mac_F16 (psSqrtRow->f16PolyCoef[2], f32X, f32Temp);     
        f32Temp = MLIB_Mac_F16 (psSqrtRow->f16PolyCoef[3], f32X, f32Temp);        
        f32Temp = MLIB_Mac_F16 (psSqrtRow->f16PolyCoef[4], f32X, f32Temp); 
        f32Temp = ((f32Temp << (16 - psSqrtRow->f16NYScl)) + psSqrtRow->f32YkOffset)<<1;
        f32Temp = (f32Temp >>(16+u16ClbNumHalf));   /* Right shift back from normalized value - 16 + shift to frac16_t */          
        
        /* Iteration */  
        f32TempRes  = MLIB_Abs_F32(MLIB_Mac_F32lss (f32Val, -f32Temp,  f32Temp));  
        f32Temp1 = f32Temp + 1;
        f32Temp1Res = MLIB_Abs_F32(MLIB_Mac_F32lss (f32Val, -f32Temp1, f32Temp1));   
        f32Temp2 = f32Temp - 1 ;
        f32Temp2Res = MLIB_Abs_F32(MLIB_Mac_F32lss (f32Val, -f32Temp2, f32Temp2));        
      
        f32X = (f32TempRes < f32Temp1Res) ? ((f32TempRes  < f32Temp2Res) ?  f32Temp  : f32Temp2) 
                                          : ((f32Temp1Res < f32Temp2Res) ?  f32Temp1 : f32Temp2); 
        return((frac16_t) (f32X > 32767) ? 32767 : f32X);           
    }
    else
    {
        return((frac16_t)0);
    }
} 
	                                           
