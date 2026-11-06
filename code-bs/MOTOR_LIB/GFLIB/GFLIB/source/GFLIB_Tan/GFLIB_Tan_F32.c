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
#include "GFLIB_Tan_F32.h"

/*******************************************************************************
* Global variables
*******************************************************************************/  
GFLIB_CONST GFLIB_TAN_T_F32 gsTanCoef = {{
    {{ 688168,   1024000,   211337216,   105498624}},  /* Coef. [0;pi/32) */
    {{ 737280,   3145728,   215465984,   318550016}},  /* Coef. [pi/32;pi*2/32) */
    {{ 851968,   5521408,   224055296,   537917440}},  /* Coef. [pi*2/32;pi*3/32) */
    {{1064960,   8364032,   237821952,   768380928}},  /* Coef. [pi*3/32;pi*4/32) */
    {{1392640,  12001280,   257990656,  1015683072}},  /* Coef. [pi*4/32;pi*5/32) */
    {{1916928,  16900096,   286568448,  1287151616}},  /* Coef. [pi*5/32;pi*6/32) */
    {{2785280,  23855104,   326795264,  1592680448}},  /* Coef. [pi*6/32;pi*7/32) */
    {{4292608,  34275328,   384016384,  1946363904}}   /* Coef. [pi*7/32;pi*8/32) */
}};

/***************************************************************************//*!
*
* @brief    The GFLIB_Tan function computes the tan(pi*x) using    
*           piece-wise polynomial approximation.
*
* @param    ptr  *psParam - Pointer to the table
*
* @param    in   f16Angle - The input data value is in the range of [-1,1), which
*                           corresponds to the angle in the range of [-pi,pi).                         
*
* @return   The function returns tan(pi*x).
*       
* @remarks  The GFLIB_Tan function computes the tan(pi*x) using     
*           piece-wise polynomial approximation. All tangent values
*           falling beyond [-1, 1), are truncated to -1 and 1      
*           respectively.calculates tan of the input fractional
*           argument                                               
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
#if defined(__GNUC__)
    #pragma GCC optimize ("O0")
#endif
frac16_t GFLIB_Tan_F16_FC(frac16_t f16Angle,GFLIB_CONST GFLIB_TAN_T_F32 *const psParam)
{
    frac32_t f32Out;
    frac16_t f16Out;
    frac32_t f32Sign;
    int32_t  s32nSector;
    frac32_t f32Angle;

    f32Angle = MLIB_Conv_F32s(f16Angle);
    /* Sign is saved in the f16Sign */

    f32Angle = MLIB_ShR_F32(MLIB_ShL_F32(f32Angle,(uint32_t)1),(uint32_t)1);
    f32Sign = MLIB_ShR_F32(f32Angle, (uint32_t)31);
    f32Angle = MLIB_Sub_F32((f32Angle ^ f32Sign), f32Sign);
    if(f32Angle > ((INT32_MAX/4)-(0x800)))
    { /* Select undefined range where is function |tan(x)| higher then 1 */
        f32Out = INT32_MAX;
    }
    else
    { 
        f32Angle = MLIB_Add_F32(f32Angle,0x800);
         /* Polynomial sector choose */
        s32nSector =  MLIB_ShR_F32(f32Angle, (uint16_t)26);
        
        /* Input value is scaled on the range FRAC <0,0.5) */
        f32Angle = MLIB_Sub_F32(f32Angle, MLIB_ShL_F32(s32nSector, (uint16_t)26));
        f32Angle = MLIB_ShL_F32(f32Angle, (uint16_t)5);
        
        /* Input value is scaled on the range FRAC <-0.5,0.5) */
        f32Angle = MLIB_Sub_F32(f32Angle, INT32_MAX/2);
        
        /* (a1 * x) + a2 */
        f32Out = MLIB_Mac_F32(psParam->GFLIB_TAN_SECTOR_F32[s32nSector].f32A[1], 
                              psParam->GFLIB_TAN_SECTOR_F32[s32nSector].f32A[0], f32Angle);
        
        /* ((a1 * x) + a2) * x + a3 */
        f32Out = MLIB_Mac_F32(psParam->GFLIB_TAN_SECTOR_F32[s32nSector].f32A[2], f32Out, f32Angle);
        
        /* (((a1 * x) + a2) * x + a3) * x + a4 */
        f32Out = MLIB_Mac_F32(psParam->GFLIB_TAN_SECTOR_F32[s32nSector].f32A[3], f32Out, f32Angle);
    }
    f16Out = MLIB_Conv_F16l(f32Out);
    f16Out = (f32Sign >= 0) ? f16Out : ((~f16Out)+1);
    return(f16Out);
}
#if defined(__GNUC__)
    #pragma GCC optimize ("O3")
#endif
