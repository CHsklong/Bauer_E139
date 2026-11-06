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
* @brief  Look-up table algorithm using linear interpolation
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_Lut_F32.h"

/***************************************************************************//*!
*
* @brief    Look-up table algorithm using linear interpolation
*
* @param    ptr  frac32_t *pf32Table - Pointer to the table values
* @param    in   frac32_t f32X - Argument in <-1;1) in frac32_t
*                uint16_t u16TableSize - Size of the look-up table 
*                                        in bit shifts, 3 for 8 values
*
* @return   This function returns - frac32_t value <-1;1)
*       
* @remarks  This function calculates the number from the table using interpolation
*           of two values in the table.
*           The table size must be based on 2^x+1, i.e. 257 values contains
*           the size 8.
*           The min value for u16TableSize is 1.
*
*******************************************************************************/
frac32_t GFLIB_Lut1D_F32_FC(frac32_t f32X, const frac32_t *pf32Table, uint16_t u16TableSize)
{
    register frac32_t f32Y;
    register frac32_t f32DX;
    register frac32_t f32Y1;
    register frac32_t f32Y2;
    register int32_t i32TablePos;  
    register const frac32_t *pf32Intvl;    
    register int16_t i16Middle;

    u16TableSize--;
    i16Middle = 1 << (u16TableSize);

    /* Extract the fractional part of the input abscissa */
    f32DX = MLIB_ShL_F32(f32X, u16TableSize);
    f32DX &= 0x7FFFFFFF;  
    i32TablePos = (int32_t)MLIB_ShR_F32(f32X, (uint16_t)((uint16_t)31U - u16TableSize));    
    pf32Intvl = (frac32_t*)pf32Table + i32TablePos + i16Middle;  

    /* Read interpolated data */
    f32Y1 = (*pf32Intvl);        /* the first interpolation value */
    f32Y2 = (*(pf32Intvl + 1));  /* the second interpolation value */

    /* Perform the actual interpolation with 32-bit accuracy */
    f32Y = MLIB_MacRnd_F32(MLIB_MsuRnd_F32(f32Y1, f32DX, f32Y1), f32DX, f32Y2);
    
    return (f32Y);
}
