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
#include "GFLIB_Lut_FLT.h"

/***************************************************************************//*!
*
* @brief    Look-up table algorithm using linear interpolation
*
* @param    ptr  float_t *psParam - Pointer to the table size, min, max values
* @param    in   float_t *pfltTable- Pointer to the table values
*                float_t fltX - Argument in float_t
*
* @return   This function returns - float_t value 
*       
* @remarks  This function calculates the number from the table using interpolation
*           of two values in the table.
*
*******************************************************************************/
float_t GFLIB_Lut1D_FLT_FC(float_t fltX, 
                           const float_t *pfltTable,
                           const GFLIB_LUT1D_T_FLT *psParam)
{
    register float_t fltMinTab;
    register float_t fltTemp;    
    register float_t fltTemp1; 
    register float_t fltTemp2; 
    register const float_t *pfltTemp;    
    register int16_t i16WIntvl;  
    
    /* store more times read quantities to registers */
    fltMinTab = psParam->fltMin;
    
    /* the input abscissa determination */
    if (fltX >= psParam->fltMax)
        return (*(pfltTable+psParam->u16TableSize-1));
    else 
        if (fltX <= fltMinTab) 
        {            
            return (*(pfltTable));
        }
        else 
        {
            /* Interval and table points for interpolation calculation */
            fltTemp  = MLIB_Sub_FLT(fltX,fltMinTab);
            fltTemp  = (MLIB_Mul_FLT(fltTemp,psParam->fltIntInv));
            i16WIntvl= (uint16_t)fltTemp;
            fltTemp  = MLIB_Sub_FLT(fltTemp, (float_t)i16WIntvl);
            pfltTemp = (float*)pfltTable+i16WIntvl;
      
            /* Read interpolated data */
            fltTemp1 = (*pfltTemp);       /* the first interpolation value */
            fltTemp2 = (*(pfltTemp+1));   /* the second interpolation value */ 
            
            /* Perform the actual interpolation with 16-bit accuracy */
            fltTemp1 = MLIB_Mac_FLT(fltTemp1, fltTemp1, MLIB_Neg_FLT(fltTemp)); 
            fltTemp1 = MLIB_Mac_FLT(fltTemp1, fltTemp2, fltTemp);             
            return (fltTemp1);        
        }
}  

