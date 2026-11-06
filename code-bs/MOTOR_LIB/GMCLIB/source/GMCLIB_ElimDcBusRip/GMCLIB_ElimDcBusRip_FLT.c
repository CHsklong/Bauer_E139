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
* @brief  Elimination of the DC-Bus voltage ripple for FOC
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GMCLIB_ElimDcBusRip_FLT.h"

/***************************************************************************//*!
*
* @brief  The function is used for elimination of the DC-Bus voltage ripple 
*         for the Field Oriented Control.
*
* @param  ptr - psUAlBe - Direct(alpha) and quadrature(beta) float component of the 
*               stator voltage vector in the stationary reference frame.
*             - psUAlBeComp - Direct(alpha) and quadrature(beta) float component 
*               of the stator voltage vector in the stationary reference frame
*               after DC-Bus ripple elimination.
* @param  in  - fltUDCBus - Actual effective value of the DC-Bus voltage.
*
* @return Function reads, recalculate and fills variables alpha,
*         beta in the data structure GMCLIB_2COOR_ALBE_T_FLT.
*
* @remarks: The GMCLIB_ElimDcBusRipFOC function compensates an amplitude 
*           of the direct-a and the quadrature- component of the 
*           stator-reference voltage vector in the fractional arithmetic by the
*           formula shown in the following equations:
*
*         for Udcbus != 0
*         UaReq =  1.0                        Ua >= 0 && abs(Ua) >= Udcbus / sqrt(3)
*         UaReq = -1.0                        Ua <  0 && abs(Ua) <  Udcbus / sqrt(3)   
*         UaReq =  sqrt(3) * Ua / Udcbus       else    
*                                                     
*         UbReq =  1.0                        Ub >= 0 && abs(Ub) >= Udcbus / sqrt(3)  
*         UbReq = -1.0                        Ub <  0 && abs(Ub) <  Udcbus / sqrt(3)   
*         UbReq = sqrt(3) * Ub / Udcbus        else    
*                                                     
*         for Udcbus = 0                              
*         UaReq =  1.0                        Ua > 0 && Udcbus = 0 
*         UaReq = -1.0                        Ua < 0 && Udcbus = 0  
*         UaReq =  0.0                        Ua = 0 && Udcbus = 0 
*                                                    
*         UbReq =  1.0                        Ub > 0 && Udcbus = 0 
*         UbReq = -1.0                        Ub < 0 && Udcbus = 0  
*         UbReq =  0 .0                       Ub = 0 && Udcbus = 0 
*
*
*******************************************************************************/
void GMCLIB_ElimDcBusRipFOC_F16ff_FC(float_t fltUDCBus,
                                     const GMCLIB_2COOR_ALBE_T_FLT *psUAlBe,
                                     GMCLIB_2COOR_ALBE_T_F16 *psUAlBeComp)
{  
    register float_t fltAlpha;
    register float_t fltBeta ; 
    register float_t fltSqrt3DivDCBus;
    
    /* Calculate reciprocal DCBus voltage value to save one division */ 
    fltSqrt3DivDCBus = MLIB_Div_FLT (GMCLIB_SQRT3_FLT, fltUDCBus); 
      
    fltAlpha = MLIB_Mul_FLT( psUAlBe->fltAlpha, fltSqrt3DivDCBus);  
    fltBeta  = MLIB_Mul_FLT( psUAlBe->fltBeta , fltSqrt3DivDCBus);        
    
    psUAlBeComp->f16Alpha = MLIB_Conv_F16f(fltAlpha); 
    psUAlBeComp->f16Beta  = MLIB_Conv_F16f(fltBeta);
}   

/***************************************************************************//*!
*
* @brief  The function is used for elimination of the DC-Bus voltage ripple
*         in general cases where the phase voltage is scale as measured
*         dc bus voltage divided by the modulation index.
*
* @param  in  - fltUDCBus - Actual effective value of the DC-Bus voltage.
*             - fltIdxMod - Modulation index, must be positive.
* @param  ptr - psUAlBe - IN - Direct(alpha) and quadrature(beta) float component of the 
*               stator voltage vector in the stationary reference frame.
*             - psUAlBeComp - OUT - Direct(alpha) and quadrature(beta) float component 
*               of the stator voltage vector in the stationary reference frame 
*               after DC-Bus ripple elimination.
*
* @return Function reads, recalculate and fills variables alpha, beta in the
*         data structure GMCLIB_2COOR_ALBE_T_FLT.
*
* @remarks: The GMCLIB_ElimDcBusRipFOC function compensates an amplitude 
*           of the direct-a and the quadrature- component of the 
*           stator-reference voltage vector in the fractional arithmetic by the
*           formula shown in the following equations:
*
*         Equations:
*         for ModIndex > 0 
*         UaReq = 1.0                         Ua > 0 || abs(Ua) >= Udcbus / ModIndex 
*         UaReq = -1.0                        Ua < 0 || abs(Ua) <  Udcbus / ModIndex   
*         UaReq =  ModIndex  / Udcbus * Ua     else                          
*                                                                            
*         UbReq = 1.0                         Ub > 0 || abs(Ub) >= Udcbus / ModIndex  
*         UbReq = -1.0                        Ub < 0 || abs(Ub) <  Udcbus / ModIndex   
*         UbReq = IdxMod / Udcbus * Ub         else   
*
*         for ModIndex = 0 
*         UaReq =  1.0                        Ua >  0 || Udcbus = 0 
*         UaReq = -1.0                        Ua <  0 || Udcbus = 0  
*         UaReq =  0                          Ua =  0 || Udcbus = 0 
*
*         UbReq =  1.0                        Ub >  0 || Udcbus = 0 
*         UbReq = -1.0                        Ub <  0 || Udcbus = 0  
*         UbReq =  0                          Ub =  0 || Udcbus = 0 
*
*******************************************************************************/
void GMCLIB_ElimDcBusRip_F16fff_FC(float_t fltUDCBus,
                                   float_t fltIdxMod,
                                   const GMCLIB_2COOR_ALBE_T_FLT *psUAlBe,
                                   GMCLIB_2COOR_ALBE_T_F16 *psUAlBeComp)
{ 
    register float_t fltAlpha;
    register float_t fltBeta ; 
    register float_t fltSqrt3DivDCBus;
    
    /* if DCBus is zero return zero alpha and beta components */
    if((fltUDCBus == (float_t)0.0F)&&(fltIdxMod == (float_t)0.0F))
    {
        fltSqrt3DivDCBus = 0;
    }
    else
    {
        /* Calculate reciprocal DCBus voltage value to save one division */
        fltSqrt3DivDCBus = MLIB_Div_FLT (fltIdxMod, fltUDCBus);
    }
      
    fltAlpha = MLIB_Mul_FLT(psUAlBe->fltAlpha, fltSqrt3DivDCBus);  
    fltBeta  = MLIB_Mul_FLT(psUAlBe->fltBeta , fltSqrt3DivDCBus);        
    
    psUAlBeComp->f16Alpha = MLIB_Conv_F16f(fltAlpha); 
    psUAlBeComp->f16Beta  = MLIB_Conv_F16f(fltBeta);
}
