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
* @brief  Algorithm of float PMSM Back Electromotive Force observer in stationary 
*         reference frame     
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "AMCLIB_PMSMBemfObsrvAB_FLT.h"

/***************************************************************************//*!
*
* @brief    BEMF observer using alpha, beta components to estimate the angle's
*           sin anAlpha cosine.
*
* @param    ptr   GMCLIB_2COOR_ALBE_T_FLT *psIAlBe  - pointer to the alpha,beta current                      
*                 GMCLIB_2COOR_ALBE_T_FLT *psUAlBe  - pointer to the alpha,beta voltage
*                 AMCLIB_BEMF_OBSRV_AB_T_FLT *psCtrl- pointer to the parameters of the observer
* @param    in    float_t fltSpeed - actual speed
*
* @return   none
*       
* @remarks  The function calculates the algorithm of back electromotive 
*           force observer in stationery reference frame. 
*
*    fltTmpEObsrvAlpha= psCtrl_sCtrl_fltIAlpha_1 + 
*                       (psCtrl_sCtrl_fltIGain + psCtrl_sCtrl_fltPGain) * (psUAlBe_fltAlpha - psCtrl_sIObsrv_fltAlpha) 
*    fltTmpEObsrvBeta = psCtrl_sCtrl_fltIBeta_1 +
*                       (psCtrl_sCtrl_fltIGain + psCtrl_sCtrl_fltPGain) * (psUAlBe_fltBeta - psCtrl_sIObsrv_fltBeta)
*
*    fltTmpEObsrvAlpha= fltTmpEObsrvAlpha / sqrt(fltTmpEObsrvAlpha ^ 2 + fltTmpEObsrvAlpha ^ 2) 
*    fltTmpEObsrvBeta = fltTmpEObsrvBeta / sqrt(fltTmpEObsrvBeta ^ 2 + fltTmpEObsrvBeta ^ 2)
*
*    fltTmpIObsrvAlpha= fltIGain * psCtrl_sIObsrv_fltAlpha + fltUGain * psUAlBe_fltAlpha + 
*                       psIAlBe_fltBeta * fltSpeed * fltWIGain + fltEGain * fltTmpEObsrvAlpha         
*    fltTmpIObsrvBeta = fltIGain * psCtrl_sIObsrv_fltBeta + fltUGain * psUAlBe_fltBeta - 
*                       psIAlBe_fltAlpha * fltSpeed * fltWIGain + fltEGain * fltTmpEObsrvBeta
*
*    fltTmpEObsrvAlpha= -fltTmpEObsrvAlpha      if   fltSpeed < 0                                                
*    fltTmpEObsrvBeta = -fltTmpEObsrvBeta       else
*
****************************************************************************/
void AMCLIB_PMSMBemfObsrvAB_FLT_FC(const GMCLIB_2COOR_ALBE_T_FLT *psIAlBe,
                                   const GMCLIB_2COOR_ALBE_T_FLT *psUAlBe,
                                   float_t fltSpeed, 
                                   AMCLIB_BEMF_OBSRV_AB_T_FLT *psCtrl)
{    
    register float_t fltTmpIObsrvAlpha;
    register float_t fltTmpIObsrvBeta;   
    register float_t fltTemp; 
    register float_t fltTemp1;  
    register float_t fltTmpEObsrvAlpha;
    register float_t fltTmpEObsrvBeta;
    register float_t fltCtrlIGain;
    register float_t fltCtrlPGain;
    float_t fltWIGain;
    float_t fltIGain;
    float_t fltEGain;
    float_t fltUGain;
    float_t fltCtrlIObsrvAlpha_1;
    float_t fltCtrlIObsrvBeta_1;
    float_t fltIObsrvAlpha;
    float_t fltIObsrvBeta; 
    float_t fltUInAlpha;     
    float_t fltUInBeta;
    float_t fltIAlphaIn; 
    float_t fltIBetaIn;
        
    /* Store more times red quantities to registers */
    fltIObsrvAlpha = psCtrl->sIObsrv.fltAlpha;
    fltIObsrvBeta  = psCtrl->sIObsrv.fltBeta;    
    fltCtrlIObsrvAlpha_1 = psCtrl->sCtrl.fltIAlpha_1;
    fltCtrlIObsrvBeta_1  = psCtrl->sCtrl.fltIBeta_1; 
    fltCtrlPGain = psCtrl->sCtrl.fltPGain;
    fltCtrlIGain = psCtrl->sCtrl.fltIGain;
    fltIGain  = psCtrl->fltIGain;    
    fltUGain  = psCtrl->fltUGain;        
    fltWIGain = psCtrl->fltWIGain;  
    fltEGain  = psCtrl->fltEGain;  
    fltUInAlpha  = psUAlBe->fltAlpha;     
    fltUInBeta   = psUAlBe->fltBeta;
    fltIAlphaIn = psIAlBe->fltAlpha;  
    fltIBetaIn  = psIAlBe->fltBeta;
        
    /* Gamma error calculation using saturation subtraction, Alpha controller */
    fltTemp1 = MLIB_Sub_FLT (fltIAlphaIn, fltIObsrvAlpha);
    
    /* Gamma Controller calculation */ 
    fltCtrlIObsrvAlpha_1 = MLIB_Mac_FLT(fltCtrlIObsrvAlpha_1, fltCtrlIGain, fltTemp1);
    fltTmpEObsrvAlpha = MLIB_Mac_FLT(fltCtrlIObsrvAlpha_1, fltCtrlPGain, fltTemp1); 
    
    /* Delta error calculation using saturation subtraction, Q controller*/
    fltTemp1 = MLIB_Sub_FLT (fltIBetaIn, fltIObsrvBeta);
  
    /* Delta Controller calculation */ 
    fltCtrlIObsrvBeta_1 = MLIB_Mac_FLT(fltCtrlIObsrvBeta_1, fltCtrlIGain, fltTemp1);
    fltTmpEObsrvBeta = MLIB_Mac_FLT(fltCtrlIObsrvBeta_1, fltCtrlPGain, fltTemp1);

    /* Gamma observer calculations in float and Delta observer calculation */ 
    /* Multiplication: fltSpeed * psCtrl->fltWIGain */
    fltTemp1 = MLIB_Mul_FLT(fltSpeed, fltWIGain);    
    
    /* Accumulation of first multiplication: psCtrl->fltUGain * psCtrl->sIObsrv.fltAlpha */
    fltTmpIObsrvAlpha = MLIB_Mul_FLT(fltIGain, fltIObsrvAlpha);  
    /* Accumulation of second multiplication: psCtrl->fltUGain * psUAlBe->fltAlpha */
    fltTmpIObsrvAlpha = MLIB_Mac_FLT(fltTmpIObsrvAlpha, fltUGain, fltUInAlpha);         
    /* Accumulation of third multiplication: (fltSpeed * psCtrl->fltWIGain) * psCtrl->psIAlbe->fltBeta */  
    fltTmpIObsrvAlpha = MLIB_Msu_FLT(fltTmpIObsrvAlpha, psIAlBe->fltBeta, fltTemp1); 
    /* Accumulation of fourth multiplication: psCtrl->fltEGain * fltTmpEObsrvAlpha */
    fltTmpIObsrvAlpha = MLIB_Mac_FLT(fltTmpIObsrvAlpha, fltEGain, fltTmpEObsrvAlpha);         
    
    /* Delta observer calculation in float */  
    /* Accumulation of first multiplication: psCtrl->fltUGain * psCtrl->sIObsrv.fltAlpha */ 
    fltTmpIObsrvBeta = MLIB_Mul_FLT(fltIGain, fltIObsrvBeta); 
    /* Accumulation of second multiplication:psCtrl->fltUGain * psUDBeta->fltAlpha */
    fltTmpIObsrvBeta = MLIB_Mac_FLT(fltTmpIObsrvBeta, fltUGain, fltUInBeta);
    /* Subtraction of third multiplication: (fltSpeed * psCtrl->fltWIGain) * psCtrl->psIAlBe->fltAlpha */      
    fltTmpIObsrvBeta = MLIB_Mac_FLT(fltTmpIObsrvBeta, psIAlBe->fltAlpha, fltTemp1);         
    /* Accumulation of fourth multiplication: psCtrl->fltEGain * fltTmpEObsrvBeta */
    fltTmpIObsrvBeta = MLIB_Mac_FLT(fltTmpIObsrvBeta, fltEGain, fltTmpEObsrvBeta);          
    
    /* Store state variables to psCtrl structure */ 
    psCtrl->sEObsrv.fltAlpha = fltTmpEObsrvAlpha;
    psCtrl->sEObsrv.fltBeta  = fltTmpEObsrvBeta;
    
    /* Unity vector calculation */
    /* Module calculation */
    fltTemp = MLIB_Mac4_FLT(fltTmpEObsrvAlpha, fltTmpEObsrvAlpha, fltTmpEObsrvBeta, fltTmpEObsrvBeta);
    /* SQRT and reciprocal value calculations */
    fltTemp = MLIB_Div_FLT(1.0F, GFLIB_Sqrt_FLT(fltTemp));
    
    /* Unity alpha and beta values calculation */ 
    fltTmpEObsrvAlpha = MLIB_Mul_FLT (fltTemp,fltTmpEObsrvAlpha);     
    fltTmpEObsrvBeta  = MLIB_Mul_FLT (fltTemp,fltTmpEObsrvBeta); 
    
    /* BEMF determination according to speed sign */
    if (fltSpeed < 0.0F)
    {   fltTmpEObsrvAlpha = MLIB_Neg_FLT(fltTmpEObsrvAlpha);
    }
    else
    {   fltTmpEObsrvBeta = MLIB_Neg_FLT(fltTmpEObsrvBeta);
    }
    
    /* Store state variables to psCtrl structure */ 
    psCtrl->sIObsrv.fltAlpha = fltTmpIObsrvAlpha;
    psCtrl->sIObsrv.fltBeta  = fltTmpIObsrvBeta;    
    psCtrl->sCtrl.fltIAlpha_1= fltCtrlIObsrvAlpha_1;
    psCtrl->sCtrl.fltIBeta_1 = fltCtrlIObsrvBeta_1;
    psCtrl->sUnityVctr.fltSin = fltTmpEObsrvAlpha;
    psCtrl->sUnityVctr.fltCos = fltTmpEObsrvBeta;     
}    
