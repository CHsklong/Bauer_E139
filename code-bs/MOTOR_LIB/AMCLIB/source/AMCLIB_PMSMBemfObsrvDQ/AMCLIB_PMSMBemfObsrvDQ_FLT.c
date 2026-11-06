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
* @brief  Algorithm of float PMSM Back Electromotive Force observer in rotating 
*         reference frame      
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "AMCLIB_PMSMBemfObsrvDQ_FLT.h"

/***************************************************************************//*!
*
* @brief    Float BEMF observer using D, Q components to calculate error BEMF gamma/delta.
*
* @param    ptr - GMCLIB_2COOR_DQ_T_FLT *psIDQ - pointer to the D, Q current                      
*               - GMCLIB_2COOR_DQ_T_FLT *psUDQ - pointer to the D, Q voltage
*               - AMCLIB_BEMF_OBSRV_DQ_T_FLT *psCtrl - pointer to the parameters of the observer
*
* @return   float_t - error between the real and estimated position
*       
* @remarks  The function calculates the algorithm of back electromotive force observer
*           in rotating reference frame by following equations:
*
*    fltTmpEObsrvD = psCtrl_sCtrl_fltID_1 + 
*                   (psCtrl_sCtrl_fltIGain + psCtrl_sCtrl_fltPGain) * (psUDQ_fltD - psCtrl_sIObsrv_fltD) 
*    fltTmpEObsrvQ = psCtrl_sCtrl_fltIQ_1 +
*                   (psCtrl_sCtrl_fltIGain + psCtrl_sCtrl_fltPGain) * (psUDQ_fltQ - psCtrl_sIObsrv_fltQ)
*
*    fltTmpIObsrvD = fltIGain * psCtrl_sIObsrv_fltD + fltUGain * psUDQ_fltD + 
*                    psIDQ_fltQ * fltSpeed * fltWIGain + fltEGain * fltTmpEObsrvD         
*    fltTmpIObsrvQ = fltIGain * psCtrl_sIObsrv_fltQ + fltUGain * psUDQ_fltQ - 
*                    psIDQ_fltD * fltSpeed * fltWIGain + fltEGain * fltTmpEObsrvQ
*
*                      | Atan(-fltTmpEObsrvD /  fltTmpEObsrvQ)   if   fltSpeed < 0
*    psCtrl_fltError = |                                               
*                      | Atan( fltTmpEObsrvD / -fltTmpEObsrvQ)   else
*
****************************************************************************/
acc32_t AMCLIB_PMSMBemfObsrvDQ_A32fff_FC(const GMCLIB_2COOR_DQ_T_FLT *psIDQ,
                                         const GMCLIB_2COOR_DQ_T_FLT *psUDQ,
                                         float_t fltSpeed, 
                                         AMCLIB_BEMF_OBSRV_DQ_T_FLT *psCtrl)
{    
    register float_t fltTmpIObsrvD;
    register float_t fltTmpIObsrvQ;  
    register float_t fltTemp1;  
    register float_t fltTmpEObsrvD;
    register float_t fltTmpEObsrvQ;
    register float_t fltCtrlIGain;
    register float_t fltCtrlPGain;
    float_t fltWIGain;
    float_t fltIGain;
    float_t fltEGain;
    float_t fltUGain;
    float_t fltCtrlIObsrvD_1;
    float_t fltCtrlIObsrvQ_1;
    float_t fltIObsrvD;
    float_t fltIObsrvQ; 
    float_t fltUInD;     
    float_t fltUInQ;
    float_t fltIDIn; 
    float_t fltIQIn;
    bool_t bErrFlag; 
        
    /* Store more times red quantities to registers */
    fltIObsrvD = psCtrl->sIObsrv.fltD;
    fltIObsrvQ = psCtrl->sIObsrv.fltQ;    
    fltCtrlIObsrvD_1 = psCtrl->sCtrl.fltID_1;
    fltCtrlIObsrvQ_1 = psCtrl->sCtrl.fltIQ_1; 
    fltCtrlPGain = psCtrl->sCtrl.fltPGain;
    fltCtrlIGain = psCtrl->sCtrl.fltIGain;
    fltIGain = psCtrl->fltIGain;    
    fltUGain = psCtrl->fltUGain;        
    fltWIGain= psCtrl->fltWIGain;  
    fltEGain = psCtrl->fltEGain;  
    fltUInD = psUDQ->fltD;     
    fltUInQ = psUDQ->fltQ;
    fltIDIn = psIDQ->fltD;  
    fltIQIn = psIDQ->fltQ;
        
    /* Gamma error calculation using saturation subtraction, D controller */
    fltTemp1 = MLIB_Sub_FLT (fltIDIn, fltIObsrvD);
    
    /* Gamma Controller calculation */  
    fltCtrlIObsrvD_1 = MLIB_Mac_FLT(fltCtrlIObsrvD_1, fltCtrlIGain, fltTemp1);
    fltTmpEObsrvD = MLIB_Mac_FLT(fltCtrlIObsrvD_1, fltCtrlPGain, fltTemp1); 
    
    /* Delta error calculation using saturation subtraction, Q controller*/
    fltTemp1 = MLIB_Sub_FLT (fltIQIn, fltIObsrvQ);
  
    /* Delta Controller calculation */
    fltCtrlIObsrvQ_1 = MLIB_Mac_FLT(fltCtrlIObsrvQ_1, fltCtrlIGain, fltTemp1);
    fltTmpEObsrvQ = MLIB_Mac_FLT(fltCtrlIObsrvQ_1, fltCtrlPGain, fltTemp1);

    /* Gamma observer calculations in float and Delta observer calculation */ 
    /* Multiplication: fltSpeed * psCtrl->fltWIGain */
    fltTemp1 = MLIB_Mul_FLT(fltSpeed, fltWIGain);    
    
    /* Accumulation of first multiplication: psCtrl->fltUGain * psCtrl->sIObsrv.fltD */
    fltTmpIObsrvD = MLIB_Mul_FLT(fltIGain, fltIObsrvD);  
    /* Accumulation of second multiplication: psCtrl->fltUGain * psUDQ->fltD */
    fltTmpIObsrvD = MLIB_Mac_FLT(fltTmpIObsrvD, fltUGain, fltUInD);         
    /* Accumulation of third multiplication: (fltSpeed * psCtrl->fltWIGain) * psCtrl->psIDQ->fltQ */  
    fltTmpIObsrvD = MLIB_Mac_FLT(fltTmpIObsrvD, psIDQ->fltQ, fltTemp1); 
    /* Accumulation of fourth multiplication: psCtrl->fltEGain * fltTmpEObsrvD */
    fltTmpIObsrvD = MLIB_Mac_FLT(fltTmpIObsrvD, fltEGain, fltTmpEObsrvD);         
    
    /* Delta observer calculation in float */  
    /* Accumulation of first multiplication: psCtrl->fltUGain * psCtrl->sIObsrv.fltD */ 
    fltTmpIObsrvQ = MLIB_Mul_FLT(fltIGain, fltIObsrvQ); 
    /* Accumulation of second multiplication:psCtrl->fltUGain * psUDQ->fltD */
    fltTmpIObsrvQ = MLIB_Mac_FLT(fltTmpIObsrvQ, fltUGain, fltUInQ);
    /* Subtraction of third multiplication: (fltSpeed * psCtrl->fltWIGain) * psCtrl->psIDQ->fltD */      
    fltTmpIObsrvQ = MLIB_Msu_FLT(fltTmpIObsrvQ, psIDQ->fltD, fltTemp1);         
    /* Accumulation of fourth multiplication: psCtrl->fltEGain * fltTmpEObsrvQ */
    fltTmpIObsrvQ = MLIB_Mac_FLT(fltTmpIObsrvQ, fltEGain, fltTmpEObsrvQ);          

    /* Store state variables to psCtrl structure */ 
    psCtrl->sIObsrv.fltD = fltTmpIObsrvD;
    psCtrl->sIObsrv.fltQ = fltTmpIObsrvQ;    
    psCtrl->sCtrl.fltID_1= fltCtrlIObsrvD_1;
    psCtrl->sCtrl.fltIQ_1= fltCtrlIObsrvQ_1;
    psCtrl->sEObsrv.fltD = fltTmpEObsrvD;
    psCtrl->sEObsrv.fltQ = fltTmpEObsrvQ;    

    /* BEMF determination according to speed sign */
    if (fltSpeed < 0.0F)
    {   fltTmpEObsrvD = MLIB_Neg_FLT(fltTmpEObsrvD);
    }
    else
    {   fltTmpEObsrvQ = MLIB_Neg_FLT(fltTmpEObsrvQ);
    }
    
    /* Error BEMF calculation in gamma/delta coordinates */
    psCtrl->a32Error = GFLIB_AtanYX_A32f(fltTmpEObsrvD, fltTmpEObsrvQ, &bErrFlag); 
    
    return((acc32_t)psCtrl->a32Error);
}    
