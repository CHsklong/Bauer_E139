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
* @brief ACIM FLux Observer 
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "AMCLIB_ACIMRotFluxObsrv_FLT.h"

/***************************************************************************//*!
*
* @brief    ACIM flux observer to calculate rotor fluxes for rotor electrical position (angle)
*
* @param    ptr   GMCLIB_2COOR_ALBE_T_FLT *psISAlBe - pointer to stator current in Alpha, Beta coordinates                      
*                 GMCLIB_2COOR_ALBE_T_FLT *psUSAlBe - pointer to stator voltage in Alpha, Beta coordinates  
*                 AMCLIB_ACIM_ROT_FLUX_OBSRV_T_FLT *psCtrl - pointer to RFO parameter structure 
*
* @return   N/A 
*       
* @remarks  The function calculates the rotor fluxes using current model in rotating synchronous reference frame 
*           (direct and quadrature coordinates) and rotor fluxes using voltage model in stationary reference frame 
*           (alpha, beta coordinates). The task of the function is to balance both rotor fluxes using PI controllers.            
*
*           Current(Rotor) Model equations:
*               Is =>> Park(RotPsiAngle) =>> mult by (PsiRotModelD(k-1) + B1*IsD) ==> ParkInv(RotPsiAngle) ==> PsiRotModel(k)                                          
*
*           Voltage(Stator) Model equations:
*               e(k) = PsiStatModel(k) - PsiRotModel(k)  
*               Ucomp = out(k-1) + e(k)*IGain + (e(k)-e(k-1))*PGain
*               PsiStatModel(k) = A1*PsiStatModel(k-1) - A2*[(Us-Ucomp)-Rs*Is]
*               PsiRotModel(k) = KrInv*PsiStatModel(k) - KrLsTotLeak*Is
*               RotPsiAngle = atanXY ( PsiRotModelBeta(k)/PsiRotModelAlpha(k) )
*               Torque = PsiRotModelAlpha * ISBeta - PsiRotModelBeta * ISAlpha 
*
****************************************************************************/
void AMCLIB_ACIMRotFluxObsrv_FLT_FC(const GMCLIB_2COOR_ALBE_T_FLT *psISAlBe,
                                    const GMCLIB_2COOR_ALBE_T_FLT *psUSAlBe,                                               
                                    AMCLIB_ACIM_ROT_FLUX_OBSRV_T_FLT *psCtrl)
{
    GMCLIB_2COOR_ALBE_T_FLT sPsiRAlBeTemp;     
    GMCLIB_2COOR_DQ_T_FLT sISDQ;      
    GMCLIB_2COOR_SINCOS_T_FLT sRotPos;
    register float_t fltTempAlpha;
    register float_t fltTempBeta ;
    register float_t sCompErrAlpha;
    register float_t sCompErrBeta ;    
    register float_t fltPsiRAlpha; 
    register float_t fltPsiRBeta;  
    register float_t fltISAlpha;
    register float_t fltISBeta ;    
    register float_t fltPGain;
    register float_t fltIGain; 
    register float_t fltTemp1;    
    register float_t fltTemp; 
    float_t fltPsiSAlpha; 
    float_t fltPsiSBeta;    
    float_t fltKrInvGain;
    float_t fltKrLsTotLeakGain;     
    float_t fltPsiSA2Gain;
    float_t fltRsEst;      
    float_t fltPsiSA1Gain;
    acc32_t a32RotFluxPos;
    bool_t bErrFlag; 
     
    /* Store more times red quantities to registers */
    fltISAlpha         = psISAlBe->fltAlpha;
    fltISBeta          = psISAlBe->fltBeta ;
    fltPsiSAlpha       = psCtrl->sPsiStatSAlBe.fltAlpha;
    fltPsiSBeta        = psCtrl->sPsiStatSAlBe.fltBeta;
    fltPsiSA2Gain      = psCtrl->fltPsiSA2Gain;
    fltRsEst           = psCtrl->fltRsEst;      
    fltKrInvGain       = psCtrl->fltKrInvGain;    
    fltKrLsTotLeakGain = psCtrl->fltKrLsTotLeakGain ;  
    fltPsiSA1Gain      = psCtrl->fltPsiSA1Gain;   
    fltPGain           = psCtrl->sCtrl.fltPGain;
    fltIGain           = psCtrl->sCtrl.fltIGain;     
    a32RotFluxPos      = psCtrl->a32RotFluxPos;
    
    /* - Current(Rotor) Model -  ------- begin ----------------------------------------- */ 
    /* Calculate sin&cos of rotor position angle */
    sRotPos.fltSin = GFLIB_Sin_FLTa(a32RotFluxPos);
    sRotPos.fltCos = GFLIB_Cos_FLTa(a32RotFluxPos);
    
    /* Transform stator current into rotor rotating reference frame */
    GMCLIB_Park_FLT (psISAlBe, &sRotPos, &sISDQ);   
    
    /* Estimated Rotor flux in DQ reference frame */
    fltTemp = MLIB_Mac_FLT(psCtrl->sPsiRotRDQ.fltD, psCtrl->fltPsiRB1Gain, sISDQ.fltD); 
    psCtrl->sPsiRotRDQ.fltD = MLIB_Mul_FLT(psCtrl->fltPsiRA1Gain, fltTemp);
    
    psCtrl->sPsiRotRDQ.fltQ = 0.0F;                                 
    
    /* Transform rotor flux into stator reference frame */
    GMCLIB_ParkInv_FLT (&psCtrl->sPsiRotRDQ, &sRotPos, &sPsiRAlBeTemp);    
    /* - Current(Rotor) Model  - ------- end ------------------------------------------- */        
    
    /* - Voltage(Stator) Model - ------ begin ------------------------------------------ */ 
    /* Compensate difference between stator and rotor model estimations */
    /* Calculate flux estimation differences */
    sCompErrAlpha = MLIB_Sub_FLT(psCtrl->sPsiRotSAlBe.fltAlpha, sPsiRAlBeTemp.fltAlpha);
    sCompErrBeta  = MLIB_Sub_FLT(psCtrl->sPsiRotSAlBe.fltBeta , sPsiRAlBeTemp.fltBeta);  

    /* Calculate compensation parts - Alpha PI controller */
    fltTempAlpha= psCtrl->sCtrl.fltCompAlphaInteg_1;    /* Last step of speed initialization from structure */        
    fltTemp1 = MLIB_Sub_FLT(sCompErrAlpha, psCtrl->sCtrl.fltCompAlphaErr_1);  /* ErrSub = (e(k)-e(k-1)) */  
    fltTempAlpha = MLIB_Mac_FLT(fltTempAlpha, sCompErrAlpha, fltIGain);       /* out = out(k-1) + e(k)*IGain */
    fltTempAlpha = MLIB_Mac_FLT(fltTempAlpha, fltTemp1, fltPGain);            /* out = out + (e(k)-e(k-1))*PGain */
    psCtrl->sCtrl.fltCompAlphaInteg_1   = fltTempAlpha;
    psCtrl->sCtrl.fltCompAlphaErr_1= sCompErrAlpha;       
  
    /* Calculate compensation parts - Beta PI controller */
    fltTempBeta= psCtrl->sCtrl.fltCompBetaInteg_1;      /* Last step of speed initialization from structure */     
    fltTemp1 = MLIB_Sub_FLT(sCompErrBeta, psCtrl->sCtrl.fltCompBetaErr_1);    /* ErrSub = (e(k)-e(k-1)) */  
    fltTempBeta = MLIB_Mac_FLT(fltTempBeta, sCompErrBeta, fltIGain);          /* out = out(k-1) + e(k)*IGain */
    fltTempBeta = MLIB_Mac_FLT(fltTempBeta, fltTemp1, fltPGain);              /* out = out + (e(k)-e(k-1))*PGain */
    psCtrl->sCtrl.fltCompBetaInteg_1   = fltTempBeta;
    psCtrl->sCtrl.fltCompBetaErr_1= sCompErrBeta;           
    
    fltTempAlpha = MLIB_Sub_FLT(psUSAlBe->fltAlpha, fltTempAlpha);
    fltTempBeta  = MLIB_Sub_FLT(psUSAlBe->fltBeta , fltTempBeta );
    
    /* Estimate stator flux */ 
    fltTempAlpha = MLIB_Msu_FLT(fltTempAlpha, fltRsEst, fltISAlpha);
    fltTempBeta  = MLIB_Msu_FLT(fltTempBeta , fltRsEst, fltISBeta );  
    fltPsiSAlpha = MLIB_Mac4_FLT(fltPsiSA1Gain, fltPsiSAlpha, fltPsiSA2Gain, fltTempAlpha);
    fltPsiSBeta  = MLIB_Mac4_FLT(fltPsiSA1Gain, fltPsiSBeta , fltPsiSA2Gain, fltTempBeta );    
    
    psCtrl->sPsiStatSAlBe.fltAlpha = fltPsiSAlpha;
    psCtrl->sPsiStatSAlBe.fltBeta = fltPsiSBeta;        
    
    /* Estimate rotor flux */
    fltPsiRAlpha = MLIB_Msu4_FLT(fltKrInvGain, fltPsiSAlpha, fltKrLsTotLeakGain, fltISAlpha);  
    fltPsiRBeta  = MLIB_Msu4_FLT(fltKrInvGain, fltPsiSBeta , fltKrLsTotLeakGain, fltISBeta ); 
    
    /* Electric toque calculation */
    fltTemp = MLIB_Msu4_FLT(fltPsiRAlpha, fltISBeta, fltPsiRBeta, fltISAlpha);
    psCtrl->fltTorque = MLIB_Mul_FLT (fltTemp, psCtrl->fltTorqueGain);
    
    /* Store state variables to structure */
    psCtrl->sPsiRotSAlBe.fltAlpha = fltPsiRAlpha;                          /* store stator flux in alpha/beta */ 
    psCtrl->sPsiRotSAlBe.fltBeta  = fltPsiRBeta ;         
    /* - Voltage(Stator) Model - ------ end ------------------------------------------ */ 
    
    /* Rotor flux position */ 
    /* Conversion to acc32_t, the revolution count is store in integer part, angle in fraction part (FRAC16(1.0) = PI) */                     
    psCtrl->a32RotFluxPos = (acc32_t) GFLIB_AtanYX_A32f(fltPsiRBeta, fltPsiRAlpha, &bErrFlag);                    
}    
