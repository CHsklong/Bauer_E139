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
* @brief ACIM Speed Estimator based on MRAS  
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "AMCLIB_ACIMSpeedMRAS_FLT.h"   

/***************************************************************************//*!
*
* @brief    ACIM Speed Estimator based on MRAS to calculate rotor electrical angular speed
*
* @param    ptr   GMCLIB_2COOR_ALBE_T_FLT *psISAlBe  - pointer to stator current in Alpha, Beta coordinates                      
*                 GMCLIB_2COOR_ALBE_T_FLT *psPsiRAlBe - pointer to rotor flux in Alpha, Beta coordinates  
*                 AMCLIB_ACIM_SPEED_MRAS_T_FLT *psCtrl - pointer to the MRAS parameter structure 
*
* @return   N/A
*       
* @remarks  The function calculates the electrical estimated rotor speed using the redundancy of two machine models of  
*           different structures that estimate the same state variable based on different sets of input variables. 
*           It means that the rotor speed can be obtained using an estimator with MRAS principle, in which 
*           the error vector is formed from the outputs of two models (both dependent on different motor parameters).    
*
*           Equations:
*           Is -> Park(RotPsiAngle) -> * Psi(MRAS) + B1*Is) -> ParkInv(RotPsiAngle) -> Psi(MRAS) 
*           e(k) = PsiAlpha(RFO) * PsiBeta(MRAS) - PsiBeta (RFO) * PsiAlpha(MRAS)
*           speed = out(k-1) + e(k)*IGain + (e(k)-e(k-1))*PGain
*           RotPsiAngle = speed * Ts
*
****************************************************************************/
void AMCLIB_ACIMSpeedMRAS_FLT_FC(const GMCLIB_2COOR_ALBE_T_FLT *psISAlBe,    
                                 const GMCLIB_2COOR_ALBE_T_FLT *psPsiRAlBe,  
                                 acc32_t a32RotPos,
                                 AMCLIB_ACIM_SPEED_MRAS_T_FLT *psCtrl)
{
    acc64_t a64Temp;     
    GMCLIB_2COOR_ALBE_T_FLT sPsiRAlBeTemp;   
    GMCLIB_2COOR_SINCOS_T_FLT sRotPos;    
    GMCLIB_2COOR_DQ_T_FLT sISDQ; 
    register float_t fltTemp;  
    register float_t fltTemp1;
    register float_t fltSpeed;     
    register float_t fltSpeedInteg_1;      
    register float_t fltPsiRA1Gain;
    register float_t fltPsiRB1Gain;     
    register float_t fltPGain;    
    register float_t fltIGain;  
    acc32_t a32TmpSpeed; 
    frac32_t f32Ts;       
    
    /* Store more times red quantities to registers */                  
    fltPsiRA1Gain  = psCtrl->fltPsiRA1Gain;
    fltPsiRB1Gain  = psCtrl->fltPsiRB1Gain;
    fltSpeed       = psCtrl->sCtrl.fltSpeedElErr_1;     
    fltSpeedInteg_1= psCtrl->sCtrl.fltSpeedElInteg_1;      
    fltPGain       = psCtrl->sCtrl.fltPGain;
    fltIGain       = psCtrl->sCtrl.fltIGain;  
    
    /*** Calculate rotor model flux estimation ***/ 
    /* Calculate sin&cos of rotor position angle */
    sRotPos.fltSin = GFLIB_Sin_FLTa (a32RotPos);
    sRotPos.fltCos = GFLIB_Cos_FLTa (a32RotPos);
    
    /* Transform stator current into rotor rotating reference frame */
    GMCLIB_Park_FLT (psISAlBe, &sRotPos, &sISDQ);
    
    /* Estimate rotor flux part */
    fltTemp = MLIB_Mac_FLT(psCtrl->sPsiRotRDQ.fltD, fltPsiRB1Gain, sISDQ.fltD); 
    psCtrl->sPsiRotRDQ.fltD = MLIB_Mul_FLT(fltPsiRA1Gain, fltTemp);

    fltTemp = MLIB_Mac_FLT(psCtrl->sPsiRotRDQ.fltQ, fltPsiRB1Gain, sISDQ.fltQ); 
    psCtrl->sPsiRotRDQ.fltQ = MLIB_Mul_FLT(fltPsiRA1Gain, fltTemp); 
    
    /* Transform rotor flux into stator reference frame */
    GMCLIB_ParkInv_FLT (&psCtrl->sPsiRotRDQ, &sRotPos, &sPsiRAlBeTemp);    
    
    /* Calculate cross product as error of reference FluxR and adaptive FluxR */
    fltTemp = MLIB_Msu4_FLT (sPsiRAlBeTemp.fltAlpha, psPsiRAlBe->fltBeta, sPsiRAlBeTemp.fltBeta, psPsiRAlBe->fltAlpha);

    /* PI Controller calculation for raw mechanical speed*/   
    fltTemp1  = MLIB_Sub_FLT(fltTemp, psCtrl->sCtrl.fltSpeedElErr_1);        /* ErrSub = (e(k)-e(k-1)) */  
    fltSpeedInteg_1= MLIB_Mac_FLT(fltSpeedInteg_1, fltTemp,  fltIGain);      /* out = out(k-1) + e(k)*IGain */   
    fltSpeed  = MLIB_Mac_FLT(fltSpeedInteg_1, fltTemp1, fltPGain);           /* out = out + (e(k)-e(k-1))*PGain */
    psCtrl->sCtrl.fltSpeedElInteg_1 = fltSpeed;
    psCtrl->fltSpeedEl = fltSpeed;
    psCtrl->sCtrl.fltSpeedElErr_1 = fltTemp;             
    
    /* Filtering speeds by first order low-pass filter */  
    fltTemp = GDFLIB_FilterIIR1_FLT (fltSpeed, &psCtrl->fltSpeedElIIR1Param); 
    psCtrl->fltSpeedElIIR1 = fltTemp;                                        /* Electric filtered speed */
    psCtrl->fltSpeedMeIIR1 = MLIB_Mul_FLT(fltTemp, psCtrl->fltSpeedMeGain);  /* Mechanical filtered speed calculation */            
    
    /* Angle (position) overflow */    
    a32TmpSpeed = MLIB_ConvSc_A32ff (fltSpeed, FLOAT_PI); 
    f32Ts       = MLIB_Conv_F32f (psCtrl->fltTs);  
    /* Integrate electrical speed to obtain position */  
    a64Temp  = (acc64_t)(((frac16_t)a32RotPos)<<16);                         /* covert to 64 bit accumulator type  */   
    a64Temp += (acc64_t) (((int64_t) a32TmpSpeed) * ((int64_t) f32Ts))>>15;  /* multiply a64 * f32 */
    a32RotPos  = (acc32_t) a64Temp;                                          /* store fraction (angle) part - 32bit overflow */    
        
    /* Conversion to acc32_t, the revolution count is stored in integer part, angle in fraction part (FRAC16(1.0) = PI) */   
    psCtrl->a32RotPos = (acc32_t)(a32RotPos >> 16);                          /* convert fraction part 16 bit for acc32 type */
}
