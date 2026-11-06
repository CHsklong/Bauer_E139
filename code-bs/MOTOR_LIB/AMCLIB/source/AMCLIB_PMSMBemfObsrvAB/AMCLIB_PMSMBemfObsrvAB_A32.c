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
*******************************************************************************
*
* @brief  Algorithm of PMSM Back Electromotive Force observer in stationary 
*         reference frame   
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "AMCLIB_PMSMBemfObsrvAB_A32.h"

/***************************************************************************//*!
*
* @brief    BEMF observer using alpha, beta components to estimate the angle's
*           sin anAlpha cosine.
*
* @param    ptr   GMCLIB_2COOR_ALBE_T_F16 *psIAlBe - pointer to the alpha, beta current
*                 GMCLIB_2COOR_ALBE_T_F16 *psUAlBe - pointer to the alpha, beta voltage
*                 AMCLIB_BEMF_OBSRV_AB_T *psCtrl - pointer to the parameters of the observer
* @param    in    frac16_t f16Speed - actual speed
*
* @return   none
*        
* @remarks  The function calculates the algorithm of back electromotive 
*           force observer in stationery reference frame. 
*
*    f32TmpEObsrvAlpha= psCtrl_sCtrl_f32IAlpha_1 + 
*                       (psCtrl_sCtrl_a32IGain + psCtrl_sCtrl_a32PGain) * (psUAlBe_f16Alpha - psCtrl_sIObsrv_f32Alpha) 
*    f32TmpEObsrvBeta = psCtrl_sCtrl_f32IBeta_1 +
*                       (psCtrl_sCtrl_a32IGain + psCtrl_sCtrl_a32PGain) * (psUAlBe_f16Beta  - psCtrl_sIObsrv_f32Beta)
*
*    f32TmpEObsrvAlpha= f32TmpEObsrvAlpha / sqrt(f32TmpEObsrvAlpha ^ 2 + f32TmpEObsrvAlpha ^ 2) 
*    f32TmpEObsrvBeta = f32TmpEObsrvBeta  / sqrt(f32TmpEObsrvBeta ^ 2  + f32TmpEObsrvBeta ^ 2)
*
*    f32TmpIObsrvAlpha= a32IGain * psCtrl_sIObsrv_f32Alpha + a32UGain * psUAlBe_f16Alpha + 
*                       psIAlBe_f16Beta  * f16Speed * a32WIGain + a32EGain * f32TmpEObsrvAlpha         
*    f32TmpIObsrvBeta = a32IGain * psCtrl_sIObsrv_f32Beta  +  a32UGain * psUAlBe_f16Beta  - 
*                       psIAlBe_f16Alpha * f16Speed * a32WIGain + a32EGain * f32TmpEObsrvBeta
*
*    f32TmpEObsrvAlpha= -f32TmpEObsrvAlpha    if    f16Speed < 0                                           
*    f32TmpEObsrvBeta = -f32TmpEObsrvBeta     else
*
****************************************************************************/
void AMCLIB_PMSMBemfObsrvAB_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psIAlBe,
                                   const GMCLIB_2COOR_ALBE_T_F16 *psUAlBe,
                                   frac16_t f16Speed,
                                   AMCLIB_BEMF_OBSRV_AB_T_A32 *psCtrl)
{
    register acc64_t a64TmpIObsrvAlpha;
    register acc64_t a64TmpIObsrvBeta;  
    register acc64_t a64CurrentAlphaProp;
    register acc64_t a64CurrentBetaProp;  
    register acc64_t a64Tmp; 
    register frac64_t f64Temp;
    register frac64_t f64Temp1;   
    register frac32_t f32TmpEObsrvAlpha;
    register frac32_t f32TmpEObsrvBeta;
    register frac32_t f32Temp;    
    register acc32_t  a32CtrlIGain;
    register acc32_t  a32CtrlPGain;
    register int32_t i32SatMax;
    register int32_t i32SatMin;
    frac32_t f32Temp1;
    acc32_t  a32WIGain;
    acc32_t  a32IGain;    
    acc32_t  a32EGain;          
    acc32_t  a32UGain;
    frac32_t f32Speed;
    frac32_t f32CtrlIObsrvAlpha_1;
    frac32_t f32CtrlIObsrvBeta_1;
    frac32_t f32IObsrvAlpha;
    frac32_t f32IObsrvBeta; 
    frac32_t f32UInAlpha;     
    frac32_t f32UInBeta;
    frac32_t f32IAlphaIn; 
    frac32_t f32IBetaIn;
    uint16_t u16tempAlpha; 
    uint16_t u16tempBeta; 
        
    /* Store more times red quantities to registers */
    f32IObsrvAlpha = psCtrl->sIObsrv.f32Alpha;
    f32IObsrvBeta  = psCtrl->sIObsrv.f32Beta;    
    f32CtrlIObsrvAlpha_1 = psCtrl->sCtrl.f32IAlpha_1;
    f32CtrlIObsrvBeta_1 = psCtrl->sCtrl.f32IBeta_1;
    a32CtrlPGain = psCtrl->sCtrl.a32PGain;
    a32CtrlIGain = psCtrl->sCtrl.a32IGain;     
    a32IGain = psCtrl->a32IGain;    
    a32UGain = psCtrl->a32UGain;        
    a32WIGain= psCtrl->a32WIGain;  
    a32EGain = psCtrl->a32EGain;   
    f32UInAlpha  = psUAlBe->f16Alpha << 16;     
    f32UInBeta  = psUAlBe->f16Beta << 16;
    f32IAlphaIn = psIAlBe->f16Alpha << 16;  
    f32IBetaIn = psIAlBe->f16Beta << 16;
    f32Speed = f16Speed;
        
    /* Alpha controller, error calculation using saturation subtraction */
    f32Temp = f32IAlphaIn - f32IObsrvAlpha;
    i32SatMax = (~(int32_t)(f32IAlphaIn) &   (int32_t)(f32IObsrvAlpha)) &   f32Temp;
    i32SatMin =   (int32_t)(f32IAlphaIn) & (~(int32_t)(f32IObsrvAlpha)) & (~f32Temp);
    f32Temp = (i32SatMin < 0) ? -2147483648 : f32Temp;
    f32Temp = (i32SatMax < 0) ? 2147483647 : f32Temp;
    
    /* Alpha controller calculations */ 
    a64CurrentAlphaProp = (acc64_t)((((int64_t) a32CtrlPGain) * ((int64_t) f32Temp))>>15);
    a64Tmp = (acc64_t)((((int64_t) a32CtrlIGain) * ((int64_t) f32Temp))>>15);  
    /* Saturation addition */      
    f64Temp = (frac64_t)f32CtrlIObsrvAlpha_1 + a64Tmp;
    f64Temp1 = a64CurrentAlphaProp + f64Temp;    
    f64Temp = (f64Temp > 2147483647) ? 2147483647 : f64Temp;   
    f64Temp = (f64Temp < -2147483648) ? -2147483648 : f64Temp;   
    f32CtrlIObsrvAlpha_1 = f64Temp; 
    /* Saturation addition */     
    f64Temp1 = (f64Temp1 > 2147483647) ? 2147483647 : f64Temp1;    
    f64Temp1 = (f64Temp1 < -2147483648) ? -2147483648 : f64Temp1; 
    f32TmpEObsrvAlpha = f64Temp1;
    
    /* Beta controller, error calculation using saturation subtraction */
    f32Temp = f32IBetaIn - f32IObsrvBeta;
    i32SatMax = (~(int32_t)(f32IBetaIn) &   (int32_t)(f32IObsrvBeta)) &   f32Temp;
    i32SatMin =   (int32_t)(f32IBetaIn) & (~(int32_t)(f32IObsrvBeta)) & (~f32Temp);
    f32Temp = (i32SatMin < 0) ? -2147483648 : f32Temp;
    f32Temp = (i32SatMax < 0) ? 2147483647 : f32Temp;
  
    /* Beta controller calculation */
    a64CurrentBetaProp = (acc64_t)((((int64_t) a32CtrlPGain) * ((int64_t) f32Temp))>>15);    
    a64Tmp = (acc64_t)((((int64_t) a32CtrlIGain) * ((int64_t) f32Temp))>>15); 
    /* Saturation addition */    
    f64Temp = (frac64_t)f32CtrlIObsrvBeta_1 + a64Tmp;
    f64Temp1 = a64CurrentBetaProp + f64Temp;     
    f64Temp = (f64Temp > 2147483647) ? 2147483647 : f64Temp;    
    f64Temp = (f64Temp < -2147483648) ? -2147483648 : f64Temp;  
    f32CtrlIObsrvBeta_1 = f64Temp;
    /* Saturation addition */    
    f64Temp1 = ((frac64_t)f64Temp1 > (frac64_t)2147483647) ? 2147483647 : f64Temp1;    
    f64Temp1 = ((frac64_t)f64Temp1 < (frac64_t)-2147483648) ? -2147483648 : f64Temp1; 
    f32TmpEObsrvBeta = f64Temp1;

    /* Alpha observer calculations and Beta observer calculation */   
    /* Alpha acc32_t Current observer calculation with accumulation to 64 bits */
    /* Accumulation of first multiplication: psCtrl->a32UGain * psCtrl->sIObsrv.f32Alpha */
    a64TmpIObsrvAlpha = (acc64_t)(((int64_t) a32IGain) * ((int64_t) f32IObsrvAlpha));     
    /* Accumulation of second multiplication: psCtrl->a32UGain * psUAlphaBeta->f16Alpha */
    a64TmpIObsrvAlpha += (acc64_t)(((int64_t) a32UGain) * ((int64_t) f32UInAlpha));            
	/* Subtraction of third multiplication: f16Speed * psCtrl->psIAlphaBeta->f16Beta * psCtrl->a32WIGain */
    f32Temp = f32Speed * psIAlBe->f16Beta;
    a64TmpIObsrvAlpha -= (acc64_t)((((int64_t) a32WIGain) * ((int64_t) f32Temp))<<1);     
	/* Accumulation of fourth multiplication: psCtrl->a32EGain * f32TmpEObsrvAlpha */
    a64TmpIObsrvAlpha += (acc64_t)(((int64_t) a32EGain) * ((int64_t) f32TmpEObsrvAlpha));         
    
    /* Beta acc32Current observer calculation with accumulation to 64 bits */
    /* Accumulation of first multiplication: psCtrl->a32UGain * psCtrl->sIObsrv.f32Alpha */ 
    a64TmpIObsrvBeta = (acc64_t)(((int64_t) a32IGain) * ((int64_t)f32IObsrvBeta)); 
    /* Accumulation of second multiplication:psCtrl->a32UGain * psUAlphaBeta->f16Alpha */
    a64TmpIObsrvBeta += (acc64_t)(((int64_t) a32UGain) * ((int64_t) f32UInBeta));
    /* Accumulation of third multiplication: f16Speed * psCtrl->psIAlphaBeta->f16Alpha * psCtrl->a32WIGain */
    f32Temp = f32Speed * psIAlBe->f16Alpha;       
    a64TmpIObsrvBeta += (acc64_t)((((int64_t) a32WIGain) * ((int64_t) f32Temp))<<1);   
    /* Accumulation of fourth multiplication: psCtrl->a32EGain * f32TmpEObsrvBeta */
    a64TmpIObsrvBeta += (acc64_t)(((int64_t) a32EGain)  * ((int64_t) f32TmpEObsrvBeta));          
 
    /* Current observer saturation */
    a64TmpIObsrvAlpha = a64TmpIObsrvAlpha >> 15; 
    a64TmpIObsrvAlpha = (a64TmpIObsrvAlpha > ((int64_t)(2147483647))) ? ((int64_t)(2147483647)) : a64TmpIObsrvAlpha;    
    a64TmpIObsrvAlpha = (a64TmpIObsrvAlpha < ((int64_t)(-2147483648))) ? ((int64_t)(-2147483648)) : a64TmpIObsrvAlpha;    
    a64TmpIObsrvBeta = a64TmpIObsrvBeta >> 15;  
    a64TmpIObsrvBeta = (a64TmpIObsrvBeta > ((int64_t)(2147483647))) ? ((int64_t)(2147483647)) : a64TmpIObsrvBeta;    
    a64TmpIObsrvBeta = (a64TmpIObsrvBeta < ((int64_t)(-2147483648))) ? ((int64_t)(-2147483648)) : a64TmpIObsrvBeta;     

    /* Store state variables to psCtrl structure */     
    psCtrl->sEObsrv.f32Alpha = f32TmpEObsrvAlpha;
    psCtrl->sEObsrv.f32Beta  = f32TmpEObsrvBeta;
    
    /* Unity vector calculation */
    /* Count leading bits for normalization the numbers for more accurate the power of two operations */  
    u16tempAlpha = MLIB_Clb_U16l (f32TmpEObsrvAlpha);
    u16tempBeta  = MLIB_Clb_U16l (f32TmpEObsrvBeta );
    if (u16tempAlpha >= u16tempBeta)
    {
        f32TmpEObsrvAlpha = f32TmpEObsrvAlpha << u16tempBeta ;   
        f32TmpEObsrvBeta  = f32TmpEObsrvBeta  << u16tempBeta ; 
    }
    else 
    {
        f32TmpEObsrvAlpha = f32TmpEObsrvAlpha << u16tempAlpha ;   
        f32TmpEObsrvBeta  = f32TmpEObsrvBeta  << u16tempAlpha ; 
    }
    
    /* Limitation to achieve addition alpha and beta components lower that FRAC32(1.0) */ 
    f32Temp = MLIB_AbsSat_F32(f32TmpEObsrvAlpha);
    f32Temp1= MLIB_AbsSat_F32(f32TmpEObsrvBeta );
    if ((f32Temp > FRAC32(0.707106781)) || (f32Temp1 > FRAC32(0.707106781)))
    {   
        f32TmpEObsrvAlpha = f32TmpEObsrvAlpha >> 1 ; 
        f32TmpEObsrvBeta  = f32TmpEObsrvBeta  >> 1 ;
    }                
        
    /* Module calculation: square components */
    f32Temp  = MLIB_Mul_F32(f32TmpEObsrvAlpha,f32TmpEObsrvAlpha);
    f32Temp1 = MLIB_Mul_F32(f32TmpEObsrvBeta ,f32TmpEObsrvBeta );        
    /* Addition saturation of squared components, each is < FRAC32(0.5) to avoid inaccuracy of the vector magnitude */   
    f32Temp = MLIB_AddSat_F32(f32Temp, f32Temp1);
    
    /* SQRT calculation of square addition */
    f32Temp = GFLIB_Sqrt_F16l(f32Temp);
    /* Reciprocal value calculation */
    f32Temp = MLIB_Rcp1Q1_A32s(f32Temp) ; 
    
    /* Unity alpha and beta values calculation */ 
    f32TmpEObsrvAlpha = MLIB_MulSat_F16as(f32Temp, f32TmpEObsrvAlpha>>16);     
    f32TmpEObsrvBeta  = MLIB_MulSat_F16as(f32Temp, f32TmpEObsrvBeta >>16); 
    
    /* BEMF determination according to speed sign */
    if (f32Speed < 0)
    {   f32TmpEObsrvAlpha = MLIB_NegSat_F16(f32TmpEObsrvAlpha);
    }
    else
    {   f32TmpEObsrvBeta  = MLIB_NegSat_F16(f32TmpEObsrvBeta);
    }
    
    /* Store state variables to psCtrl structure */ 
    psCtrl->sIObsrv.f32Alpha  = (frac32_t) a64TmpIObsrvAlpha;
    psCtrl->sIObsrv.f32Beta   = (frac32_t) a64TmpIObsrvBeta ;    
    psCtrl->sCtrl.f32IAlpha_1 = f32CtrlIObsrvAlpha_1;
    psCtrl->sCtrl.f32IBeta_1  = f32CtrlIObsrvBeta_1 ;  
    psCtrl->sUnityVctr.f16Sin = (frac16_t)f32TmpEObsrvAlpha;
    psCtrl->sUnityVctr.f16Cos = (frac16_t)f32TmpEObsrvBeta ;         
}
