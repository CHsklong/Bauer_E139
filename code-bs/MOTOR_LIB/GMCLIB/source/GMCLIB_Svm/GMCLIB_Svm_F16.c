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
* @brief Space vector modulation 
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GMCLIB_Svm_F16.h" 
#include "gmclib.h"

/***************************************************************************//*!
*
* @brief  Standard ICT function
*
* @param  ptr - GMCLIB_2COOR_ALBE_T_F16 *psIn - IN - Direct(alpha) and quadrature(beta)             
*               component of the stator voltage vector in the stationary reference 
*               in frac16_t <-1;1) format.                                                     
*             - GMCLIB_3COOR_T_F16 *psOut - OUT - Pointer to duty cycle of the A, B and C 
*               phases in frac16_t <-1;1) format.   
*
* @return This function returns - sector number of the stator voltage vector resides in.
*       
* @remarks  The function calculates the appropriate duty cycles needed to generate 
*           a given stator reference voltage using the Space Vector Modulation with 
*           a duty cycle of the null switching state from states O000 and O111 
*           in each sector of the hexagon.
*
*         The function fills variables a, b and c in the data structure psOut 
*         and returns the sector number of the stator voltage vector resides in.
*
*    pwm_a = 0.5 + ua / 2   
*    pwm_b = 0.5 + (ualpha + sqrt(3) * ubeta) / 4 
*    pwm_c = 0.5 + (ualpha - sqrt(3) * ubeta) / 4  
*
*******************************************************************************/
uint16_t GMCLIB_SvmIct_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psIn,
                              GMCLIB_3COOR_T_F16 *psOut)
{
    register frac32_t f32Sqrt3DivBy2;   
    register frac32_t f32Uref1, f32Uref2, f32Uref3; 
    register frac32_t f32temp, f32Sector;
    register frac32_t f32Alpfa,f32BetaMultSqrtDiv2Half,f32BetaDiv2,f32AlphaDiv2;
    register frac32_t f32HalfSubAlphaDiv2Half;

    f32Alpfa = psIn->f16Alpha;
    f32Uref1 = psIn->f16Beta;
    f32BetaDiv2 = f32Uref1 >> 1;                          /* f32BetaDiv2 = f32Beta >> 1 */
    f32AlphaDiv2 = f32Alpfa >> 1;    
    f32Sqrt3DivBy2 = GMCLIB_SQRT3_DIV_2_F16;
      
    f32temp = (f32Alpfa * f32Sqrt3DivBy2) >> 15 ; 
    f32Uref2 =  f32temp - f32BetaDiv2 ; 
    f32Uref3 = -f32temp - f32BetaDiv2 ;
    
    /* 
    f32Uref1 =  beta
    f32Uref2 =  alpfa * sqrt(3)/2 - beta/2
    f32Uref3 = -alpfa * sqrt(3)/2 - beta/2
    */ 
    
    /* Find sector in the hexagon */
    f32Sector  = (f32Uref1 > 0) ? 1 : 0;                  /* uref1 > 1 sector  = 1 else 0 */ 
    f32Sector += (f32Uref2 > 0) ? 2 : 0;                  /* uref2 > 2 sector += 2 else 0 */     
    f32Sector += (f32Uref3 > 0) ? 4 : 0;                  /* uref3 > 3 sector += 4 else 0 */        
    
    /* Phase A calculation */
    f32temp = 0x4000 + f32AlphaDiv2;                      /* 0.5 + Alpha / 2 */      
    f32temp = (f32temp > 32767 ) ? INT16_MAX : f32temp;   /* high saturation */
    psOut -> f16A = (f32temp < 0) ? 0 : f32temp;          /* low restriction */      
    
    /* Constant for B and C phase calculations */    
    f32HalfSubAlphaDiv2Half = (0x8000 - f32AlphaDiv2)>>1; /* (1 - alpha / 2 ) / 2 = 0.5 -alpha / 4 */
    f32BetaMultSqrtDiv2Half = (f32Uref1 * f32Sqrt3DivBy2) >> 16 ; /* (beta * sqrt(3) / 2 ) / 2 */
    
    /* Phase B calculation */  
    f32temp = f32HalfSubAlphaDiv2Half + f32BetaMultSqrtDiv2Half;    /* f32temp = 0.5 - alpha / 4  + beta * sqrt(3) / 4  */
    f32temp = (f32temp > 32767 ) ? INT16_MAX : f32temp;   /* high saturation */ 
    psOut -> f16B = (f32temp < 0) ? 0 : f32temp;          /* low restriction */
      
    /* Phase C calculation */    
    f32temp = f32HalfSubAlphaDiv2Half - f32BetaMultSqrtDiv2Half;    /* f32temp = 0.5 - alpha / 4  - beta *  sqrt(3) / 4  */
    f32temp = (f32temp > 32767 ) ? INT16_MAX : f32temp;   /* high saturation */
    psOut -> f16C = (f32temp < 0) ? 0 : f32temp;          /* low restriction */
       
    return(f32Sector);  /* return sector value */
}

/***************************************************************************//*!
*
* @brief  O000 state null vector SVM function
*
* @param  ptr - GMCLIB_2COOR_ALBE_T_F16 *psIn - IN - Direct(alpha) and quadrature(beta)             
*               component of the stator voltage vector in the stationary reference 
*               in frac16_t <-1;1) format.                                                     
*             - GMCLIB_3COOR_T_F16 *psOut - OUT - Pointer to duty cycle of the A, B and C 
*               phases in frac16_t <-1;1) format.   
*
* @return This function returns - sector number of the stator voltage vector resides in.
*       
* @remarks  The function calculates the appropriate duty cycles needed to generate 
*           a given stator reference voltage using the Space Vector Modulation with 
*           a duty cycle of the null switching state from states O000 in each sector 
*           of the hexagon.
*
*         The function fills variables a, b and c in the data structure psOut 
*         and returns the sector number of the stator voltage vector resides in.
*
*     X = ubeta
*     Y = 1/2(ubeta - sqrt(3)*ualfa)
*     Z = 1/2(ubeta + sqrt(3)*ualfa)
*
*     Sectors   U0,U60   U60,U120   U120,U180   U180,U240   U240,U300   U300,U0
*     t_1       X        Y          -Y          Z           -Z          -X
*     t_2       -Z       Z          X          -X           -Y          Y
*
*     t1 = 0
*     t2 = t1 + t_1
*     t3 = t2 + t_2
*
*******************************************************************************/
uint16_t GMCLIB_SvmU0n_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psIn,
                              GMCLIB_3COOR_T_F16 *psOut)
{
    register frac32_t f32Sqrt3DivBy2;   
    register frac32_t f32A, f32B, f32C;
    register frac32_t f32temp;
    register frac32_t f32Alpha,f32Beta,f32BetaDiv2;
    
    f32Alpha = psIn->f16Alpha;
    f32Beta = psIn->f16Beta;
    f32BetaDiv2 = f32Beta >> 1;
    f32Sqrt3DivBy2 = GMCLIB_SQRT3_DIV_2_F16;
    
    /* 
    f32AlphaMulSqrt3Half =  f32Alpha * sqrt(3)/2
    f32A =  f32Alpha * sqrt(3)/2 - beta/2
    f32B = -f32Alpha * sqrt(3)/2 - beta/2
    */    
    f32C = (f32Alpha * f32Sqrt3DivBy2) >> 15 ; 
    f32A =  f32C - f32BetaDiv2 ; 
    f32B = -f32C - f32BetaDiv2 ;  /* -f32B = f32C + f32BetaDiv2 */ 
    
     /* Sector determination: if B > 0 and A > 0, the sector 5 is calculated */
    if (f32B <= 0)
    {
        /*** Sector 2 calculation(s) ***/
        if (f32A <= 0 )            
        { 
    /*  psOut -> f16C = 0
        psOut -> f16A = PhaseC + alpha * sqrt(3) / 2 + beta / 2
        psOut -> f16B = PhaseA - alpha * sqrt(3) / 2 + beta / 2   */
            
            psOut -> f16C = 0;                                      /* psOut -> f16C = 0 */
            f32temp = -f32B;                                        /* temp = alpha * sqrt(3) / 2 + beta / 2  */
            f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;  /* high saturation */
            psOut -> f16A = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase A */
            psOut -> f16B = (f32Beta < 0) ? 0 : f32Beta;            /* psOut -> f16B = beta, limits output and store phase B */
            return((uint16_t)0x2);                                  /* return sector 2 */
        }  
        
        /*** Sector 6 calculation(s) ***/
        if (f32Beta <= 0)        
        {
    /*  psOut -> f16B = 0
        psOut -> f16C = PhaseB - beta
        psOut -> f16A = PhaseC + alpha * sqrt(3) / 2 + beta / 2   */
            
            psOut -> f16B =  0 ;                                    /* psOut -> f16B = 0 */
            f32temp = 0 - f32Beta;                                  /* temp = PhaseB - beta  */
            f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;  /* high saturation */
            psOut -> f16C = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase C */
            f32temp = f32A ;                                        /* temp = alpha * sqrt(3) / 2 - beta / 2 */
            f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;  /* high saturation */
            psOut -> f16A = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase A */
            return((uint16_t)0x6);                                  /* return sector 6 */
        }  
      
    /* Sector 1 calculation(s) */  
    /* psOut -> f16C = 0 
       psOut -> f16B = PhaseC + beta
       psOut -> f16A = PhaseB + alpha * sqrt(3) / 2 - beta / 2 */
    
    psOut -> f16C =  0 ;                                            /* psOut -> f16C = 0 */
    f32temp = f32Beta;                                              /* temp = Beta */
    psOut -> f16B = (f32temp < 0) ? 0 : f32temp;                    /* limits output and store phase B */
    f32temp += f32A;                                                /* temp = temp + alpha * sqrt(3)/2 - beta/2 = temp + f32A */
    f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;          /* high saturation */
    psOut -> f16A = (f32temp < 0) ? 0 : f32temp;                    /* limits output and store phase A */
    return((uint16_t)0x1);                                          /* return sector 1 */
    }  
  
    if (f32A <= 0) 
    {
        /* Sector 4 calculation(s) */
        if (f32Beta <= 0) 
        /*  psOut -> f16A = 0
            psOut -> f16B = PhaseA - alpha * sqrt(3) / 2 + beta / 2
            psOut -> f16C = PhaseB - beta */
        {
            psOut -> f16A = 0 ;                                     /* psOut -> f16A = 0 */
            f32temp = f32B;                                         /* temp = - alpha * sqrt(3) / 2 - beta / 2 */
            f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;  /* high saturation */            
            psOut -> f16C = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase C */
            f32B += f32Beta ;                                       /* f32B = f32B + beta */
            f32B = (f32B > INT16_MAX) ? INT16_MAX : f32B;           /* high saturation */
            psOut -> f16B = (f32temp < 0) ?  0 : f32B;              /* limits output and store phase B */
            return((uint16_t)0x4);                                  /* return sector 4 */
        }  
      
        /* Sector 3 calculation(s) */
        /*  psOut -> f16A = 0
            psOut -> f16C = PhaseA - alpha * sqrt(3) / 2 - beta / 2
            psOut -> f16B = PhaseC + beta */
         
        psOut -> f16A = 0 ;                                         /* psOut -> f16A = 0 */
        f32temp = f32B;                                             /* temp = - alpha * sqrt(3) / 2 - beta / 2 */
        f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;      /* high saturation */
        psOut -> f16C = (f32temp < 0) ? 0 : f32temp;                /* limits output and store phase C */
        f32temp += f32Beta;                                         /* temp = temp + beta  */
        f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;      /* high saturation */
        psOut -> f16B = (f32temp < 0) ? 0 : f32temp;                /* limits output and store phase B */
        return((uint16_t)0x3);                                      /* return sector 3 */
    }

    /*** Sector 5 calculation(s) ***/
    
    /* B > 0 and A > 0, the sector 5 is calculated */ 
    /*  pOut -> f16B = 0
        pOut -> f16A = PhaseB + alpha * sqrt(3) / 2 - beta / 2
        pOut -> f16C = PhaseA - alpha * sqrt(3) / 2 - beta / 2  */  
    
    psOut -> f16B = 0;                                              /* psOut -> f16B = 0 */
    f32temp = f32A;                                                 /* Phase A calculation*/      
    f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;          /* high saturation */
    psOut -> f16A  = (f32temp < 0) ? 0 : f32temp;                   /* limits output and store phase A */
    f32A += f32B;                                                   /* f32temp = f32temp - beta*/        
    f32A = (f32A > INT16_MAX) ? INT16_MAX : f32A;                   /* high saturation */
    psOut -> f16C = (f32A < 0)  ? 0 : f32A;                         /* limits output and store phase C */   
    return((uint16_t)0x5);                                          /* return sector 2 */     
}

/***************************************************************************//*!
*
* @brief  O111 state null vector SVM function
*
* @param  ptr - GMCLIB_2COOR_ALBE_T_F16 *psIn - IN - Direct(alpha) and quadrature(beta)             
*               component of the stator voltage vector in the stationary reference 
*               in frac16_t <-1;1) format.                                                     
*             - GMCLIB_3COOR_T_F16 *psOut - OUT - Pointer to duty cycle of the A, B and C 
*               phases in frac16_t <-1;1) format.   
*
* @return This function returns - sector number of the stator voltage vector resides in.
*       
* @remarks  The function calculates the appropriate duty cycles needed to generate 
*           a given stator reference voltage using the Space Vector Modulation with 
*           a duty cycle of the null switching state from states O111 in each sector
*           of the hexagon.
*
*         The function fills variables a, b and c in the data structure psOut 
*         and returns the sector number of the stator voltage vector resides in.
*
*     X = ubeta
*     Y = 1/2(ubeta - sqrt(3)*ualfa)
*     Z = 1/2(ubeta + sqrt(3)*ualfa)
*
*     Sectors   U0,U60   U60,U120   U120,U180   U180,U240   U240,U300   U300,U0
*     t_1       X        Y          -Y          Z           -Z          -X
*     t_2       -Z       Z          X          -X           -Y          Y
*
*     t1 = T - t_1 - t_2
*     t2 = t1 + t_1
*     t3 = t2 + t_2
*
*******************************************************************************/
uint16_t GMCLIB_SvmU7n_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psIn,
                              GMCLIB_3COOR_T_F16 *psOut)
{
    register frac32_t f32Sqrt3DivBy2;   
    register frac32_t f32A, f32B, f32C;
    register frac32_t f32temp;
    register frac32_t f32Alpha,f32Beta,f32BetaDiv2;
    
    f32Alpha = psIn->f16Alpha;
    f32Beta = psIn->f16Beta;
    f32BetaDiv2 = f32Beta >> 1;
    f32Sqrt3DivBy2 = GMCLIB_SQRT3_DIV_2_F16;
    
    /* 
    f32AlphaMulSqrt3Half =  f32Alpha * sqrt(3)/2
    f32A =  f32Alpha * sqrt(3)/2 - beta/2
    f32B = -f32Alpha * sqrt(3)/2 - beta/2
    */    
    f32C = (f32Alpha * f32Sqrt3DivBy2) >> 15 ; 
    f32A =  f32C - f32BetaDiv2 ; 
    f32B = -f32C - f32BetaDiv2 ;  /* -f32B = f32C + f32BetaDiv2 */
    
     /* Sector determination: if B > 0 and A > 0, the sector 5 is calculated */
    if (f32B <= 0)
    {
        /*** Sector 2 calculation(s) ***/
        if (f32A <= 0 )            
        { 
    /* psOut -> f16C = 1 - beta
       psOut -> f16A = PhaseC + alpha * sqrt(3) / 2 + beta / 2
       psOut -> f16B = PhaseA - alpha * sqrt(3) / 2 + beta / 2   */      
            
            f32temp = INT16_MAX;
            psOut -> f16B = f32temp;                                /* psOut -> f16B = 1 */  
            f32temp -= f32Beta;                                     /* temp = 1 - beta  */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16C = f32temp;                                /* limits output and store phase C */                      
            f32temp -= f32B;                                        /* temp = temp + alpha * sqrt(3) / 2 + beta / 2)  */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16A = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase A */
            return((uint16_t)0x2);                                  /* return sector 2 */
        }  
        
        /*** Sector 6 calculation(s) ***/
        if (f32Beta <= 0)        
        {
    /*  psOut -> f16B = 1 - alpha * sqrt(3) / 2 + beta / 2
        psOut -> f16C = PhaseB - beta
        psOut -> f16A = PhaseC + alpha * sqrt(3) / 2 + beta / 2    */
            
            f32temp = INT16_MAX;
            psOut -> f16A = f32temp;                                /* psOut -> f16A = 1 */  
            f32temp -= f32A;                                        /* temp = temp - alpha * sqrt(3) / 2 + beta / 2 */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16B = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase B */
            f32temp -= f32Beta ;                                    /* temp = temp - beta  */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16C = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase C */
            return((uint16_t)0x6);                                  /* return sector 6 */
        }  
      
    /* Sector 1 calculation(s) */  
    /* psOut -> f16C = 1 - alpha * sqrt(3) / 2 - beta / 2
       psOut -> f16B = PhaseC + beta
       psOut -> f16A = PhaseB + alpha * sqrt(3) / 2 - beta / 2  */
    
    f32temp = INT16_MAX;              
    psOut -> f16A = f32temp;                                        /* psOut -> f16A = 1 */  
    f32temp += f32B;                                                /* temp = 1 - alpha * sqrt(3) / 2 - beta / 2 */
    f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;          /* high saturation */ 
    psOut -> f16C = (f32temp < 0) ? 0 : f32temp;                    /* limits output and store phase C */
    f32temp += f32Beta;                                             /* temp = temp + beta */
    f32temp = (f32temp > INT16_MAX) ? INT16_MAX : f32temp;          /* high saturation */
    psOut -> f16B = (f32temp < 0) ? 0 : f32temp;                    /* limits output and store phase B */
    return((uint16_t)0x1);                                          /* return sector 1 */
    }  
  
    if (f32A <= 0) 
    {
        /* Sector 4 calculation(s) */
        if (f32Beta <= 0) 
        /*  psOut -> f16A = 1 + alpha * sqrt(3) / 2 + beta / 2
            psOut -> f16B = PhaseA - alpha * sqrt(3) / 2 + beta / 2
            psOut -> f16C = PhaseB - beta */
        {
            f32temp = INT16_MAX;              
            psOut -> f16C = f32temp;                                /* psOut -> f16C = 1 */  
            f32temp -= f32B;                                        /* temp = 1 + alpha * sqrt(3) / 2 + beta / 2 */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16A = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase A */
            f32temp -= f32A;                                        /* temp = temp - alpha * sqrt(3) / 2 + beta / 2  */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16B = (f32temp < 0) ?  0 : f32temp;           /* limits output and store phase B */
            return((uint16_t)0x4);                                  /* return sector 4 */
        }  
      
        /* Sector 3 calculation(s) */
        /*  psOut -> f16A = 1 + alpha * sqrt(3) / 2 - beta / 2
            psOut -> f16C = PhaseA - alpha * sqrt(3) / 2 - beta / 2
            psOut -> f16B = PhaseC + beta */
         
        f32temp = INT16_MAX;              
        psOut -> f16B = f32temp;                                    /* psOut -> f16B = 1 */ 
        f32temp += f32A;                                            /* temp = 1 + alpha * sqrt(3) / 2 - beta / 2 */
        f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp;     /* high saturation */
        psOut -> f16A = (f32temp < 0) ? 0 : f32temp;                /* limits output and store phase A */
        f32temp += f32B;                                            /* temp = temp - alpha * sqrt(3) / 2 - beta / 2  */
        f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp;     /* high saturation */
        psOut -> f16C = (f32temp < 0) ? 0 : f32temp;                /* limits output and store phase C */
        return((uint16_t)0x3);                                      /* return sector 3 */
    }

    /*** Sector 5 calculation(s) ***/
    
    /* B > 0 and A > 0, the sector 5 is calculated */ 
    /*  pOut -> f16B = 1 + beta
        pOut -> f16A = PhaseB + alpha * sqrt(3) / 2 - beta / 2
        pOut -> f16C = PhaseA - alpha * sqrt(3) / 2 - beta / 2   */  
    
    f32temp = INT16_MAX;              
    psOut -> f16C = f32temp;                                        /* psOut -> f16C = 1 */ 
    f32temp += f32Beta;                                             /* temp = temp + beta */    
    f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp;         /* high saturation */
    psOut -> f16B  = (f32temp < 0) ? 0 : f32temp;                   /* limits output and store phase B */
    f32temp += f32A;                                                /* temp = temp + alpha * sqrt(3) / 2 - beta / 2 */        
    f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp;         /* high saturation */
    psOut -> f16A = (f32temp < 0)  ? 0 : f32temp;                   /* limits output and store phase A */   
    return((uint16_t)0x5);                                          /* return sector 2 */     
}

/***************************************************************************//*!
*
* @brief  Standard Discontinuous PWM
*
* @param  ptr - GMCLIB_2COOR_ALBE_T_F16 *psIn - IN - Direct(alpha) and quadrature(beta)
*               component of the stator voltage vector in the stationary reference
*               in frac16_t <-1;1) format.
*             - GMCLIB_3COOR_T_F16 *psOut - OUT - Pointer to duty cycle of the A, B and C
*               phases in frac16_t <-1;1) format.
*
* @return This function returns - sector number of the stator voltage vector resides in.
*
* @remarks  The function calculates the appropriate duty cycles needed to generate
*           a given stator reference voltage using the Standard Discontinuous PWM modulation,
*           offering lower switching losses but at low modulation indexes it causes higher
*           distortion and more complicated current sensing.
*
*         The function fills variables a, b and c in the data structure psOut
*         and returns the sector number of the stator voltage vector resides in.
*
*
*    Function calculations:
*
*    Inverse Clarke Transformation:
*    a =  u_alpha
*    b = -u_alpha/2 + sqrt(3)*u_beta/2
*    c = -u_alpha/2 - sqrt(3)*u_beta/2
*
*    Sector identification and PWM calculation:
*              c <= 0                      c > 0
*        b > 0         b <= 0        b <= 0      b <= 0
*    a <= 0 a > 0                            a <= 0  a > 0
*      6      1          2             5       4       3
*
*    Modified Park Transformation:
*    uref_1 =  u_beta
*    uref_2 =  sqrt(3)*u_alpha/2 + u_beta/2
*    uref_3 = -sqrt(3)*u_alpha/2 - u_beta/2
*
*    Partition identification and PWM calculation:
*                       uref_1 >= 0                             uref_1 < 0
*            uref_2 < 0         uref_2 >= 0            uref_2 < 0       uref_2 >= 0
*    uref_3 >= 0   uref_3 < 0                  uref_3 >= 0   uref_3 < 0
*           6             1            2              5             4          3
*
*    Sector identification:
*    Partition     1          2          3           4            5            6
*    pwm_a      1          0-u_ref3   1+u_ref2    0            1-u_ref3     0+u_ref2
*    pwm_b      1-u_ref2   0+u_ref1   1           0-u_ref2     1+u_ref1     0
*    pwm_c      1+u_ref3   0          1-u_ref1    0+u_ref3     1            0-u_ref1
*
*******************************************************************************/
uint16_t GMCLIB_SvmDpwm_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psIn,
                               GMCLIB_3COOR_T_F16 *psOut)
{
    GMCLIB_3COOR_T_F16 sUABC;
    register frac32_t f32TempA;
    register frac32_t f32TempB;
    register frac32_t f32Temp1;
    register frac32_t f32Temp2;
    register frac32_t f32Temp;
    register uint16_t u16Sector;

    /* Store values for calculation */
    f32TempA = psIn->f16Beta; /* Store to f32TempA will be used as Uref1 = beta */

    /* Calling Inverse Clark function */
    GMCLIB_ClarkInv_F16(psIn, &sUABC);

    /* Modified Inverse Clark function */
    /* Store constant beta * sqrt(3) / 2 shifted right to f16 range as f32Temp1 */
    f32Temp1 = ((frac16_t)(((int32_t)(psIn->f16Alpha)*(int32_t)(GMCLIB_SQRT3_DIV_2_F16))>>15));
    /* Store constant -0.5*beta to f32Temp2 */
    f32Temp2 = -(f32TempA >> 1);

    /* Calculate Uref1 coordinate = psIn->f16Beta [Uref1 = beta]  */

    /* Calculate Uref2 coordinate [Uref2 = +alpha*sqrt(3)/2 - beta/2  ] */
    f32TempB = f32Temp1 + f32Temp2;
    /* Result limitation to 16-bit range for Uref2 */
    f32TempB = (f32TempB < INT16_MIN) ? INT16_MIN : f32TempB;
    f32TempB = (f32TempB > INT16_MAX) ? INT16_MAX : f32TempB;

    /* Calculate Uref3 coordinate [Uref3 = -alpha*sqrt(3)/2 - beta/2  ] */
    f32Temp = -f32Temp1 + f32Temp2;
    /* Result limitation to 16-bit range for Uref3 */
    f32Temp = (f32Temp < INT16_MIN) ? INT16_MIN : f32Temp;
    f32Temp = (f32Temp > INT16_MAX) ? INT16_MAX : f32Temp;

    /* Sector identification and PWM calculation */
    /*               c <= 0                      c > 0                   */
    /*         b > 0         b <= 0        b <= 0      b <= 0            */
    /*     a <= 0 a > 0                            a <= 0  a > 0         */
    /*       6      1          2             5       4       3           */

    if (f32Temp <= 0)
    {   if (f32TempB > 0)
        {   if (f32TempA <= 0)
            {   u16Sector=0x6;                 /* return sector 6 */
            }
            else
            {   u16Sector=0x1;                 /* return sector 1 */
            }
        }
        else
        {   u16Sector=0x2;                     /* return sector 2 */
        }
    }
    else
    {   if (f32TempB <= 0)
        {   if (f32TempA <= 0)
            {   u16Sector=0x4;                 /* return sector 4 */
            }
            else
            {   u16Sector=0x3;                 /* return sector 3 */
            }
        }
        else
        {   u16Sector=0x5;                     /* return sector 5 */
        }
    }

    /* Partition identification and PWM calculation */
    /*               a >= 0                      a < 0                   */
    /*         b < 0         b >= 0        b < 0       b >= 0            */
    /*     c >= 0 c < 0                            c >= 0  c < 0         */
    /*       6      1          2             5       4       3           */

    if (sUABC.f16A >= 0)
    {   if (sUABC.f16B < 0)
        {   if (sUABC.f16C >= 0)
            {                                                              /* sector 6 calculations */
                psOut -> f16A = (f32TempB < 0) ? 0 : f32TempB;             /* pwm_a = 0+Uref2, limits output,store pwm_a */
                psOut -> f16B = 0;                                         /* store pwm_b  */
                f32Temp1 = -f32TempA;                                      /* pwm_c = 0 - Uref1*/
                f32Temp1 = (f32Temp1 > INT16_MAX)? INT16_MAX : f32Temp1;   /* high saturation */
                psOut -> f16C = (f32Temp1 < 0) ? 0 : f32Temp1;             /* limits output and store pwm_c  */
                return((uint16_t)u16Sector);                               /* return sector determined previously */
            }
            else
            {                                                              /* sector 1 calculations */
                f32Temp1 = INT16_MAX;                                      /* pwm_a = FRAC16(1) */
                psOut -> f16A = f32Temp1;                                  /* store pwm_a  */
                f32Temp2 = f32Temp1 - f32TempB;                            /* pwm_b = FRAC16(1) - Uref2*/
                f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
                psOut -> f16B = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_b  */
                f32Temp2 = f32Temp1 + f32Temp;                             /* pwm_c = FRAC16(1) + Uref3*/
                f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
                psOut -> f16C = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_c  */
                return((uint16_t)u16Sector);                               /* return sector determined previously */
            }
        }
        else
        {                                                              /* sector 2 calculations */
            f32Temp1 = -f32Temp;                                       /* pwm_a = 0 - Uref3*/
            f32Temp1 = (f32Temp1 > INT16_MAX)? INT16_MAX : f32Temp1;   /* limits output and high saturation */
            psOut -> f16A = (f32Temp1 < 0) ? 0 : f32Temp1;             /* limits output and store pwm_a */
            psOut -> f16B = (f32TempA < 0) ? 0 : f32TempA;             /* pwm_b = 0+Uref1,limits output, store pwm_b */
            psOut -> f16C = 0;                                         /* store pwm_c  */
            return((uint16_t)u16Sector);                               /* return sector determined previously */
        }
    }
    else
    {   if (sUABC.f16B >= 0)
        {   if (sUABC.f16C >= 0)
            {                                                              /* sector 4 calculations */
                f32Temp1 = -f32TempB ;                                     /* pwm_b = 0 - Uref2*/
                f32Temp1 = (f32Temp1 > INT16_MAX)? INT16_MAX : f32Temp1;   /* limits output and high saturation */
                psOut -> f16B = (f32Temp1 < 0) ? 0 : f32Temp1;             /* limits output and store pwm_b */
                psOut -> f16C = (f32Temp < 0) ? 0 : f32Temp;               /* pwm_c = 0+Uref3,limits output, store pwm_c  */
                psOut -> f16A = 0;                                         /* store pwm_a  */
                return((uint16_t)u16Sector);                               /* return sector determined previously */
            }
            else
            {                                                              /* sector 3 calculations */
                f32Temp1 = INT16_MAX ;                                     /* pwm_b = FRAC16(1) */
                psOut -> f16B = f32Temp1;                                  /* store pwm_b  */
                f32Temp2 = f32Temp1 - f32TempA;                            /* pwm_c = FRAC16(1) - Uref1*/
                f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
                psOut -> f16C = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_c  */
                f32Temp2 = f32Temp1 + f32TempB;                            /* pwm_a = FRAC16(1) + Uref2*/
                f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
                psOut -> f16A = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_a  */
                return((uint16_t)u16Sector);                               /* return sector determined previously */
            }
        }
        else
        {                                                              /* sector 5 calculations */
            f32Temp1 = INT16_MAX ;                                     /* pwm_c = FRAC16(1) */
            psOut -> f16C = f32Temp1;                                  /* store pwm_c  */
            f32Temp2 = f32Temp1 + f32TempA;                            /* pwm_b = FRAC16(1) + Uref1*/
            f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
            psOut -> f16B = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_c  */
            f32Temp2 = f32Temp1 - f32Temp;                             /* pwm_a = FRAC16(1) - Uref3*/
            f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
            psOut -> f16A = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_a  */
            return((uint16_t)u16Sector);                               /* return sector determined previously */
        }
    }
}

/***************************************************************************//*!
*
* @brief  Extended Discontinuous PWM
*
* @param  ptr - GMCLIB_2COOR_ALBE_T_F16 *psIn - IN - Direct(alpha) and quadrature(beta)
*               component of the stator voltage vector in the stationary reference
*               in frac16_t <-1;1) format.
*             - GMCLIB_2COOR_SINCOS_T_F16 *psAngle - IN - sine and cosine structure of
*               phase shift angle of the generated phase voltages and currents, angle (-PI/6;PI/6)
*               in frac it is angle (-1/6;1/6)
*             - GMCLIB_3COOR_T_F16 *psOut - OUT - Pointer to duty cycle of the A, B and C
*               phases in frac16_t <-1;1) format.
*
* @return This function returns - sector number of the stator voltage vector resides in.
*
* @remarks  The function calculates the appropriate duty cycles needed to generate
*           a given stator reference voltage using the Extended Discontinuous PWM,
*           offering lower switching losses, but at low modulation indexes it causes higher
*           distortion and more complicated current sensing. The function is a superset
*           of standard DPWM technique and includes Park transformation (to rotate input voltage
*           wector) which generates signals a*,b*,c* for partition definition signal.
*
*         The function fills variables a, b and c in the data structure psOut
*         and returns the sector number of the stator voltage vector resides in.
*
*    Function calculations:
*
*    Park Transformation:
*    u_alphaTmp = u_alpha*cos(fi) + u_beta*sin(fi)
*    u_betaTmp  =-u_alpah*sin(fi) + u_beta*cos(fi)
*
*    Inverse Clarke Transformation:
*    a =  u_alphaTmp
*    b = -u_alphaTmp/2 + sqrt(3)*u_betaTmp/2
*    c = -u_alphaTmp/2 - sqrt(3)*u_betaTmp/2
*
*    Sector identification and PWM calculation:
*              c <= 0                      c > 0
*        b > 0         b <= 0        b <= 0      b <= 0
*    a <= 0 a > 0                            a <= 0  a > 0
*      6      1          2             5       4       3
*
*    Modified Park Transformation:
*    uref_1 =  u_beta
*    uref_2 =  sqrt(3)*u_alpha/2 + u_beta/2
*    uref_3 = -sqrt(3)*u_alpha/2 - u_beta/2
*
*    Partition identification and PWM calculation:
*                       uref_1 >= 0                             uref_1 < 0
*            uref_2 < 0         uref_2 >= 0            uref_2 < 0       uref_2 >= 0
*    uref_3 >= 0   uref_3 < 0                  uref_3 >= 0   uref_3 < 0
*           6             1            2              5             4          3
*
*    Sector identification:
*    Partition     1          2          3           4            5            6
*    pwm_a      1          0-u_ref3   1+u_ref2    0            1-u_ref3     0+u_ref2
*    pwm_b      1-u_ref2   0+u_ref1   1           0-u_ref2     1+u_ref1     0
*    pwm_c      1+u_ref3   0          1-u_ref1    0+u_ref3     1            0-u_ref1
*
*******************************************************************************/
uint16_t GMCLIB_SvmExDpwm_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psIn,
                                  const GMCLIB_2COOR_SINCOS_T_F16 *psAngle,
                                  GMCLIB_3COOR_T_F16 *psOut)
{
    GMCLIB_2COOR_DQ_T_F16 sUDQ;
    GMCLIB_3COOR_T_F16 sUABC;
    register frac32_t f32TempA;
    register frac32_t f32TempB;
    register frac32_t f32Temp1;
    register frac32_t f32Temp2;
    register frac32_t f32Temp;
    register uint16_t u16Sector;

    /* Store values for calculation */
    f32TempA = psIn->f16Beta;   /* Store to f32TempA will be used as Uref1 = beta */

    /* Park Cos(fi) Transformation calculation - rotates input Us vector according power factor angle */
    GMCLIB_Park_F16(psIn, psAngle, &sUDQ);

    /* Calling Inverse Clark function */
    GMCLIB_ClarkInv_F16((GMCLIB_2COOR_ALBE_T_F16*)&sUDQ, &sUABC);

    /* Modified Inverse Clark function */
    /* Store constant beta * sqrt(3) / 2 shifted right to f16 range as f32Temp1 */
    f32Temp1 = ((frac16_t)(((int32_t)(psIn->f16Alpha)*(int32_t)(GMCLIB_SQRT3_DIV_2_F16))>>15));
    /* Store constant -0.5*beta to f32Temp2 */
    f32Temp2 = -(f32TempA >> 1);

    /* Calculate Uref1 coordinate = psIn->f16Beta [Uref1 = beta]  */

    /* Calculate Uref2 coordinate [Uref2 = +alpha*sqrt(3)/2 - beta/2  ] */
    f32TempB = f32Temp1 + f32Temp2;
    /* Result limitation to 16-bit range for Uref2 */
    f32TempB = (f32TempB < INT16_MIN) ? INT16_MIN : f32TempB;
    f32TempB = (f32TempB > INT16_MAX) ? INT16_MAX : f32TempB;

    /* Calculate Uref3 coordinate [Uref3 = -alpha*sqrt(3)/2 - beta/2  ] */
    f32Temp = -f32Temp1 + f32Temp2;
    /* Result limitation to 16-bit range for Uref3 */
    f32Temp = (f32Temp < INT16_MIN) ? INT16_MIN : f32Temp;
    f32Temp = (f32Temp > INT16_MAX) ? INT16_MAX : f32Temp;

    /* Sector identification and PWM calculation */
    /*               c <= 0                      c > 0                   */
    /*         b > 0         b <= 0        b <= 0      b <= 0            */
    /*     a <= 0 a > 0                            a <= 0  a > 0         */
    /*       6      1          2             5       4       3           */

    if (f32Temp <= 0)
    {   if (f32TempB > 0)
        {   if (f32TempA <= 0)
            {   u16Sector=0x6;                 /* return sector 6 */
            }
            else
            {   u16Sector=0x1;                 /* return sector 1 */
            }
        }
        else
        {   u16Sector=0x2;                     /* return sector 2 */
        }
    }
    else
    {   if (f32TempB <= 0)
        {   if (f32TempA <= 0)
            {   u16Sector=0x4;                 /* return sector 4 */
            }
            else
            {   u16Sector=0x3;                 /* return sector 3 */
            }
        }
        else
        {   u16Sector=0x5;                     /* return sector 5 */
        }
    }

    /* Partition identification and PWM calculation */
    /*               a >= 0                      a < 0                   */
    /*         b < 0         b >= 0        b < 0       b >= 0            */
    /*     c >= 0 c < 0                            c >= 0  c < 0         */
    /*       6      1          2             5       4       3           */

    if (sUABC.f16A >= 0)
    {   if (sUABC.f16B < 0)
        {   if (sUABC.f16C >= 0)
            {                                                              /* sector 6 calculations */
                psOut -> f16A = (f32TempB < 0) ? 0 : f32TempB;             /* pwm_a = 0+Uref2, limits output,store pwm_a */
                psOut -> f16B = 0;                                         /* store pwm_b  */
                f32Temp1 = -f32TempA;                                      /* pwm_c = 0 - Uref1*/
                f32Temp1 = (f32Temp1 > INT16_MAX)? INT16_MAX : f32Temp1;   /* high saturation */
                psOut -> f16C = (f32Temp1 < 0) ? 0 : f32Temp1;             /* limits output and store pwm_c  */
                return((uint16_t)u16Sector);                               /* return sector determined previously */
            }
            else
            {                                                              /* sector 1 calculations */
                f32Temp1 = INT16_MAX;                                      /* pwm_a = FRAC16(1) */
                psOut -> f16A = f32Temp1;                                  /* store pwm_a  */
                f32Temp2 = f32Temp1 - f32TempB;                            /* pwm_b = FRAC16(1) - Uref2*/
                f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
                psOut -> f16B = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_b  */
                f32Temp2 = f32Temp1 + f32Temp;                             /* pwm_c = FRAC16(1) + Uref3*/
                f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
                psOut -> f16C = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_c  */
                return((uint16_t)u16Sector);                               /* return sector determined previously */
            }
        }
        else
        {                                                              /* sector 2 calculations */
            f32Temp1 = -f32Temp;                                       /* pwm_a = 0 - Uref3*/
            f32Temp1 = (f32Temp1 > INT16_MAX)? INT16_MAX : f32Temp1;   /* limits output and high saturation */
            psOut -> f16A = (f32Temp1 < 0) ? 0 : f32Temp1;             /* limits output and store pwm_a */
            psOut -> f16B = (f32TempA < 0) ? 0 : f32TempA;             /* pwm_b = 0+Uref1,limits output, store pwm_b */
            psOut -> f16C = 0;                                         /* store pwm_c  */
            return((uint16_t)u16Sector);                               /* return sector determined previously */
        }
    }
    else
    {   if (sUABC.f16B >= 0)
        {   if (sUABC.f16C >= 0)
            {                                                              /* sector 4 calculations */
                f32Temp1 = -f32TempB ;                                     /* pwm_b = 0 - Uref2*/
                f32Temp1 = (f32Temp1 > INT16_MAX)? INT16_MAX : f32Temp1;   /* limits output and high saturation */
                psOut -> f16B = (f32Temp1 < 0) ? 0 : f32Temp1;             /* limits output and store pwm_b */
                psOut -> f16C = (f32Temp < 0) ? 0 : f32Temp;               /* pwm_c = 0+Uref3,limits output, store pwm_c  */
                psOut -> f16A = 0;                                         /* store pwm_a  */
                return((uint16_t)u16Sector);                               /* return sector determined previously */
            }
            else
            {                                                              /* sector 3 calculations */
                f32Temp1 = INT16_MAX ;                                     /* pwm_b = FRAC16(1) */
                psOut -> f16B = f32Temp1;                                  /* store pwm_b  */
                f32Temp2 = f32Temp1 - f32TempA;                            /* pwm_c = FRAC16(1) - Uref1*/
                f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
                psOut -> f16C = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_c  */
                f32Temp2 = f32Temp1 + f32TempB;                            /* pwm_a = FRAC16(1) + Uref2*/
                f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
                psOut -> f16A = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_a  */
                return((uint16_t)u16Sector);                               /* return sector determined previously */
            }
        }
        else
        {                                                              /* sector 5 calculations */
            f32Temp1 = INT16_MAX ;                                     /* pwm_c = FRAC16(1) */
            psOut -> f16C = f32Temp1;                                  /* store pwm_c  */
            f32Temp2 = f32Temp1 + f32TempA;                            /* pwm_b = FRAC16(1) + Uref1*/
            f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
            psOut -> f16B = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_c  */
            f32Temp2 = f32Temp1 - f32Temp;                             /* pwm_a = FRAC16(1) - Uref3*/
            f32Temp2 = (f32Temp2 > INT16_MAX) ? INT16_MAX : f32Temp2;  /* high saturation */
            psOut -> f16A = (f32Temp2 < 0) ? 0 : f32Temp2;             /* limits output and store pwm_a  */
            return((uint16_t)u16Sector);                               /* return sector determined previously */
        }
    }
}
#ifdef  RSIC_V   
/***************************************************************************//*!
*
* @brief    Standard SVM function
*
* @param    ptr  r0 - GMCLIB_2COOR_ALBE_T_F16 *psIn
*                     IN: Direct(alpha) and quadrature(beta) component
*                     of the stator voltage vector in the stationary reference frame.
*                     Format frac16_t, range 8000-7FFF
*                r1 - GMCLIB_3COOR_T_F16 *psOut
*                     OUT: Pointer to duty cycle of the A, B and C phases.
*                     Format frac16_t, range 8000-7FFF
*
* @return   This function returns
*                r0 - sector number of the stator voltage vector resides in.
*
* @remarks  The function calculates the appropriate duty cycles needed to generate
*           a given stator reference voltage using the Space Vector Modulation with
*           a duty cycle of the null switching state from states O000 and O111
*           in each sector of the hexagon.
*
*           The function fills variables a, b and c in the data structure psOut
*           and returns the sector number of the stator voltage vector resides in.
*
*            X = ubeta
*            Y = 1/2(ubeta - sqrt(3)*ualfa)
*            Z = 1/2(ubeta + sqrt(3)*ualfa)
*
*            Sectors   U0,U60   U60,U120   U120,U180   U180,U240   U240,U300   U300,U0
*            t_1       X        Y          -Y          Z           -Z          -X
*            t_2       -Z       Z          X          -X           -Y          Y
*
*            t1 = (T - t_1 - t_2) / 2
*            t2 = t1 + t_1
*            t3 = t2 + t_2
*
*******************************************************************************/
uint16_t GMCLIB_SvmStd_F16_FAsm(const GMCLIB_2COOR_ALBE_T_F16 *psIn,
                                       GMCLIB_3COOR_T_F16 *psOut)
{
#if 0
    //frac16_t A,B,C ;
    frac16_t A,B ;
    uint16_t ret =0 ;
    A = sqrtf(3.0F)/2 * psIn->f16Alpha - psIn->f16Beta/2  ;
    B = -psIn->f16Alpha * sqrtf(3.0f)/2 - psIn->f16Beta/2  ;
   // C = psIn->f16Alpha * sqrtf(3.0f)/2 ;
    if(B<=0 && A>0 && psIn->f16Beta>0)  /* B <= 0, A > 0, beta > 0 1sf sector */ 
    {
        psOut -> f16C = (0x8000 +B) / 2  ;
        psOut -> f16B = psOut -> f16C + psIn->f16Beta ;
        psOut -> f16A = psOut -> f16B + A ;
        ret = 1 ;
    }
    else if(B<=0 && A<=0) /* B <= 0, A <= 0 2en sector */ 
    {
        psOut -> f16C = (0x8000 - psIn->f16Beta) / 2 ;
        psOut -> f16A = psOut -> f16C - B ;
        psOut -> f16B = psOut -> f16A - A ;
        ret = 2 ;
    }
    else if(B>0 && A<=0 && psIn->f16Beta >0)/*  B > 0, A <= 0, beta > 0  3rd sector */ 
    {
        psOut -> f16A = ( 0x8000 + A ) / 2;
        psOut -> f16C = psOut -> f16A + B;
        psOut -> f16B = psOut -> f16C + psIn->f16Beta;
        ret =3 ;
    }
    else if(B>0 && A<=0 && psIn->f16Beta <= 0)/* B > 0, A <= 0, beta <= 0  4th sector*/
    {
        psOut -> f16A =  (0x8000 - B ) / 2;
        psOut -> f16B = psOut -> f16A - A;
        psOut -> f16C = psOut -> f16B - psIn->f16Beta;
        ret = 4 ;
    }
    else if(B>0 && A>0 )  /* B > 0, A > 0 5th sector*/
    {
        psOut -> f16A = ( 0x8000 + A ) / 2;
        psOut -> f16C = psOut -> f16A + B;
        psOut -> f16B = psOut -> f16C + psIn->f16Beta;
        ret =5 ;
    }
    else if(B<=0 && A>0 && psIn->f16Beta <=0 )/* B <= 0, A > 0, beta <= 0 6th sector*/
    {
        psOut -> f16A =  ( 0x8000 + A ) / 2;
        psOut -> f16C = psOut -> f16A + B;
        psOut -> f16B = psOut -> f16C + psIn->f16Beta;
        ret =6 ;
    }
    return ret ;
#else
    register frac32_t f32Sqrt3DivBy2;   
    register frac32_t f32A, f32B, f32C;
    register frac32_t f32temp;
    register frac32_t f32Alpha,f32Beta,f32BetaDiv2;
    
    f32Alpha = psIn->f16Alpha;
    f32Beta = psIn->f16Beta;
    f32BetaDiv2 = f32Beta >> 1;
    f32Sqrt3DivBy2 = GMCLIB_SQRT3_DIV_2_F16;
    
    /* 
    f32AlphaMulSqrt3Half =  f32Alpha * sqrt(3)/2
    f32A =  f32Alpha * sqrt(3)/2 - beta/2
    f32B = -f32Alpha * sqrt(3)/2 - beta/2
    */    
    f32C = (f32Alpha * f32Sqrt3DivBy2) >> 15 ; 
    f32A =  f32C - f32BetaDiv2 ; 
    f32B = -f32C - f32BetaDiv2 ;  /* -f32B = f32C + f32BetaDiv2 */
    
     /* Sector determination: if B > 0 and A > 0, the sector 5 is calculated */
    if (f32B <= 0)
    {
        /*** Sector 2 calculation(s) ***/
        if (f32A <= 0 )            
        { 
    /* psOut -> f16C = 1 - beta
       psOut -> f16A = PhaseC + alpha * sqrt(3) / 2 + beta / 2
       psOut -> f16B = PhaseA - alpha * sqrt(3) / 2 + beta / 2   */      
            
            f32temp = INT16_MAX;
  
            f32temp -= f32Beta;  /* temp = 1 - beta  */
            f32temp = f32temp>>1; /* temp = temp /2 */;                                  
         //   f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16C = f32temp;                                /* limits output and store phase C */   
            psOut ->f16A =  psOut -> f16C - f32B ;                   
            psOut -> f16B = psOut ->f16A  -f32A;
            return((uint16_t)0x2);                                  /* return sector 2 */
        }  
        
        /*** Sector 6 calculation(s) ***/
        if (f32Beta <= 0)        
        {
    /*  psOut -> f16B = 1 - alpha * sqrt(3) / 2 + beta / 2
        psOut -> f16C = PhaseB - beta
        psOut -> f16A = PhaseC + alpha * sqrt(3) / 2 + beta / 2    */
          
          #if 0  
            f32temp = INT16_MAX;
            psOut -> f16A = f32temp;                                /* psOut -> f16A = 1 */  
            f32temp -= f32A;                                        /* temp = temp - alpha * sqrt(3) / 2 + beta / 2 */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16B = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase B */
            f32temp -= f32Beta ;                                    /* temp = temp - beta  */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16C = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase C */
            return((uint16_t)0x6);                                  /* return sector 6 */
          #else
            f32temp = INT16_MAX - f32A;
            f32temp = f32temp >>1 ;
            psOut -> f16B = f32temp ;
            psOut -> f16C = psOut -> f16B -f32Beta ;
            psOut -> f16A = psOut -> f16C - f32B ;
           // f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            return((uint16_t)0x6);   
          #endif
        }  
      
    /* Sector 1 calculation(s) */  
    /* psOut -> f16C = 1 - alpha * sqrt(3) / 2 - beta / 2
       psOut -> f16B = PhaseC + beta
       psOut -> f16A = PhaseB + alpha * sqrt(3) / 2 - beta / 2  */
    
        f32temp = INT16_MAX;              
        f32temp += f32B ;
        f32temp =f32temp>>1 ;
        psOut -> f16C = f32temp ;
        psOut -> f16B =  psOut -> f16C + f32Beta ;
        psOut -> f16A =  psOut ->f16B + f32A ;
        return((uint16_t)0x1);                                          /* return sector 1 */
    }  
  
    if (f32A <= 0) 
    {
        /* Sector 4 calculation(s) */
        if (f32Beta <= 0) 
        /*  psOut -> f16A = 1 + alpha * sqrt(3) / 2 + beta / 2
            psOut -> f16B = PhaseA - alpha * sqrt(3) / 2 + beta / 2
            psOut -> f16C = PhaseB - beta */
        {
            f32temp = INT16_MAX; 
            f32temp = f32temp - f32B ;
            f32temp = f32temp>>1 ;
            psOut ->f16A = f32temp ;
            psOut ->f16B =  psOut ->f16A - f32A ;
            psOut ->f16C =  psOut ->f16B - f32Beta ;          
            return((uint16_t)0x4);                                  /* return sector 4 */
        }  
      
        /* Sector 3 calculation(s) */
        /*  psOut -> f16A = 1 + alpha * sqrt(3) / 2 - beta / 2
            psOut -> f16C = PhaseA - alpha * sqrt(3) / 2 - beta / 2
            psOut -> f16B = PhaseC + beta */
         
        f32temp = INT16_MAX;    
        f32temp += f32A ;
        f32temp = f32temp >>1 ;
        psOut ->f16A = f32temp ;
        psOut ->f16C = psOut ->f16A + f32B ;
        psOut ->f16B =  psOut ->f16C +f32Beta ;       
        return((uint16_t)0x3);                                      /* return sector 3 */
    }

    /*** Sector 5 calculation(s) ***/
    
    /* B > 0 and A > 0, the sector 5 is calculated */ 
    /*  pOut -> f16B = 1 + beta
        pOut -> f16A = PhaseB + alpha * sqrt(3) / 2 - beta / 2
        pOut -> f16C = PhaseA - alpha * sqrt(3) / 2 - beta / 2   */  
    
    f32temp = INT16_MAX;   
    f32temp +=f32Beta ;
    f32temp = f32temp >>1 ;
    psOut ->f16B = f32temp ;
    psOut -> f16A =  psOut ->f16B + f32A ;
    psOut -> f16C =  psOut -> f16A +f32B ;         
    return((uint16_t)0x5); 
#endif
}
uint16_t GMCLIB_SvmStd2_F16_FAsm(const GMCLIB_2COOR_ALBE_T_F16 *psIn,
                        GMCLIB_3COOR_T_F16 *psOut)
{

    psOut->f16A = (0x8000 - psIn->f16Alpha)>>1 ;
    psOut->f16B = (0x8000 - psIn->f16Beta)>>1 ;

    return 0; 

}
#endif