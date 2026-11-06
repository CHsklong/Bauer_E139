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
* @brief  Parallel PI controller with and without anti-wind-up protection
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(GFLIB_CtrlPIpAW_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

/***************************************************************************//*!
*
* @brief  16-bit input and output parallel form of PI controller with
*         anti-wind-up protection
*
* @param  ptr  r2 - pointer to the structure of the PI controller
*
*                  GFLIB_CTRL_PI_P_AW_T_A32 data structure:
*                    acc32_t a32PGain    
*                        - proportional gain; input parameter
*                          in the following range
*                          0 <= f16PGain < 65536.0  
*                    acc32_t a32IGain
*                        - integral gain; input parameter
*                          in the following range
*                          0 <= f16IGain < 65536.0    
*                    frac32_t f32IAccK_1
*                        - state variable; fractional part (lower long)
*                          of integral part at step k-1;
*                          can be modified outside of the function;
*                          input/output parameter
*                          in the following range
*                          -1.0 <= f32IAccK_1 < 1.0  
*                    frac16_t f16InErrK_1
*                          error at the step k-1;
*                          input/output parameter
*                          in the following range
*                          -1.0 <= f16InErrK_1 < 1.0   
*                    frac16_t f16UpperLim
*                        - upper limit of the controller output;
*                          input parameter
*                          in the following range
*                          -1.0 <= f16UpperLim < 1.0  
*                          f16UpperLim > f16LowerLim
*                    frac16_t f16LowerLim
*                        - lower limit of the controller output;
*                          input parameter
*                          in the following range
*                          -1.0 <= f16LowerLim < 1.0   
*                          f16UpperLim > f16LowerLim
*                    bool_t bLimFlag
*                        - limitation flag; if set, the controller
*                          output reached either f16UpperLim or f16LowerLim,
*                          otherwise it is cleared
*                          output parameter 
*
*              r1 - Pointer to a bool_t variable variable; if the pointed variable
*                   is false, then the integral part is limited only by
*                   the PI controller limits.
*                   If the pointed variable is true, then
*                   the integral part is frozen immediately.
*                   The f16InErrK_1 keeps updated although the
*                   pointed flag is true.
*                    
* @param  in   r0 - Input error of the controller at step K processed by P and I terms
*                   of the PI algorithm fractional value in the range <-1;1)
*                                                  
*
* @return      This function returns: f16LowerLim <= result <= f16UpperLim
*              r0 - frac16_t value <-1;1)

* @remarks     PI controller algorithm:
*       e(t) - input error in continuous time domain                    
*       u(t) - controller output in continuous time domain              
*                                                                       
*       e(k) - input error at step k - discrete time domain             
*       u(k) - controller output at step k - discrete time domain                                                 
*                                                                       
*       The PI controller algorithm in continuous time domain:          
*                              /t                                       
*       u(t) = K[e(t) + 1/Ti * | e(t)*dt]                            
*                              /0     
*       The PI controller algorithm in discrete time domain:  
*       u(k) = Kpf*e(k) + ui(k-1) + Kif*(e(k)+e(k-1))/2  
*                                        
*       K  - controller gain                                            
*       Ti - integral time constant                                     
*                                                                       
*******************************************************************************/ 
ASM_LABEL(GFLIB_CtrlPIpAW_F16_FAsm)
    push    {r4 - r7}
    ldm     r2, {r5, r6, r7}    /* r5 = a32PGain, r6 = a32IGain, r7 = f32IAccK_1 */

    /* Integral part calculation */

    ldrsh   r3, [r2, #12]       /* r3 = f16InErrK_1 */
    ldrh    r4, [r1]            /* r4 = pbStopIntegFlag */
    cmp     r4, #0              /* If stop flag is set integration will be stopped */
    ittee   eq
    addeq   r3, r3, r0          /* r3 = f16InErr + f16InErrK_1 */
    strheq  r0, [r2, #12]       /* f16InErrK_1 = r0 */
    movne   r4, #0              /* r4 = 0 */
    strhne  r4, [r2, #12]       /* f16InErrK_1 = r4 */    

    asr     r4, r7, #31         /* r4 = sign of r7 */
    smlal   r7, r4, r3, r6      /* r4:r7 = f32IAccK_1 + (f16InErr + f16InErrK_1) * a32IGain */

    /* Integral part limitation */
    lsr     r1, r7, #17         /* r1 = r7 >> 17 */
    add     r1, r1, r4, lsl #15 /* r1 = r4 << 15 + r1 */
    qadd    r1, r1, r1          /* r1 = r1 + r1 with saturation */
    lsl     r3, r7, #15         /* r3 = lower 17 bits of r7 */       
    it      ne                  /* If r3 != 0, then r1 = r1 + 1 */
    addne   r1, r1, #1          /* r1 = r1 + 1 */

    ldrsh   r3, [r2, #14]       /* r3 = f16UpperLim */
    ldrsh   r6, [r2, #16]       /* r6 = f16LowerLim */

    cmp     r1, r3              /* Compares I term with UpperLim */
    it      gt                  /* Condition block */
    lslgt   r7, r3, #16         /* r7 = r3 << 16 */

    cmp     r1, r6              /* Compares I term with LowerLim */
    it      lt                  /* Condition block */
    lsllt   r7, r6, #16         /* r7 = r6 << 16 */

    str     r7, [r2, #8]        /* f32IAccK_1 = r7 */

    /* Proportional part calculation */
    asr     r4, r7, #31         /* r4 = sign of r7 */
    lsl     r0, r0, #1          /* r0 << 1 */
    smlal   r7, r4, r5, r0      /* r4:r7 = r4:r7 + r5 * r0 */

    /* Output limitation */
    ssat    r4, #16, r4         /* Saturates r4 to 16 bit value*/
    lsr     r0, r7, #16         /* r0 = r7 >> 16 */
    add     r0, r0, r4, lsl #16 /* r0 = r0 + (r4 << 16) + Carry*/
    mov     r4, #0              /* r4 = 0 */

    cmp     r0, r3              /* Compares Result with UpperLim */
    itt     ge                  /* Condition block */
    movge   r0, r3              /* r0 = r3 */
    movge   r4, #1              /* r4 = 1 */

    cmp     r0, r6              /* Compares Result with LowerLim */
    itt     le                  /* Condition block */
    movle   r0, r6              /* r0 = r6 */
    movle   r4, #1              /* r4 = 1 */

    strh    r4, [r2, #18]       /* bLimFlag = r4 */

    pop     {r4 - r7}
    bx      lr                  /* Returns */

    ASM_END
