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
* @brief  Parallel PID controller with anti-wind-up protection
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(GFLIB_CtrlPIDpAW_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

/***************************************************************************//*!
*
* @brief  16-bit input and output parallel form of PID controller with
*         anti-wind-up protection and separated PI and D error inputs
*
* @param  ptr        r3 - pointer to the structure of the PI controller
*                         GFLIB_CTRL_PID_P_AW_T_A32 data structure:
*                           acc32_t a32PGain
*                               - proportional gain; input parameter
*                                 in the following range
*                                 0 <= f16PGain < 65536.0
*                           acc32_t a32IGain
*                               - integral gain; input parameter
*                                 in the following range
*                                 0 <= f16IGain < 65536.0
*                           acc32_t a32DGain
*                               - derivative gain; input parameter
*                                 in the following range:
*                                 0 <= f16DGain < 65536.0
*                           frac32_t f32IAccK_1
*                               - state variable; fractional part (lower long)
*                                 of integral part at step k-1;
*                                 can be modified outside of the function;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f16PropGain < 1.0
*                           frac16_t f16InErrK_1
*                               - error at the step k-1;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f16PropGain < 1.0
*                           frac16_t f16UpperLim
*                               - upper limit of the controller output;
*                                 input parameter
*                                 in the following range
*                                 -1.0 <= f16PropGain < 1.0
*                                 f16UpperLim > f16LowerLim
*                           frac16_t f16LowerLim
*                               - lower limit of the controller output;
*                                 input parameter
*                                 in the following range
*                                 -1.0 <= f16PropGain < 1.0
*                                 f16UpperLim > f16LowerLim
*                           frac16_t f16InErrDK_1
*                               - derivative input's error at the step k-1;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f16InErrDK_1 < 1.0
*                           bool_t bLimFlag
*                               - limitation flag; if set, the controller
*                                 output reached either f16UpperLim or f16LowerLim,
*                                 otherwise it is cleared
*                                 output parameter
*
*                    r2 - pointer to a bool_t variable variable; if the pointed variable
*                         is false, then the integral part is limited only by
*                         the PI controller limits.
*                         If the pointed variable is true, then
*                         the integral part is frozen immediately.
*                         The f16InErrK_1 keeps updated although the
*                         pointed flag is true.
*
* @param  in         r0 - input error of the controller at step K processed
*                         by P and I terms of the PID algorithm
*                         fractional value in the range <-1;1)
*                    r1 - input error of the controller at step K processed
*                         by D terms of the PID algorithm
*                         fractional value in the range <-1;1)
*
* @return This function returns: f16LowerLim <= result <= f16UpperLim
*                    r0 - frac16_t value <-1;1)

* @remarks      PI controller algorithm:
*
*                        -----------
*                 e(k)  |           |  u(k)
*               --------|    PID    |---------
*               --------|           |
*                ed(k)  |           |
*                        -----------
*
*       e(t) - input error in continuous time domain, processed by P and I components of PID algorithm
*       ed(t) - input error in continuous time domain,processed by D component of PID algorithm
*       u(t) - controller output in continuous time domain
*
*       e(k) - input error at step k - discrete time domain, processed by P and I components
*       ed(k) - input error at step k - discrete time domain, processed by D component
*       u(k) - controller output at step k - discrete time domain
*
*       e_max - max range of e(k)
*       u_max - max range of u(k)
*       ed_max = e_max
*
*       The PID controller algorithm in continuous time domain:
*                              /t
*       u(t) = K[e(t) + 1/Ti * | e(t)*dt + Td(ded/dt]
*                              /0
*       K  - controller gain
*       Ti - integral time constant
*       Td - derivative time constant
*
*       PID controller expressed in fractional arithmetic:
*
*       u_f(k) = K_sc * e_f(k) + ui_f(k - 1) + Ki_sc * e_f(k) + Kd_sc(e_d_f(k) - e_d_f(k_1))
*
*       e_f(k) = e(k)/e_max
*       e_d_f(k) = e_d_f(k)/ed_max
*       u_f(k) = u(k)/u_max
*
*       K_sc =  K * e_max/u_max
*       Ki_sc = K * T/Ti * e_max/u_max
*       Kd_sc = K * Td/T * ed_max/u_max
*
*       T - sampling time/period
*
*       a32PGain = K_sc * 2 ^ 15
*
*       a32IGain = Ki_sc * 2 ^ 15
*
*       a32DGain = Kd_sc * 2 ^ 15
*
****************************************************************************/
ASM_LABEL(GFLIB_CtrlPIDpAW_F16_FAsm)
    push    {r4 - r7}
    ldr     r6, [r3, #4]        /* r6 = a32IGain */
    ldr     r7, [r3, #12]       /* r7 = f32IAccK_1 */

    /* Integral part calculation */

    ldrsh   r5, [r3, #16]       /* r5 = f16InErrK_1 */
    ldrh    r4, [r2]            /* r4 = pbStopIntegFlag */
    cmp     r4, #0              /* If stop flag is set integration will be stopped */
    ittee   eq
    addeq   r5, r5, r0          /* r5 = f16InErr + f16InErrK_1 */
    strheq  r0, [r3, #16]       /* f16InErrK_1 = r0 */
    movne   r4, #0              /* r4 = 0 */
    strhne  r4, [r3, #16]       /* f16InErrK_1 = r4 */

    asr     r4, r7, #31         /* r4 = sign of r7 */
    smlal   r7, r4, r5, r6      /* r4:r7 = f32IAccK_1 + (f16InErr + f16InErrK_1) * a32IGain */

    /* Integral part limitation */
    lsr     r2, r7, #17         /* r2 = r7 >> 17 */
    add     r2, r2, r4, lsl #15 /* r2 = r4 << 15 + r2 */
    qadd    r2, r2, r2          /* r2 = r2 + r2 with saturation */
    lsl     r5, r7, #15         /* r5 = lower 15 bits of r7 */
    it      ne                  /* If r5 != 0, then r1 = r1 + 1 */
    addne   r2, r2, #1          /* r2 = r2 + 1 */

    ldrsh   r5, [r3, #18]       /* r5 = f16UpperLim */
    ldrsh   r6, [r3, #20]       /* r6 = f16LowerLim */

    cmp     r2, r5              /* Compares I term with UpperLim */
    it      gt                  /* Condition block */
    lslgt   r7, r5, #16         /* r7 = r5 << 16 */

    cmp     r2, r6              /* Compares I term with LowerLim */
    it      lt                  /* Condition block */
    lsllt   r7, r6, #16         /* r7 = r6 << 16 */

    str     r7, [r3, #12]       /* f32IAccK_1 = r7 */

    /* Proportional part calculation */
    ldr     r2, [r3]            /* r2 = a32PGain */
    asr     r4, r7, #31         /* r4 = sign of r7 */
    lsl     r0, r0, #1          /* r0 << 1 */
    smlal   r7, r4, r2, r0      /* r4:r7 = r4:r7 + r5 * r0 */

    /* Derivative part calculation */
    ldrsh   r0, [r3, #22]       /* r0 = f16InErrDK_1 */
    sub     r0, r1, r0          /* r0 = f16InErrD - f16InErrDK_1 */
    lsl     r0, r0, #1          /* r0 << 1 */
    ldr     r2, [r3, #8]        /* r2 = a32DGain */
    smlal   r7, r4, r2, r0      /* r4:r7 = r4:r7 + r2 * r0 */
    strh    r1, [r3, #22]       /* f16InErrDK_1 = r1 */

    /* Output limitation */ 
    ssat    r4, #16, r4         /* Saturates r4 to 17 bit value*/
    lsr     r0, r7, #16         /* r0 = r7 >> 17; Sets Carry */
    add     r0, r0, r4, lsl #16 /* r0 = r0 + (r4 << 15) + Carry */
    mov     r4, #0              /* r4 = 0 */

    cmp     r0, r5              /* Compares Result with UpperLim */
    itt     ge                  /* Condition block */
    movge   r0, r5              /* r0 = r5 */
    movge   r4, #1              /* r4 = 1 */

    cmp     r0, r6              /* Compares Result with LowerLim */
    itt     le                  /* Condition block */
    movle   r0, r6              /* r0 = r6 */
    movle   r4, #1              /* r4 = 1 */

    strh    r4, [r3, #24]       /* bLimFlag = r4 */

    pop     {r4 - r7}
    bx      lr                  /* Returns */

    ASM_END
