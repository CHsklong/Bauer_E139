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
* @brief  Tracking observer
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(AMCLIB_TrackObsrv_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

/***************************************************************************//*!
*
* @brief    Tracking observer estimates the position using an angle error
*
* @param    in  r0 - frac16_t f16Error - error of angle
* @param    ptr r1 - pointer to control structure
*                    AMCLIB_TRACK_OBSRV_T_F32 *psCtrl
*                     - frac32_t f32Theta   - Estimated position <-1;1) corresponds to <-pi;pi)
*                     - frac32_t f32Speed   - Estimated speed <-1;1)
*                     - frac16_t f32I_1     - Internal integrator <-1;1)
*                     - frac16_t f16IGain   - Integ. constant to get speed from error <0;1)
*                     - int16_t  i16IGainSh - Shift for f16IGain <-15;15>
*                     - frac16_t f16PGain   - Prop. constant to get speed from angle <0;1)
*                     - int16_t  i16PGainSh - Shift for f16PGain <-15;15>
*                     - frac16_t f16ThGain  - Constant to get angle from speed <0;1)
*                     - int16_t  i16ThGainSh- Shift for f16ThGain <-15;15>
*
* @return   r0 - estimated position
*
* @remarks  The function calculates tracking observer to determine the angular
*           speed and position of input error functional signal according to following equations:
*
*   f32I_1   = f32I_1   + [(f16Error * f16IGain) << i16IGainSh]
*
*   f32Speed = f32I_1   + [(f16Error * f16PGain) << i16PGainSh]
*
*   f32Theta = f32Theta + [(f32Speed * f16ThGain) << i16ThGainSh]
*
*******************************************************************************/
ASM_LABEL(AMCLIB_TrackObsrv_F16_FAsm)
    /* A = f16Error * psCtrl -> f16IGain */
    push   {r4-r6}            /* Stores r4 - r6 to the stack */
    ldmia  r1!, {r4-r6}       /* r4 = psCtrl -> f32Theta, r6 = psCtrl -> f32I_1 */
    ldrsh  r2, [r1], #2       /* r2 = psCtrl -> f16IGain */
    mul    r2, r0, r2         /* r2 = f16Error * psCtrl -> f16IGain */

    /* A = A << psCtrl -> i16IGainSh */
    ldrsh  r5, [r1], #2       /* r5 = psCtrl -> i16IGainSh */
    adds   r5, r5, #1         /* r5 = r5 + 1 */

    ittt   lt                 /* If r5 < 0, then right shift */
    rsblt  r5, r5, #0         /* r5 = - r5 */
    asrlt  r2, r2, r5         /* r2 = r2 >> r5 */
    movlt  r5, #0             /* r5 = 0 */

    lsl    r3, r2, r5         /* r3 = r2 << r5 */
    asr    r5, r3, r5         /* r5 = r3 >> r5 */
    cmp    r5, r2             /* Compares r2 and r5 */
    ittt   ne                 /* If r2 != r5, then returns the saturate value */
    ssatne r3, #2, r2         /* Saturates r2 to 2-bit value */
    lslne  r3, r3, #30        /* r3 << 30 */
    qaddne r3, r3, r3         /* Addition with saturation */

    /* psCtrl -> f32I_1 = psCtrl -> f32I_1 + A */
    qadd   r3, r3, r6         /* r3 = psCtrl -> f32I_1 + A */
    str    r3, [r1, #-8]      /* psCtrl -> f32I_1 = r3 */

    /* B = f16Error * psCtrl -> f16PGain */
    ldrsh  r2, [r1], #2       /* r2 = psCtrl -> f16PGain */
    mul    r2, r0, r2         /* r2 = f16Error * psCtrl -> f16PGain */

    /* B = B << psCtrl -> i16PGainSh */
    ldrsh  r5, [r1], #2       /* r5 = psCtrl -> i16PGainSh */
    adds   r5, r5, #1         /* r5 = r5 + 1 */

    ittt   lt                 /* If r5 < 0, then right shift */
    rsblt  r5, r5, #0         /* r5 = - r5 */
    asrlt  r2, r2, r5         /* r2 = r2 >> r5 */
    movlt  r5, #0             /* r5 = 0 */

    lsl    r6, r2, r5         /* r6 = r2 << r5 */
    asr    r5, r6, r5         /* r5 = r2 << r5 */
    cmp    r5, r2             /* Compares r2 and r5 */
    ittt   ne                 /* If r2 != r5, then returns the saturate value */
    ssatne r6, #2, r2         /* Saturates r2 to 2-bit value */
    lslne  r6, r6, #30        /* r6 << 30 */
    qaddne r6, r6, r6         /* Addition with saturation */

    /* psCtrl -> f32Speed = B + psCtrl -> f32I_1 */
    qadd   r6, r6, r3         /* r6 = B + psCtrl -> f32I_1 */
    str    r6, [r1, #-16]     /* psCtrl -> f32Speed = r6 */

    /* A = psCtrl -> f32Speed * psCtrl -> f16ThGain */
    ldrh   r2, [r1], #2       /* r2 = psCtrl -> f16ThGain */
    lsl    r2, r2, #16
    smull  r6, r2, r2, r6     /* r2:r6 = f32Speed * psCtrl -> f16ThGain */

    /* A = A << psCtrl -> i16ThGainSh */
    ldrsh  r5, [r1], #-22     /* r5 = psCtrl -> i16ThGainSh */
    adds   r5, r5, #1         /* r5 = r5 + 1 */

    ittt   lt                 /* If r5 < 0, then right shift */
    rsblt  r5, r5, #0         /* r5 = - r5 */
    asrlt  r2, r2, r5         /* r2 = r2 >> r5 */
    movlt  r5, #0             /* r5 = 0 */

    lsl    r0, r2, r5         /* r0 = r2 << r5 */
    asr    r0, r0, r5         /* r0 = r0 >> r5 */
    cmp    r2, r0             /* Compares r2 and r0 */
    ittt   ne                 /* If r2 != r0, then returns the saturate value */
    ssatne r6, #2, r2         /* Saturates r2 to 2-bit value */
    lslne  r6, r6, #30        /* r6 << 30 */
    qaddne r6, r6, r6         /* Addition with saturation */
    mov    r0, #32            /* r0 = 32 */
    sub    r0, r0, r5         /* r0 = 32 - r5 */
    lsl    r2, r2, r5         /* r2 << r5 */
    lsr    r6, r6, r0         /* r6 >> r0 */
    add    r6, r6, r2         /* r6 = r2 + r6 */

    /* psCtrl -> f32Theta = psCtrl -> f32Theta + A */
    add    r6, r6, r4         /* r6 = psCtrl -> f32Theta + A */
    str    r6, [r1]           /* psCtrl -> f32Theta = r1 */
    asr    r0, r6, #16        /* r0 = r6 >> 16 */
    pop    {r4-r6}            /* Restores r4 - r6 to the stack */

    bx      lr                /* Returns */

    ASM_END


