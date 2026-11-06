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
* @brief  Arcus tangent algorithms using piece-wise polynomial approximation
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(GFLIB_Atan_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

/***************************************************************************//*!
*
* @brief     The GFLIB_Atan function computes the Atan(x)/pi
*            using piece-wise polynomial approximation.
*
* @param     in  r0 - The input data value is in the range of [-1,1)
*                r1 - Pointer to the table
*
* @return    This function returns
*                r0 - frac16_t value [-0.25, 0.25)
*
* @return    The function returns atan(x)/pi.The output data value
*            is in range [-0.25, 0.25), which corresponds to the angle
*            in the range of [-pi/4,pi/4).
*
* @remarks   The GFLIB_Atan function computes the atan(x)/pi
*            using piece-wise polynomial approximation.
*
*            Interval [0,1) is divided into eight equally spaced sub intervals,
*            for each sub-interval is calculated polynomial coefficients.
*            Minimax polynomial approximation
*            is done for each interval respectively.
*            Input value is scaled on the range FRAC [-0.5,0.5) inside
*            appropriate sub-interval.
*
*            Output value is:
*
*            Atan(x) =  sign(x) * ((A1*|x| + A2)*|x| + A3)
*
*
*
*******************************************************************************/
ASM_LABEL(GFLIB_Atan_F16_FAsm)

    /* The Tan function is same on the ranges (-pi;-pi/2) and (0;pi/2), the sign is saved in the r0 */
    push    {r4, r5}            /* Stores r4, r5 */
    movs    r2, r0              /* r2 = x */
    it      lt                  /* Compares x with 0 */
    rsblt   r2, r2, #0          /* If x<0 then r2 = -x */
    usat    r2, #15, r2         /* Saturation */
    /* Polynom sector choose */
    asr     r3, r2, #12         /* r3 = |x| >> 12 */
    /* Input value is scaled on the range FRAC <0,0.5) */
    sub     r2, r2, r3, lsl #12 /* r2 = r2 - (r3 << 12)*/
    lsl     r2, r2, #3          /* r2 << 3 */
    /* Input value is scaled on the range FRAC <-0.5,0.5) */
    movw    r4, #0x3FFF         /* r4 = 0x3FFF/2 */
    sub     r2, r2, r4          /* r2 = r2 - 0x3FFF */
    lsl     r2, r2, #1          /* r2 << 1 */
    /* (a1 * x) + a2 */
    mov     r4, #12             /* r4 = 12 */
    mla     r1, r3, r4, r1      /* r1 = r1 + 12 * r3 */
    ldrd    r3, r4, [r1], #8    /* r3 = A1, r4 = A2 */
    smull   r5, r3, r3, r2      /* r3:r5 = A1*|x| */
    lsr     r5, r5, #15         /* r5 = r5 >> 15 */
    add     r5, r5, r3, lsl #17 /* r5 = r5 + (r3 << 17)*/
    add     r5, r5, r4          /* r5 = A1*|x| + A2 */
    /* ((a1 * x) + a2) * x + a3 */
    ldr     r4, [r1]            /* r4 = A3 */
    smull   r5, r3, r5, r2      /* r3:r5 = ((a1 * x) + a2) * x*/
    lsr     r5, r5, #15         /* r5 >> 15 */
    add     r5, r5, r3, lsl #17 /* r5 = r5 + (r3 << 17) */
    add     r5, r5, r4          /* r5 = ((a1 * x) + a2) * x + a3 */
    /* Sign is added to the result value */
    cmp     r0, #0              /* Conpares r0 with 0 */
    it      lt                  /* If r0 < 0, then executes next command */
    rsblt   r5, r5, #0          /* r5 = - r5*/
    add     r5, r5, #0x8000     /* Rounding */
    asr     r0, r5, #16         /* r0 = r5 >> 16 */
    pop     {r4, r5}            /* Restores r4, r5 */
    bx      lr                  /* Returns */

    ASM_END
