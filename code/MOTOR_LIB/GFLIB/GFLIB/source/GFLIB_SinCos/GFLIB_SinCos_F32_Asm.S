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
* @brief  Sine
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(GFLIB_Sin_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

/***************************************************************************//*!
* @brief  Calculates the sine of the given argument using Taylor 9th order
*         polynomial.
*
* @param  in  r0 - Argument in range [-1; 1) in frac16_t corresponding to [-pi; pi)
*             r1 - Pointer to the Taylor polynomial coefficients
*
* @return This function returns
*             r0 - frac16_t value [-1; 1)
*
* @remarks
*    This function calculates sin(pi * x) using Taylor 9th order polynomial:
*    x=[-pi; pi]:
*        sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*        sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*        sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*        where:
*        d1 =  1
*        d3 = -1 / 3!
*        d5 =  1 / 5!
*        d7 = -1 / 7!
*        d9 =  1 / 9!
*
*    because the x value is in range [-1; 1) that corresponds to [-pi; pi),
*    the coefficients 'd' needs to be scaled:
*
*        x=[-1; 1):
*        sin(pi * x) = x(c1 + x^2(c3 + x^2(c5 + x^2(c7 + x^2*c9))))
*
*        c1 = d1 * pi^1 =  pi
*        c3 = d3 * pi^3 = -pi^3 / 3!
*        c5 = d5 * pi^5 =  pi^5 / 5!
*        c7 = d7 * pi^7 = -pi^7 / 7!
*        c9 = d9 * pi^9 =  pi^9 / 9!
*
*    The 9th order polynomial approximation of the sine function
*    has very good accuracy in range [-pi/2; pi/2] of argument, but in wider
*    range the calculation error is quickly growing up. Therefore the sine
*    function is    approximated only in range [-pi/2; pi/2] and it is used the
*    symmetry of the sine function [ sin(alpha) = sin(pi - alpha) ].
*    Therefore the alpha argument is in range [-pi/2; pi/2) that corresponds
*    to x [-0.5; 0.5).
*    To make calculations more precise (because we need to use the value
*    of x^2 rounded to 16-bit fractional number in our calculations),
*    the given argument value x (that is in range [-0.5; 0.5)) is shifted
*    by 1 bit to the left (multiplied by 2), then the calculated x^2 value
*    is in range [-1; 1) instead of [-0.25; 0.25).
*    Then the polynomial coefficients 'c' needs to be scaled
*    (shifted to the right):
*
*        b1 = c1 / 2^1 =  pi / 2
*        b3 = c3 / 2^3 = -pi^3 / 3! / 2^3
*        b5 = c5 / 2^5 =  pi^5 / 5! / 2^5
*        b7 = c7 / 2^7 = -pi^7 / 7! / 2^7
*        b9 = c9 / 2^9 =  pi^9 / 9! / 2^9
*
*    To avoid the saturation error during the polynomial calculation
*    the coefficients 'b' are divided by 2. After the polynomial calculation
*    the result is multiplied by 2 (shifted by 1 bit to the left)
*    to take the right result ( in range [-1; 1) ).
*
*        a1 = b1 / 2 =  pi / 2^2         =  0.785398163
*        a3 = b3 / 2 = -pi^3 / 3! / 2^4  = -0.322982049
*        a5 = b5 / 2 =  pi^5 / 5! / 2^6  =  0.039846313
*        a7 = b7 / 2 = -pi^7 / 7! / 2^8  = -0.002340877
*        a9 = b9 / 2 =  pi^9 / 9! / 2^10 =  0.000080220
*
*        In 32-bit signed fractional representation:
*        a1 = 0x6487ED51
*        a3 = 0xD6A88634
*        a5 = 0x0519AF1A
*        a7 = 0xFFB34B4D
*        a9 = 0x0002A0F0
*
*    sin(pi * x) = (x<<1)*(a1 + (x<<1)^2(a3 + (x<<1)^2(a5 + (x<<1)^2(a7 \
*                  + (x<<1)^2*a9)))) << 1
*
*******************************************************************************/
ASM_LABEL(GFLIB_Sin_F16_FAsm)
    push    {r4-r6}              /* Stores r4-r6 */
    lsls    r2, r0, #17          /* x = f16Angle << 17 */
    it      pl                   /* Compares x with 0 */
    rsbpl   r2, r2, #0           /* If x > 0, then x = - x */
    asr     r2, r2, #16          /* x >> 16 */
    mul     r6, r2, r2           /* r6 = x * x */
    add     r6, r6, #0x4000      /* Rounding */
    asr     r6, r6, #15          /* r6 >> 15 */
    /*------------------------------------------------------------------------
    * x(a1 + x^2*(a3 + x^2(a5 + x^2(a7 + a9 * x^2))))
    *------------------------------------------------------------------------*/
    /* a7 + a9 * x^2 */
    ldrd    r4, r3, [r1], #8     /* r3 = a7, r4 = a9*/
    smull   r4, r5, r6, r4       /* r5:r4 = a9 * x^2 */
    lsr     r4, r4, #15          /* r4 >> 15 */
    add     r4, r4, r5, lsl #17  /* r4 = r4 + (r5 << 17) */
    add     r3, r3, r4           /* r3 = a7 + a9 * x^2 */
    /* a5 + x^2(a7 + a9 * x^2) */
    ldr     r4, [r1], #4         /* r4 = a5*/
    smull   r3, r5, r6, r3       /* r5:r3 = r3 * x^2 */
    lsr     r3, r3, #15          /* r3 >> 15 */
    add     r3, r3, r5, lsl #17  /* r3 = r3 + (r5 << 17) */
    add     r3, r3, r4           /* r3 = a5 + x^2(a7 + a9 * x^2) */
    /* a3 + x^2(a5 + x^2(a7 + a9 * x^2)) */
    ldr     r4, [r1], #4         /* r4 = a3*/
    smull   r3, r5, r6, r3       /* r5:r3 = r3 * x^2 */
    lsr     r3, r3, #15          /* r3 >> 15 */
    add     r3, r3, r5, lsl #17  /* r3 = r3 + (r5 << 17) */
    add     r3, r3, r4           /* r3 = a5 + x^2(a7 + a9 * x^2) */
    /* a1 + x^2*(a3 + x^2(a5 + x^2(a7 + a9 * x^2))) */
    ldr     r4, [r1]             /* r4 = a1*/
    smull   r3, r5, r6, r3       /* r5:r3 = r3 * x^2 */
    lsr     r3, r3, #15          /* r3 >> 15 */
    add     r3, r3, r5, lsl #17  /* r3 = r3 + (r5 << 17) */
    add     r3, r3, r4           /* r3 = a1 + x^2*(a3 + x^2(a5 + x^2(a7 + a9 * x^2))) */
    /* x(a1 + x^2*(a3 + x^2(a5 + x^2(a7 + a9 * x^2)))) */
    smull   r4, r5, r3, r2       /* r5:r4 = x * r3 */
    lsr     r4, r4, #29          /* r4 >> 29*/
    add     r4, r4, r5, lsl #3   /* r4 = r4 + (r5 << 3) */
    add     r4, r4, #1           /* Rounding */
    asr     r1, r4, #1           /* r1 = r4 << 1*/
    pop     {r4-r6}              /* Restores r4-r6 */
    cmp     r0, #0               /* Compares r0 with 0 */
    it      gt                   /* If r0 > 0, then executes next command */
    rsbgt   r1, r1, #0           /* r1 = -r1 */
    ssat    r0, #16, r1          /* Saturates r1 to r0 */
    bx      lr                   /* Returns */

    ASM_END
