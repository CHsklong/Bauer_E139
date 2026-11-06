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
* @brief  Arcus tangent function based on the provided x,y coordinates as arguments
*         using division and piece-wise polynomial approximation
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(GFLIB_AtanYX_F16_FAsm)

    ASM_EXTERN(GFLIB_Atan_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

/***************************************************************************//*!
*
* @brief     The GFLIB_AtanYX function computes the AtanYX(f16Y,f16X)
*            using piece-wise polynomial approximation.
*
* @param     ptr r2 - *pbErrFlag - The pointed variable is 1, if the inputs were 0, 0
*                              otherwise it is 0.
*                r3 - Pointer to the Atan table
*
* @param     in  r0 - f16Y in the range of [-1,1).
*                r1 - f16X in the range of [-1,1).
*
* @return    The function returns
*                r0 - AtanYX(f16Y,f16X).The output data value
*            is in range [-0.25, 0.25), which corresponds to the angle
*            in the range of [-pi/4,pi/4).
*
* @remarks   The GFLIB_Atan function computes the atan(x)/pi
*            using piece-wise polynomial approximation.
*
*            For calculation of atan(x) interval [0,1) is divided into eight equally
*            spaced sub intervals, for each sub-interval is calculated polynomial
*            coefficients. Minimax polynomial approximation is done for each interval
*            respectively. Input value is scaled on the range FRAC [-0.5,0.5) inside
*            appropriate sub-interval.
*
*            f16Arg = f16Y / f16X
*
*            x = ( (f16Arg - (f16Arg & 0xF000) << (3) ) - 0x3FFF) << (1)
*
*            Atan(x) =  (A1*|x| + A2)*|x| + A3
*
*            Atan(f16Arg) = 2 * (sing * |Atan(x)|)
*
*            | 16bit Xsign | 15bit Ysign |  Result Interval  |
*            |===============================================|
*            |      0      |      0      |       [0;0,5]     |
*            |      0      |      1      |       (0,5;1)     |
*            |      1      |      0      |       [-0,5;0)    |
*            |      1      |      1      |       [-1;-0.5)   |
*            |_____________|_____________|___________________|
*            "sign register"
*
*            In case of the inputs 0, 0 the error flag is set to 1 and the output
*            of the function is 0.
*******************************************************************************/
ASM_LABEL(GFLIB_AtanYX_F16_FAsm)
    push    {r4-r6, lr}              /* Stores r4 - r6 and lr */
    mov     r5, #0                   /* r5 = 0 */
    mov     r6, r0                   /* r4 = f16Y */

    mul     r4, r0, r1               /* r4 = r0*r1 */
    cmp     r4, #0                   /* Compares r4 with 0 */
    bne     AtanYX_F16_NotError      /* If r0*r1 != 0, then computes the AtanYX function*/

    orrs    r3, r0, r1               /* r3 = f16Y | f16X */
    it      eq                       /* If f16Y | f16X = 0, then the error flag is set */
    moveq   r4, #1                   /* r4 = 1 */
    strh     r4, [r2]                /* *pbErrFlag = r4 */

    cmp     r1, #0                   /* Compares f16X with 0 */
    ittt    eq                       /* If f16X = 0, then executes next two commands */
    ssateq  r0, #16, r0, lsl #16     /* Saturates (r0 << 16) to 16-bit value */
    addeq   r0, r0, #1               /* r0 = r0 + 1 */
    asreq   r0, r0, #1               /* r0 = r0 >> 1 */

    cmp     r0, #0                   /* Compares f16Y with 0 */
    itt     eq                       /* If f16X = 0, then executes next two commands */
    ssateq  r0, #1, r1               /* Saturates r0 to a 2-bit value */
    lsreq   r0, r0, #17              /* r0 >> 17 */

    pop     {r4-r6, lr}              /* Restores r4 - r6 and lr */
    bx      lr                       /* Return */

/************************* inputs without zero value  *********************/
ASM_LABEL(AtanYX_F16_NotError)
    strh    r5, [r2]                 /* *pbErrFlag = 0 */

    ittt    lt                       /* If f16Y * f16X < 0, then executes next three commands */
    movlt   r5, #0x4000              /* r5 = 0x4000 */
    rsblt   r0, r1, #0               /* r0 = - f16X */
    movlt   r1, r6                   /* r1 = f16Y */

    cmp     r6, #0                   /* Compares f16Y with 0 */
    ittt    lt                       /* If f16Y < 0, then executes next three commands */
    orrlt   r5, r5, #0x8000          /* r5 = r5 | 0x8000 */
    rsblt   r0, r0, #0               /* r0 = - r0 */
    rsblt   r1, r1, #0               /* r1 = - r1 */
    sxth    r5, r5                   /* Sets sign of r5 */

    cmp     r0, r1                   /* Compares r0 and r1 */
    bgt     AtanYX_F16_L1            /* If r0 > r1, then goes to AtanYX_F16_L1 */
    blt     AtanYX_F16_L2            /* If r0 < r1, then goes to AtanYX_F16_L1 */
    add     r0, r5, #0x2000          /* r0 = r5 + 0x2000 */
    pop     {r4-r6, lr}              /* Restores r4 - r6 and lr */
    bx      lr                       /* Returns */

ASM_LABEL(AtanYX_F16_L1)
    add     r5, r5, #0x2000          /* r5 = r5 + 0x2000 */
    mov     r6, r0                   /* r6 = r0 */
    sub     r0, r0, r1               /* r0 = r0 - r1 */
    adds    r1, r1, r6               /* r1 = r1 + r6 */

ASM_LABEL(AtanYX_F16_L2)
    lsl     r0, r0, #15              /* r0 << 15 */
    sdiv    r0, r0, r1               /* r0 = r0 / r1 */
    mov     r1, r3                   /* r1 = Pointer to the Atan table */

    bl      GFLIB_Atan_F16_FAsm      /* r0 = Atan(r0) */
    add     r0, r0, r5               /* r0 = r0 + r5 */
    ssat    r0, #16, r0              /* Saturation */

    pop     {r4-r6, lr}              /* Restores r4 - r6 and lr */
    bx      lr                       /* Returns */

    ASM_END
