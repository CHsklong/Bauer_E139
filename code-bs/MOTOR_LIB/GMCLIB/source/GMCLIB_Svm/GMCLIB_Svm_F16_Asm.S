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
* @brief  Space vector modulation
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(GMCLIB_SvmStd_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

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

ASM_LABEL(GMCLIB_SvmStd_F16_FAsm)

    /* C = alfa * sqrt(3)/2 */
    ldr     r2, [r0]             /* r2.top = alfa, r2.bottom = beta */
    movw    r3, #0xEBA1          /* Loads bottom halfword of r3 */
    movt    r3, #0x6ED9          /* r3 = sqrt(3)/2 */
    smulwb  r3, r3, r2           /* r3 = (sqrt(3)/2 * alfa)/2 */
    asr     r3, r3, #15          /* r3 = sqrt(3)/2 * alfa */

    /* B = -alfa * sqrt(3)/2 - beta/2 */
    asr     r2, r2, #16          /* r2 = beta */
    rsb     r0, r3, #0           /* r0 = - sqrt(3)/2 * alfa */
    subs    r0, r0, r2, asr #1   /* r0 = - alfa * sqrt(3)/2 - beta/2 */
    bgt     Sector3              /* If r0 > 0, then it is not 1st, 2nd or 6th sector */

    /* A = sqrt(3)/2 * alfa - beta/2 */
    subs    r3, r3, r2, asr #1   /* r3 = sqrt(3)/2 * alfa - beta/2 */
    ble     Sector2              /* If r3 <= 0, then it is the 2nd sector */
    cmp     r2, #0               /* Compares beta with 0 */
    ble     Sector6              /* If beta <= 0, then it is the 6th sector */

    /* B <= 0, A > 0, beta > 0 */
    /* 1st sector

       psOut -> f16C = (1 - alpha * sqrt(3)/2 - beta/2 ) / 2 = (1 + B ) / 2
       psOut -> f16B = PhaseC + beta
       psOut -> f16A = PhaseB + alpha * sqrt(3)/2 - beta/2 =PhaseB + A */

    add     r0, r0, #0x8000      /* r0 = (1 + B) */
    add     r2, r2, r0, asr #1   /* r2 = (1 + B)/2 + beta */
    add     r3, r2, r3           /* r3 = (1 + B)/2 + beta + A */
    usat    r0, #15, r0, asr #1  /* r0 = (1 + B)/2 */
    usat    r2, #15, r2          /* Saturation */
    usat    r3, #15, r3          /* Saturation */
    strh    r3, [r1], #2         /* psOut.A = (1 + B)/2 + beta + A */
    strh    r2, [r1], #2         /* psOut.B = (1 + B)/2 + beta */
    strh    r0, [r1]             /* psOut.C = (1 + B)/2 */
    mov     r0, #1               /* Result = 1 */
    bx      lr                   /* Returns */

ASM_LABEL(Sector2)               /* B <= 0, A <= 0 */
    /* 2nd sector

       psOut -> f16C = (1 - beta) / 2
       psOut -> f16A = PhaseC + alpha * sqrt(3)/2 + beta/2 = PhaseC - B;
       psOut -> f16B = PhaseA - alpha * sqrt(3)/2 + beta/2 = PhaseA - A */

    rsb     r2, r2, #0x8000      /* r2 = (1 - beta) */
    asr     r2, r2, #1           /* r2 = (1 - beta)/2 */
    sub     r0, r2, r0           /* r0 = (1 - beta)/2 - B */
    sub     r3, r0, r3           /* r0 = (1 - beta)/2 - B - A */
    usat    r0, #15, r0          /* Saturation */
    usat    r3, #15, r3          /* Saturation */
    strh    r0, [r1], #2         /* psOut.A = (1 - beta)/2 - B */
    strh    r3, [r1], #2         /* psOut.B = (1 - beta)/2 - B - A */
    strh    r2, [r1]             /* psOut.C = (1 - beta)/2 */
    mov     r0, #2               /* Result = 2 */
    bx      lr                   /* Returns */

ASM_LABEL(Sector3)               /* B > 0, A <= 0, beta > 0 */
    /* A = sqrt(3)/2 * alfa - beta/2 */
    subs    r3, r3, r2, asr #1   /* r3 = sqrt(3)/2 * alfa - beta/2 */
    bgt     Sector5              /* If r3 > 0, then it is the 5th sector */

    cmp     r2, #0               /* Compares beta with 0 */
    ble     Sector4              /* If beta <= 0, then it is the 4th sector */

    /* 3rd sector

       psOut -> f16A = (1 + alpha * sqrt(3)/2 - beta/2 ) / 2 = ( 1 + A ) / 2
       psOut -> f16C = PhaseA - alpha * sqrt(3)/2 - beta/2 = PhaseA + B
       psOut -> f16B = PhaseC + beta  */

    add     r3, r3, #0x8000      /* r3 = (1 + A) */
    add     r0, r0, r3, asr #1   /* r0 = (1 + A)/2 + B */
    add     r2, r0, r2           /* r0 = (1 + A)/2 + B + beta */
    usat    r3, #15, r3, asr #1  /* r3 = (1 + A)/2 */
    usat    r0, #15, r0          /* Saturation */
    usat    r2, #15, r2          /* Saturation */
    strh    r3, [r1], #2         /* psOut.A = (1 + A)/2 */
    strh    r2, [r1], #2         /* psOut.B = (1 + A)/2 + B + beta */
    strh    r0, [r1]             /* psOut.C = (1 + A)/2 + B*/
    mov     r0, #3               /* Result = 3 */
    bx      lr                   /* Returns */

ASM_LABEL(Sector4)               /* B > 0, A <= 0, beta <= 0 */
    /* 4th sector

       psOut -> f16A = (1 + alpha * sqrt(3)/2 + beta/2) / 2 = (1 - B ) / 2
       psOut -> f16B = PhaseA - alpha * sqrt(3)/2 + beta / 2 = PhaseA - A
       psOut -> f16C = PhaseB - beta */

    rsb     r0, r0, #0x8000      /* r0 = (1 - B) */
    asr     r0, r0, #1           /* r0 = (1 - B)/2 */
    sub     r3, r0, r3           /* r2 = (1 - B)/2 - A */
    sub     r2, r3, r2           /* r0 = (1 - B)/2 - A - beta */
    usat    r0, #15, r0          /* Saturation */
    usat    r3, #15, r3          /* Saturation */
    usat    r2, #15, r2          /* Saturation */
    strh    r0, [r1], #2         /* psOut.A = (1 - B)/2 */
    strh    r3, [r1], #2         /* psOut.B = (1 - B)/2 - A */
    strh    r2, [r1]             /* psOut.C = (1 - B)/2 - A - beta */
    mov     r0, #4               /* Result = 4 */
    bx      lr                   /* Returns */

ASM_LABEL(Sector5)               /* B > 0, A > 0 */
    /* 5th sector

       pOut -> f16B = (1 + beta) / 2
       pOut -> f16A = PhaseB + alpha * sqrt(3)/2 - beta/2 = PhaseB + A
       pOut -> f16C = PhaseA - alpha * sqrt(3)/2 - beta/2 = PhaseA + B */

    add     r2, r2, #0x8000      /* r2 = (1 + beta) */
    asr     r2, r2, #1           /* r2 = (1 + beta)/2 */
    add     r3, r3, r2           /* r2 = (1 + beta)/2 + A */
    add     r0, r0, r3           /* r0 = (1 + beta)/2 + A + B */
    usat    r3, #15, r3          /* Saturation */
    usat    r0, #15, r0          /* Saturation */
    strh    r3, [r1], #2         /* psOut.A = (1 + beta)/2 + A */
    strh    r2, [r1], #2         /* psOut.B = (1 + beta)/2 */
    strh    r0, [r1]             /* psOut.C = (1 + beta)/2 + A + B */
    mov     r0, #5               /* Result = 5 */
    bx      lr                   /* Returns */

ASM_LABEL(Sector6)               /* B <= 0, A > 0, beta <= 0 */
    /* 6th sector

       psOut -> f16B = (1 - alpha * sqrt(3)/2 + beta/2) / 2 = (1 - A) / 2
       psOut -> f16C = PhaseB - beta
       psOut -> f16A = PhaseC + alpha * sqrt(3)/2 + beta/2 = PhaseC - B  */

    rsb     r3, r3, #0x8000      /* r3 = (1 - A) */
    asr     r3, r3, #1           /* r3 = (1 - A)/2 */
    sub     r2, r3, r2           /* r3 = (1 - A)/2 - beta */
    sub     r0, r2, r0           /* r0 = (1 - A)/2 - beta - B */
    usat    r3, #15, r3          /* Saturation */
    usat    r2, #15, r2          /* Saturation */
    usat    r0, #15, r0          /* Saturation */
    strh    r0, [r1], #2         /* psOut.A = (1 - A)/2 - beta - B */
    strh    r3, [r1], #2         /* psOut.B = (1 - A)/2 */
    strh    r2, [r1]             /* psOut.C = (1 - A)/2 - beta */
    mov     r0, #6               /* Result = 6 */
    bx      lr                   /* Returns */

    ASM_END
