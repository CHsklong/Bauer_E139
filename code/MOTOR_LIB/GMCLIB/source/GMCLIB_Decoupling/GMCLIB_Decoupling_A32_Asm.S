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
* @brief  Decoupling calculation
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(GMCLIB_DecouplingPMSM_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

/*******************************************************************************
*
* @brief  This function calculates decoupled d-axis and q-axis voltage commands
*         of PMSM. The d,q axis decoupling terms are added to controller outputs.
*
* @param  ptr   r0 - pointer to the IN structure of d-axis and q-axis output
*                    voltages from d-axis and q-axis controllers
*               r1 - pointer to the IN structure of d-axis and q-axis output
*                    voltages from d-axis and q-axis controllers
*               r2 - actual measured motor electric speed
*               r3 - pointer to the IN structure of motor parameters needed for
*                    decoupling scaled to 16-bit fractional range
*                    The members of the structure are:
*                    - acc32 a32KdGain
*                         - coefficient scaled; input parameter in the following
*                           range 0 <= f16IGain < 65536.0
*                         - a32KdGain = Kd
*                    - acc32 a32KqGain
*                         - coefficient scaled; input parameter in the following
*                           range 0 <= f16IGain < 65536.0
*                         - a32KqGain = Kq
*               stack - pointer to the OUT structure of decoupled d-axis and q-axis
*                       voltage commands
*
* @remarks
*
*         The decoupling algorithm is calculated according to the following equations:
*           ud_dec = ud - Lq * f16SpeedEI * iq        (1)
*           uq_dec = uq + Ld * f16SpeedEI * id        (2)
*
*         The physical equations (1) and (2) expressed in fractional arithmetic:
*           ud_dec_s = ud_s - f16SpeedEI * iq * (Lq * omega_max * i_max / u_max)   (3)
*           uq_dec_s = uq_s + f16SpeedEI * id * (Ld * omega_max * i_max / u_max)   (4)
*         where
*           Kq = Lq * omega_max * i_max / u_max      (5)
*           Kd = Ld * omega_max * i_max / u_max      (6)
*
*******************************************************************************/
ASM_LABEL(GMCLIB_DecouplingPMSM_F16_FAsm)

    push    {r4, r5}             /* Stores r4 and r5 to the stack */
    ldm     r3, {r4, r5}         /* r4 = Kd, r5 = Kq */

    ldrsh   r3, [r1, #2]         /* r3 = iq */
    mul     r3, r3, r2           /* r3 = f16SpeedEI * iq */
    smull   r5, r3, r5, r3       /* r3:r5 = Kq * f16SpeedEI * iq */
    lsls    r3, r3, #2           /* r3 << 2 */
    add     r3, r3, r5, lsr #30  /* r3 = r3 + r5 >> 30 */
    ldrsh   r5, [r0]             /* r5 = ud */
    sub     r5, r5, r3           /* r5 = ud - Kq * f16SpeedEI * iq*/

    ldrsh   r3, [r1]             /* r3 = id */
    mul     r3, r3, r2           /* r3 = f16SpeedEI * id */
    smull   r4, r3, r4, r3       /* r3:r4 = Kd * f16SpeedEI * id */
    lsls    r3, r3, #2           /* r3 << 2 */
    add     r3, r3, r4, lsr #30  /* r3 = r3 + r4 >> 30 */
    ldrsh   r4, [r0, #2]         /* r4 = uq */
    add     r4, r4, r3           /* r4 = uq + Kd * f16SpeedEI * id*/

    ssat    r0, #16, r5          /* Saturation */
    ssat    r1, #16, r4          /* Saturation */
    pop     {r4, r5}             /* Restores r4 and r5 from the stack */
    ldr     r3, [sp]             /* r3 = pointer to the OUT structure */
    strh    r0, [r3]             /* Stores ud */
    strh    r1, [r3, #2]         /* Stores uq */

    bx      lr                   /* Returns */

    ASM_END
