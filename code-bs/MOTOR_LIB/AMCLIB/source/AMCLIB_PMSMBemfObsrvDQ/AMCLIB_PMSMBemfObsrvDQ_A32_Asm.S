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
* @brief  Algorithm of PMSM Back Electromotive Force observer in rotating
*         reference frame
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(AMCLIB_PMSMBemfObsrvDQ_F16_FAsm)

    ASM_EXTERN(GFLIB_Atan_F16_FAsm)

    ASM_EXTERN(gsAtanCoef)

    ASM_COMP_SPECIFIC_DIRECTIVES

/***************************************************************************//*!
*
* @brief    BEMF observer using D,Q components to calculate error BEMF gamma/delta.
*
* @param    ptr   r0 - GMCLIB_2COOR_DQ_T_F16 *psIDQ - pointer to the D, Q current
*                 r1 - GMCLIB_2COOR_DQ_T_F16 *psUDQ - pointer to the D, Q voltage
*                 r3 - AMCLIB_BEMF_OBSRV_DQ_T_A32 *psCtrl - pointer to the parameters of the observer
*           in    r2 - the electrical angular speed in the actual step
*
* @return         r0 - frac16_t error between the real and estimated position
*
* @remarks  The function calculates the algorithm of back electromotive
*           force observer in rotating reference frame.
*
*    f32TmpEObsrvD = psCtrl_sCtrl_f32ID_1 +
*                    (psCtrl_sCtrl_a32IGain + psCtrl_sCtrl_a32PGain) * (psUDQ_f16D - psCtrl_sIObsrv_f32D)
*                    
*    f32TmpEObsrvQ = psCtrl_sCtrl_f32IQ_1 +
*                    (psCtrl_sCtrl_a32IGain + psCtrl_sCtrl_a32PGain) * (psUDQ_f16Q - psCtrl_sIObsrv_f32Q)
*                    
*    f32TmpIObsrvD = a32IGain * psCtrl_sIObsrv_f32D + a32UGain * psUDQ_f16D +
*                    psIDQ_f16Q * f16Speed * a32WIGain + a32EGain * f32TmpEObsrvD
*    f32TmpIObsrvQ = a32IGain * psCtrl_sIObsrv_f32Q + a32UGain * psUDQ_f16Q -
*                    psIDQ_f16D * f16Speed * a32WIGain + a32EGain * f32TmpEObsrvQ
*
*                      |  Atan(-f16TmpEObsrvD /  f16TmpEObsrvQ)  if   f16Speed < 0
*    psCtrl_f16Error = |
*                      |  Atan( f16TmpEObsrvD / -f16TmpEObsrvQ)  else
*
****************************************************************************/
ASM_LABEL(AMCLIB_PMSMBemfObsrvDQ_F16_FAsm)
    push    {r4-r7, lr}          /* Stores r4 - r7 and lr to the stack */
    /* D controller */
    /* D_error =  psIDQ->f16D - psCtrl->sIObsrv.f32D */
    ldrh    r4, [r0]             /* r4 = psIDQ->f16D */
    ldr     r5, [r3, #8]         /* r5 = psCtrl->sIObsrv.f32D */
    lsl     r4, r4, #16          /* r4 << 16 */
    qsub    r4, r4, r5           /* r4 = psIDQ->f16D - psCtrl->sIObsrv.f32D */

    /* D integral scale = psCtrl->Ctrl.a32IGain * D_error + psCtrl->Ctrl.f32ID_1 */
    ldr     r5, [r3, #28]        /* r5 = psCtrl->Ctrl.a32IGain */
    ldr     r6, [r3, #16]        /* r6 = psCtrl->Ctrl.f32ID_1 */
    smull   r7, r5, r5, r4       /* r5:r7 = psCtrl->Ctrl.a32IGain * D_error */
    lsr     r7, r7, #15          /* r7 >> 15 */
    add     r7, r7, r5, lsl #17  /* r7 = r7 + r5 << 17 */
    asr     r5, r5, #15          /* r5 >> 15 */
    adds    r7, r7, r6           /* r7 = r7 + r6 */
    adc     r5, r5, r6, asr #31  /* r5:r7 = psCtrl->Ctrl.a32IGain * D_error + psCtrl->Ctrl.f32ID_1 */

    lsls    r6, r7, #1           /* If r7 < 0, then sets Carry */
    adcs    r6, r5, #0           /* r6 = r5 + Carry */
    ittte   ne                   /* If r6 != 0, then saturates result */
    asrne   r6, r5, #31          /* r6 = sign of r5 */
    addne   r6, r6, #1           /* r6 = r6 + 1 */
    rsbne   r6, r6, #0x80000000  /* r6 = 0x80000000 - r6 */
    moveq   r6, r7               /* r6 = r7 */
    str     r6, [r3, #16]        /* psCtrl->Ctrl.f32ID_1 = r6 */

    /* D proportional scale = psCtrl->Ctrl.a32PGain * D_error */
    ldr     r6, [r3, #24]        /* r6 = psCtrl->Ctrl.a32PGain */
    smull   r6, r4, r6, r4       /* r4:r6 = psCtrl->Ctrl.a32PGain * D_error */
    lsr     r6, r6, #15          /* r6 >> 15 */
    add     r6, r6, r4, lsl #17  /* r6 = r6 + r4 << 17 */
    asr     r4, r4, #15          /* r4 >> 15 */

    /* D controller output calculation */
    adds    r7, r7, r6           /* r7 = r7 + r6 */
    adc     r5, r5, r4           /* r5 = r5 + r4 + Carry */

    lsls    r6, r7, #1           /* If r7 < 0, then sets Carry */
    adcs    r4, r5, #0           /* r4 = r5 + Carry */
    ittt    ne                   /* If r4 != 0, then saturates result */
    asrne   r7, r5, #31          /* r7 = sign of r5 */
    addne   r7, r7, #1           /* r7 = r7 + 1 */
    rsbne   r7, r7, #0x80000000  /* r7 = 0x80000000 - r7 */
    str     r7, [r3]             /* Stores psCtrl->sEObsrv.f32D value */

    /* Q controller */
    /* Q_error =  psIDQ->f16Q - psCtrl->sIObsrv.f32Q */
    ldrh    r4, [r0, #2]         /* r4 = psIDQ->f16Q */
    ldr     r5, [r3, #12]        /* r5 = psCtrl->sIObsrv.f32Q */
    lsl     r4, r4, #16          /* r4 << 16 */
    qsub    r4, r4, r5           /* r4 = psIDQ->f16Q - psCtrl->sIObsrv.f32Q */

    /* Q integral scale = psCtrl->Ctrl.a32IGain * Q_error + psCtrl->Ctrl.f32IQ_1 */
    ldr     r5, [r3, #28]        /* r5 = psCtrl->Ctrl.a32IGain */
    ldr     r6, [r3, #20]        /* r6 = psCtrl->Ctrl.f32IQ_1 */
    smull   r7, r5, r5, r4       /* r5:r7 = psCtrl->Ctrl.a32IGain * Q_error */
    lsr     r7, r7, #15          /* r7 >> 15 */
    add     r7, r7, r5, lsl #17  /* r7 = r7 + r5 << 17 */
    asr     r5, r5, #15          /* r5 >> 15 */
    adds    r7, r7, r6           /* r7 = r7 + r6 */
    adc     r5, r5, r6, asr #31  /* r5:r7 = psCtrl->Ctrl.a32IGain * Q_error + psCtrl->Ctrl.f32IQ_1 */

    lsls    r6, r7, #1           /* If r7 < 0, then sets Carry */
    adcs    r6, r5, #0           /* r6 = r5 + Carry */
    ittte   ne                   /* If r6 !=0, then saturates result */
    asrne   r6, r5, #31          /* r6 = sign of r5 */
    addne   r6, r6, #1           /* r6 = r6 + 1 */
    rsbne   r6, r6, #0x80000000  /* r6 = 0x80000000 - r6 */
    moveq   r6, r7               /* r6 = r7 */
    str     r6, [r3, #20]        /* psCtrl->Ctrl.f32IQ_1 = r6 */

    /* Q proportional scale = psCtrl->Ctrl.a32PGain * Q_error */
    ldr     r6, [r3, #24]        /* r6 = psCtrl->Ctrl.a32PGain */
    smull   r6, r4, r6, r4       /* r4:r6 = psCtrl->Ctrl.a32PGain * Q_error */
    lsr     r6, r6, #15          /* r6 >> 15 */
    add     r6, r6, r4, lsl #17  /* r6 = r6 + r4 << 17 */
    asr     r4, r4, #15          /* r4 >> 15 */

    /* Q controller output calculation */
    adds    r7, r7, r6           /* r7 = r7 + r6 */
    adc     r5, r5, r4           /* r5 = r5 + r4 + Carry */

    lsls    r6, r7, #1           /* If r7 < 0, then sets Carry */
    adcs    r4, r5, #0           /* r4 = r5 + Carry */
    ittt    ne                   /* If r4 != 0, then saturates result */
    asrne   r7, r5, #31          /* r7 = sign of r5 */
    addne   r7, r7, #1           /* r7 = r7 + 1 */
    rsbne   r7, r7, #0x80000000  /* r7 = 0x80000000 - r7 */
    str     r7, [r3, #4]         /* Stores psCtrl->sEObsrv.f32Q value */

    /* Q observer calculation */
    /* r4:r7 = a32EGain * sEObsrv.f32Q */
    ldr     r4, [r3, #44]        /* r4 = psCtrl->a32EGain */
    smull   r7, r4, r4, r7       /* r4:r7 = a32EGain * sEObsrv.f32Q */
    /* a32EGain * sEObsrv.f32Q - psIDQ->f16D * f16Speed * a32WIGain */
    ldrsh   r5, [r0]             /* r5 = psIDQ->f16D */
    mul     r5, r2, r5           /* r5 = psIDQ->f16D * f16Speed */
    lsl     r5, r5, #1           /* r5 << 1 */
    rsbs    r5, r5, #0           /* r5 = - psIDQ->f16D * f16Speed */
    ldr     r6, [r3, #40]        /* r6 = psCtrl->a32WIGain */
    smlal   r7, r4, r5, r6       /* r4:r7 = r4:r7 - psIDQ->f16D * f16Speed * a32WIGain */
    /* r4:r7 = a32EGain * sEObsrv.f32Q - psIDQ->f16D * f16Speed * a32WIGain + a32UGain * psUDQ->f16Q */
    ldr     r5, [r3, #36]        /* r5 = psCtrl->a32UGain */
    ldrh    r6, [r1, #2]         /* r6 = psUDQ->f16Q */
    lsls    r6, r6, #16          /* r6 << 16 */
    smlal   r7, r4, r5, r6       /* r4:r7 = r4:r7 + a32UGain * psUDQ->f16Q */
    /* r4:r7 = a32EGain * sEObsrv.f32Q - psIDQ->f16D * f16Speed * a32WIGain + a32UGain * psUDQ->f16Q + a32IGain * sIObsrv.f32Q */
    ldr     r5, [r3, #32]        /* r5 = psCtrl->a32IGain */
    ldr     r6, [r3, #12]        /* r6 = psCtrl->sIObsrv.f32Q */
    smlal   r7, r4, r5, r6       /* r4:r7 = r4:r7 + a32IGain * sIObsrv.f32Q */
    /* Saturation */
    lsr     r7, r7, #15          /* r7 >> 15 */
    add     r7, r7, r4, lsl #17  /* r7 = r7 + r4 << 17 */
    add     r4, r4, #0x4000      /* r4 = r4 + 0x4000 */
    asrs    r4, r4, #15          /* r4 >> 15 */
    ittt    ne
    movne   r7, #0x80000000      /* r7 = 0x80000000 */
    subne   r7, r7, #1           /* r7 = 0x7FFFFFFF */
    subne   r7, r7, r4, asr #17  /* r7 = 0x7FFFFFFF - (r4 >> 17) */
    str     r7, [r3, #12]        /* psCtrl->sIObsrv.f32Q = r7 */

    /* D observer calculation */
    /* r4:r7 = a32IGain * sIObsrv.f32Q */
    ldr     r4, [r3, #8]         /* r4 = psCtrl->sIObsrv.f32D */
    smull   r7, r4, r4, r5       /* r4:r7 = a32IGain * sIObsrv.f32D */
    /* r4:r7 = a32IGain * sIObsrv.f32Q + a32UGain * psUDQ->f16D */
    ldrh    r6, [r1]             /* r6 = psUDQ->f16D */
    lsls    r6, r6, #16          /* r6 << 16 */
    ldr     r5, [r3, #36]        /* r5 = psCtrl->a32UGain */
    smlal   r7, r4, r5, r6       /* r4:r7 = r4:r7 + a32UGain * psUDQ->f16D */
    /* r4:r7 = a32IGain * sIObsrv.f32Q + a32UGain * psUDQ->f16D + psIDQ->f16Q * f16Speed * a32WIGain */
    ldrsh   r5, [r0, #2]         /* r5 = psIDQ->f16Q */
    mul     r5, r2, r5           /* r5 = psIDQ->f16D * f16Speed */
    qadd    r5, r5, r5           /* r5 << 1 */
    ldr     r6, [r3, #40]        /* r6 = psCtrl->a32WIGain */
    smlal   r7, r4, r5, r6       /* r4:r7 = r4:r7 + psIDQ->f16Q * f16Speed * a32WIGain */
    /* r4:r7 = a32IGain * sIObsrv.f32Q + a32UGain * psUDQ->f16D + psIDQ->f16Q * f16Speed * a32WIGain + a32EGain * sEObsrv.f32D */
    ldr     r5, [r3, #44]        /* r5 = psCtrl->a32EGain */
    ldr     r6, [r3]             /* r6 = psCtrl->sEObsrv.f32D */
    smlal   r7, r4, r5, r6       /* r4:r7 = r4:r7 + a32EGain * sEObsrv.f32D */
    /* Saturation */
    lsr     r7, r7, #15          /* r7 >> 15 */
    add     r7, r7, r4, lsl #17  /* r7 = r7 + r4 << 17 */
    add     r4, r4, #0x4000      /* r4 = r4 + 0x4000 */
    asrs    r4, r4, #15          /* r4 >> 15 */
    ittt    ne                   /* If r4 != 0, then saturates result */
    movne   r7, #0x80000000      /* r7 = 0x80000000 */
    subne   r7, r7, #1           /* r7 = 0x7FFFFFFF */
    subne   r7, r7, r4, asr #17  /* r7 = 0x7FFFFFFF - (r4 >> 17) */
    str     r7, [r3, #8]         /* psCtrl->sIObsrv.f32D = r7 */

    /* BEMF determination according to speed sign */
    add     r4, r3, #48          /* r4 = pointer to the psCtrl->f16Error */

    asr     r0, r6, #1           /* r0 = sEObsrv.f32D >> 1 */
    ldr     r1, [r3, #4]         /* r1 = psCtrl->sEObsrv.f32Q */
    asr     r1, r1, #1           /* r1 = sEObsrv.f32Q >> 1 */

    cmp     r2, #0               /* Compares f16Speed with 0 */
    ite     lt                   /* Condition block */
    rsblt   r0, r0, #0           /* If f16Speed < 0, then r0 = - r0 */
    rsbge   r1, r1, #0           /* If f16Speed >= 0, then r1 = - r1 */

    movs    r2, r0               /* r2 = r0 */
    it      mi                   /* If r2 < 0, then r2 = ~r2 */
    mvnmi   r2, r2               /* r2 = ~r2 */
    clz     r2, r2               /* Count of leading bits */

    movs    r3, r1               /* r3 = r0 */
    it      mi                   /* If r3 < 0, then r3 = ~r3 */
    mvnmi   r3, r3               /* r3 = ~r3 */
    clz     r3, r3               /* Count of leading bits */

    cmp     r3, r2               /* Compares r3 and r2 */
    it      gt                   /* If r3 > r2, then r3 = r2 */
    movgt   r3, r2               /* r3 = r2 */

    sub     r3, r3, #1           /* r3 = r3 - 1 */
    lsl     r0, r0, r3           /* r0 << r3 */
    lsl     r1, r1, r3           /* r1 << r3 */
    asr     r0, r0, #16          /* r0 >> 16 */
    asr     r1, r1, #16          /* r1 >> 16 */

    /* Error BEMF calculation in gamma/delta coordinates */
    mul     r2, r0, r1           /* r2 = r0*r1 */
    cmp     r2, #0               /* Compares r2 with 0 */
    bne     AtanYX_F16_NotError  /* If r0*r1 != 0, then computes the AtanYX function*/

    cmp     r1, #0               /* Compares f16X with 0 */
    ittt    eq                   /* If f16X = 0, then executes next two commands */
    ssateq  r0, #16, r0, lsl #16 /* Saturates (r0 << 16) to 16-bit value */
    addeq   r0, r0, #1           /* r0 = r0 + 1 */
    asreq   r0, r0, #1           /* r0 = r0 >> 1 */

    cmp     r0, #0               /* Compares f16Y with 0 */
    itt     eq                   /* If f16X = 0, then executes next two commands */
    ssateq  r0, #1, r1           /* Saturates r0 to a 2-bit value */
    lsreq   r0, r0, #17          /* r0 >> 17 */

    strh    r0, [r4]             /* psCtrl->f16Error = r0 */
    pop     {r4-r7, pc}          /* Restores r4 - r7 and pc from the stack */

ASM_LABEL(AtanYX_F16_NotError)
    mov     r5, #0               /* r5 = 0 */
    mov     r6, r0               /* r6 = f16Y */

    ittt    lt                   /* If f16Y * f16X < 0, then executes next three commands */
    movlt   r5, #0x4000          /* r5 = 0x4000 */
    rsblt   r0, r1, #0           /* r0 = - f16X */
    movlt   r1, r6               /* r1 = f16Y */

    cmp     r6, #0               /* Compares f16Y with 0 */
    ittt    lt                   /* If f16Y < 0, then executes next three commands */
    orrlt   r5, r5, #0x8000      /* r5 = r5 | 0x8000 */
    rsblt   r0, r0, #0           /* r0 = - r0 */
    rsblt   r1, r1, #0           /* r1 = - r1 */
    sxth    r5, r5               /* Sets sign of r5 */

    cmp     r0, r1               /* Compares r0 and r1 */
    bgt     AtanYX_F16_L1        /* If r0 > r1, then goes to AtanYX_F16_L1 */
    blt     AtanYX_F16_L2        /* If r0 < r1, then goes to AtanYX_F16_L1 */
    add     r0, r5, #0x2000      /* r0 = r5 + 0x2000 */
    strh    r0, [r4]             /* psCtrl->f16Error = r0 */
    pop     {r4-r7, pc}          /* Restores r4 - r7 and pc from the stack */

ASM_LABEL(AtanYX_F16_L1)
    add     r5, r5, #0x2000      /* r5 = r5 + 0x2000 */
    mov     r6, r0               /* r6 = r0 */
    sub     r0, r0, r1           /* r0 = r0 - r1 */
    adds    r1, r1, r6           /* r1 = r1 + r6 */

ASM_LABEL(AtanYX_F16_L2)
    lsl     r0, r0, #15          /* r0 << 15 */
    sdiv    r0, r0, r1           /* r0 = r0 / r1 */
    ldr     r1, =gsAtanCoef      /* r1 = pointer to the gsAtanCoef */

    bl      GFLIB_Atan_F16_FAsm  /* r0 = Atan(r0) */
    add     r0, r0, r5           /* r0 = r0 + r5 */
    ssat    r0, #16, r0          /* Saturation */

    strh    r0, [r4]             /* psCtrl->f16Error = r0 */
	nop                          /* to avoid warning */ 
	pop     {r4-r7, pc}          /* Restores r4 - r7 and pc from the stack */

    ASM_END
