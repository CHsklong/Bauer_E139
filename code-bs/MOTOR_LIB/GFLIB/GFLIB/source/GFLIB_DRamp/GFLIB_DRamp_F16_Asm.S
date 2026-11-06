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
* @brief  Dynamic ramp
*
*******************************************************************************/
    #include "asm_mac.h"

    ASM_CODE_SECTION(.text)

    ASM_PUBLIC(GFLIB_DRamp_F16_FAsm)

    ASM_COMP_SPECIFIC_DIRECTIVES

/***************************************************************************//*!
*
* @brief  Ramp function
*
* @param  ptr  r3 - GFLIB_DRAMP_T_F16 *psParam
*                 - f16RampUp: Ramp-up increment
*                 - f16RampDown: Ramp-down increment
*                 - f16RampUpSat: Ramp-up increment used in case of saturation
*                 - f16RampDownSat: Ramp-down increment used in case of saturation
*                 - f16State: Previous ramp value
*                 - bReachFlag: Flag is set to 1 if the desired value is achieved
* @param  in   r0 - desired value in <-1;1) in frac16_t
*              r1 - instant value in <-1;1) in frac16_t
*              r2 - determines the saturation mode: 0 non-saturation
*
* @return This function returns
*             r3 - frac16_t value <-1;1)
*
* @remarks  This function ramps the value from the actual value up/down to
*           the f16Target value using the up/down increments defined in
*           the pParam structure. In case of saturation (pbStopFlag != 0)
*           the function uses the saturation up/down increments and ramps
*           the value toward the f16Instant value.
*
*******************************************************************************/
ASM_LABEL(GFLIB_DRamp_F16_FAsm)
    push    {r4 - r7}           /* Stores r4 - r7 to the stack */

    ldrsh   r4, [r3, #8]        /* r4 = psParam.f16State */

    movs    r7, r4              /* r7 = r4 */
    it      mi                  /* If r4 < 0, then r4 = - psParam.f16State */
    rsbmi   r4, r4, #0          /* r4 = -r4 */

    movs    r5, r0              /* r5 = f16Target */
    it      mi                  /* If r5 < 0, then r5 = - f16Target */
    rsbmi   r5, r5, #0          /* r5 = -r5 */

    ldrh    r6, [r2]            /* r6 = pbStopFlag */
    and     r6, r6, #1          /* r6 = r6 & 1 */
    movs    r2, #0              /* r2 = 0 */

    cmp     r5, r4              /* Compares |f16Target| and |psParam.f16State| */
    it      gt                  /* If |f16Target| > |psParam.f16State|, then executes next command */
    lslgt   r2, r6, #2          /* r2 = r6 << 2 */

    cmp     r2, #0              /* Compares r2 with 0 */
    it      eq                  /* If r2 = 0, then executes next command */
    movseq  r1, r0              /* r1 = f16Target */

    ldrh    r4, [r3, r2]        /* r4 = psParam.f16RampUp */
    add     r6, r2, #2          /* r6 = r2 + 2 */
    ldrh    r5, [r3, r6]        /* r5 = psParam.f16RampDown */

    cmp     r1, r7              /* Compares f16Target with psParam.f16State */
    ittee   ge                  /* Condition block */
    addge   r7, r7, r4          /* Ramp Up */
    subge   r6, r7, r1          /* r6 = r7 - r1 */
    sublt   r7, r7, r5          /* Ramp Down */
    sublt   r6, r1, r7          /* r6 = r1 - r7 */

    cmp     r6, #0              /* Compares r6 with 0 */
    it      le                  /* If r6 <=0, then executes next command */
    ssatle  r1, #16, r7         /* r1 = r7 */

    movs    r2, #0              /* r2 = 0 */
    cmp     r0, r1              /* Compares Result with Target */
    it      eq                  /* If Result = Target, sets ReachFlag */
    moveq   r2, #1              /* r2 = 1 */

    strh    r2, [r3, #10]       /* r2 -> psParam.bReachFlag */

    strh    r1, [r3, #8]        /* r1 -> psParam.f16State */
    mov     r0, r1              /* Result = r1 */

    pop     {r4 - r7}           /* Restores r4 - r7 */
    bx      lr                  /* Returns */

    ASM_END
