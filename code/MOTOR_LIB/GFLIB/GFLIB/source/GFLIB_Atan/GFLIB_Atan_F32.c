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

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GFLIB_Atan_F32.h"
  
/*******************************************************************************
* Global variables
*******************************************************************************/
GFLIB_CONST GFLIB_ATAN_T_F32 gsAtanCoef = {{
        {{-164794,    42515925,    42667172}},     /* Coef. [0;1/8) */
        {{-465182,    41238272,   126697014}},     /* Coef. [1/8;2/8) */
        {{-690034,    38899574,   207041074}},     /* Coef. [2/8;3/8) */
        {{-820713,    35848645,   281909001}},     /* Coef. [3/8;4/8) */
        {{-865105,    32453241,   350251355}},     /* Coef. [4/8;5/8) */
        {{-845462,    29016149,   411702516}},     /* Coef. [5/8;6/8) */
        {{-786689,    25743137,   466407809}},     /* Coef. [6/8;7/8) */
        {{-708969,    22748418,   514828039}}      /* Coef. [7/8;1) */
}};
