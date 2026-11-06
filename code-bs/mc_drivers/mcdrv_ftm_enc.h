/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _MCDRV_FTM_ENC_H_
#define _MCDRV_FTM_ENC_H_

#include "mlib.h"
#include "mlib_types.h"
//#include "fsl_device_registers.h"
#include "amclib.h"
#include "pmsm_control.h"
//#include "m1_pmsm_align.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MAX_ALIGN_CNT      (64)

typedef struct _mcdrv_ftm_abs_align
{
    int16_t  s16AlignPos[2][MAX_ALIGN_CNT];               /*align position*/
    int16_t  s16AlignPosDista[MAX_ALIGN_CNT];
    int16_t  s16Zero;
} mcdrv_ftm_abs_align_t;

typedef struct _mcdrv_ftm_uvw_align
{
    int32_t s32ZeroOffset;  
    int32_t s32ZeroCnt;          
    uint8_t bOverZero;                  
} mcdrv_ftm_uvw_align_t;

typedef union _mcdrv_ftm_align
{
    mcdrv_ftm_abs_align_t sAbcAlignData;
    mcdrv_ftm_uvw_align_t sUvwAlignData;
} mcdrv_ftm_align_t;

#define ENC_TYPE_ABS            (0)
#define ENC_TYPE_UVW            (1)

typedef struct _mcdrv_ftm_enc
{
    AMCLIB_TRACK_OBSRV_T_FLT        sTo;                                        /* tracking observer structure */
//    FTM_Type                       *pui32FtmBase;                                /* pointer to QD module base address*/  
    float_t                        *pfltSpdMeEst;                               /* pointer to measured mechanical speed  */    
    frac16_t                       *pf16PosElEst;                               /* pointer to measured electrical position */
    acc32_t                         a32PosErr;                                  /* position error to tracking observer  */    
    acc32_t                         a32PosMeGain;                               /* encoder pulses to mechanical position scale gain */ 
    int8_t                          i8EncDir;									/* encoder direction*/
    float_t                         fltSpdMeEst;                                /* estimated speed calculated using tracking observer */            
    frac16_t                        f16PosMe;                                   /* mechanical position calculated using encoder edges */
    frac16_t                        f16PosMe1;                                   /* mechanical position calculated using encoder edges */
    int16_t                         i16ZCnt;									/* number of circle  */
    int16_t                         i16ZCnt1;									/* number of circle  */
    frac16_t                        f16PosMeEst;                                /* estimated position calculated using tracking observer */        
    //uint16_t                        ui16Pp;                                   /* number of motor pole pairs */ //PB vymaz
    uint16_t                        *pui16Pp;
    uint8_t                         bNeedAlign;
    uint8_t                         encType;
    mcdrv_ftm_align_t               encAlignData;
}   mcdrv_ftm_enc_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

uint32_t MCDRV_FtmEncGetCnt(mcdrv_ftm_enc_t *this);
uint32_t MCDRV_FtmEncGetAbsCnt(mcdrv_ftm_enc_t *this);

/*!
 * @brief Function returns actual position and speed
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmEncGet(mcdrv_ftm_enc_t *this);

/*!
 * @brief Function clears internal variables and decoder counter
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmEncClear(mcdrv_ftm_enc_t *this);

/*!
 * @brief Function se mechanical position of quadrature encoder
 *
 * @param this     Pointer to the current object
 *        f16PosMe Mechanical position
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmEncSetPosMe(mcdrv_ftm_enc_t *this, frac16_t f16PosMe);

uint16_t get_clib_encode(uint8_t type) ;

#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_FTM_ENC_H_ */
