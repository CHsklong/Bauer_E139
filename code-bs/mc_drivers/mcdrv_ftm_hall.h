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

#ifndef _MCDRV_FTM_HS_H_
#define _MCDRV_FTM_HS_H_

#include "mlib.h"
#include "mlib_types.h"
//#include "fsl_device_registers.h"
#include "amclib.h"
#include "pmsm_control.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct _mcdrv_ftm_hs
{
    GFLIB_INTEGRATOR_T_A32         sSpeedIntegrator;        /* Speed integrator structure */
    AMCLIB_TRACK_OBSRV_T_FLT       sTo;                     /* Hall sensor tracking observer */
    GDFLIB_FILTER_IIR1_T_F32       sSpeedFilter;            /* Speed filter */
    GDFLIB_FILTER_MA_T_A32         sCalibrationFilter[6];   /* Filter used at edge displacement calibration */
    frac16_t                       f16HallPlacementPos[6];  /* Calibrated rotor position at the moment of Hall edge at positive speed */
    frac16_t                       f16HallPlacementNeg[6];  /* Calibrated rotor position at the moment of Hall edge at negative speed */
    uint8_t                        ui8HallStatesOrder[6];   /* Hall states order at positive direction */
    //FTM_Type                      *pui32FtmBase;            /* Pointer to FTM module base address*/  
   // GPIO_Type                     *pui32HS1Port;            /* HS1 pin port base adress */   
   // GPIO_Type                     *pui32HS2Port;            /* HS2 pin port base adress */      
   // GPIO_Type                     *pui32HS3Port;            /* HS3 pin port base adress */    
    uint32_t                       ui32HS1PinShift;         /* HS1 pin number */
    uint32_t                       ui32HS2PinShift;         /* HS2 pin number */
    uint32_t                       ui32HS3PinShift;         /* HS3 pin number */
    uint16_t                       ui16PeriodToSpeed;       /* Period to speed multiplication constant */
    frac16_t                       f16Speed;                /* Motor mechanical speed */
    
    frac16_t                       f16SpeedFilt;            /* Motor mechanical speed filtered */
    frac16_t                       f16PosEl;                /* Electrical position */
    frac16_t                       f16PosElSector;          /* Average electrical position in current sector */
    
    frac16_t                      *pf16PosElEst;            /* Pointer where the actual electrical position is passed */
    float_t                       *pfltSpdMeEst;            /* Pointer where the actual mechanical speed is passed */  
    acc32_t                        a32PosErr;               /* position error to tracking observer  */ 
       
    frac16_t                      *pf16PosEstim;            /* Pointer where the actual position */
    uint8_t                        ui8HallState;            /* Actual Hall state */
    uint8_t                        ui8HallStateK_1;         /* Previous Hall state */
    uint16_t                       ui16FtmCounter;          /* Captured FTM counter value (between two edges) */
    float_t                        fltSpeedMaxScale;        /* Maximum mechanical speed in Hz */
    bool_t                         bDirection;              /* Motor direction determined based Hall state order */
    bool_t                         bCalibration;            /* Turns on/off Hall sensors calibration */
    bool_t                         bCalibrationDirection;   /* Motor direction during calibration (set by user externally) */
} mcdrv_ftm_hs_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

bool_t MCDRV_FtmHsGetSpeedPosition(mcdrv_ftm_hs_t *this);
bool_t MCDRV_FtmHsCalibration(mcdrv_ftm_hs_t *this);
bool_t MCDRV_FtmHsGetStatus(mcdrv_ftm_hs_t *this);
bool_t MCDRV_FtmGetSectorMeanPos(mcdrv_ftm_hs_t *this);
bool_t MCDRV_FtmHsClear(mcdrv_ftm_hs_t *this);
bool_t MCDRV_FtmCorrectPosition(mcdrv_ftm_hs_t *this);
bool_t MCDRV_FtmGetDirection(mcdrv_ftm_hs_t *this);

#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_FTM_HS_H_ */

