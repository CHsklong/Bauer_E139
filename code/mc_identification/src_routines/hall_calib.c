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

#include "hall_calib.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Hall calibration routine
 *
 * @param *sHallCalibFcn input structure of type #mid_calib_hall_t for passing
 *                          all necessary parameters.
 *
 * @return None
 */
void MID_calibHall(mid_calib_hall_t* sHallCalibFcn)
{
    /* Initialization */
    if(sHallCalibFcn->ui16Active == 0)
    {
        sHallCalibFcn->ui16Active   = TRUE;
        sHallCalibFcn->ui32CalibrationCounter = 0U;
        *(sHallCalibFcn->pbCalibrationStartStop) = FALSE;
        *(sHallCalibFcn->pbCalibrationDirection) = TRUE; //positive speed 
        sHallCalibFcn->fltFreqElRamp                   = 0.0;
        sHallCalibFcn->sFreqElRampParam.fltRampUp      = (double)sHallCalibFcn->fltFreqElReq / MID_SPEED_RAMP_TIME / 10000;
        sHallCalibFcn->sFreqElRampParam.fltRampDown    = (double)sHallCalibFcn->fltFreqElReq / MID_SPEED_RAMP_TIME / 10000;

        sHallCalibFcn->sFreqIntegrator.a32Gain         = ACC32(1 * (double)sHallCalibFcn->fltFreqMax / 10000 * 2);
        GFLIB_IntegratorInit_F16(0, &sHallCalibFcn->sFreqIntegrator);
        GFLIB_RampInit_FLT(0.0, &sHallCalibFcn->sFreqElRampParam);
    }
    
    sHallCalibFcn->ui32CalibrationCounter++;
    
    /* After 32s start calculation */
    if(sHallCalibFcn->ui32CalibrationCounter == MID_TIME_16S)
    {
        /* turn off calibration */
        *(sHallCalibFcn->pbCalibrationStartStop) = FALSE;
        /* select calibration for negative speed */
        *(sHallCalibFcn->pbCalibrationDirection) = FALSE; //negative speed
        /* select negative speed */
        sHallCalibFcn->fltFreqElReq = MLIB_Neg_FLT(sHallCalibFcn->fltFreqElReq);
    }
    
    /* Set Id required */
    *(sHallCalibFcn->pfltIdReq) = sHallCalibFcn->fltIdReqOpenLoop;
    /* Ramp electrical speed */
    sHallCalibFcn->fltFreqElRamp = GFLIB_Ramp_FLT(sHallCalibFcn->fltFreqElReq, &sHallCalibFcn->sFreqElRampParam);
    /* Integrate electrical speed to get electrical position */
    *sHallCalibFcn->pf16PosEl = GFLIB_Integrator_F16(MLIB_ConvSc_F16ff(sHallCalibFcn->fltFreqElRamp, sHallCalibFcn->fltFreqMax), &sHallCalibFcn->sFreqIntegrator);
    
    /* ked dosiahne pozadovanu rychlost, zapne kalibraciu */
    if(sHallCalibFcn->fltFreqElRamp == sHallCalibFcn->fltFreqElReq)
    {
        /* turn on calibration */
        *(sHallCalibFcn->pbCalibrationStartStop) = TRUE;
    }

    /* After 32s start calculation */
    if(sHallCalibFcn->ui32CalibrationCounter == MID_TIME_32S)
    {
      /* turn off calibration */
        *(sHallCalibFcn->pbCalibrationStartStop) = FALSE;
        sHallCalibFcn->ui16Active   = FALSE;
        sHallCalibFcn->ui16HallCalibDone = TRUE;
    }
    
}


