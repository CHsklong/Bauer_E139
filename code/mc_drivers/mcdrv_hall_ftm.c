/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#include "mcdrv_ftm_hall.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define HALL_EDGE_TOTAL (6.0)

/*******************************************************************************
 * Variables
 ******************************************************************************/

static bool_t    statusPass;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief   Initializes the driver
 *
 * @param   this   Pointer to the current object
 *          init   Pointer to initialization structure
 *
 * @return  boot_t true on success
 */

bool_t MCDRV_FtmHsInit(mcdrv_ftm_hs_t *this)
{
    statusPass = TRUE;
    
    /* Init Speed integrator */
    this->sSpeedIntegrator.f32IAccK_1   = 0;
    this->sSpeedIntegrator.f16InValK_1  = 0;
    
    /* Init Tracking observer */
    this->sPositionTO.f16IGain      = HALL_TO_KI_GAIN;
    this->sPositionTO.i16IGainSh    = HALL_TO_KI_SHIFT;
    this->sPositionTO.f16PGain      = HALL_TO_KP_GAIN;
    this->sPositionTO.i16PGainSh    = HALL_TO_KP_SHIFT;
    this->sPositionTO.f16ThGain     = HALL_TO_THETA_GAIN;
    this->sPositionTO.i16ThGainSh   = HALL_TO_THETA_SHIFT;
        
    /* Hall state 0 */
    this->ui8HallState = 0U;
    
    /* Init speed filter */
    this->sSpeedFilter.sFltCoeff.f32A1 = SPEED_IIR_FILT_A1;
    this->sSpeedFilter.sFltCoeff.f32B0 = SPEED_IIR_FILT_B0;
    this->sSpeedFilter.sFltCoeff.f32B1 = SPEED_IIR_FILT_B1;
  
    /* Init default hall sensor placement */
    this->f16HallPlacementNeg[0] = FRAC16(-96.12/180.0);
    this->f16HallPlacementNeg[1] = FRAC16(24.15/180.0);
    this->f16HallPlacementNeg[2] = FRAC16(-37.69/180.0);
    this->f16HallPlacementNeg[3] = FRAC16(143.15/180.0);
    this->f16HallPlacementNeg[4] = FRAC16(-155.5/180.0);
    this->f16HallPlacementNeg[5] = FRAC16(84.18/180.0);
    
    this->f16HallPlacementPos[0] = FRAC16(-145.8/180.0);
    this->f16HallPlacementPos[1] = FRAC16(-31.88/180.0);
    this->f16HallPlacementPos[2] = FRAC16(-87.94/180.0);
    this->f16HallPlacementPos[3] = FRAC16( 82.77/180.0);
    this->f16HallPlacementPos[4] = FRAC16(138.57/180.0);
    this->f16HallPlacementPos[5] = FRAC16( 24.23/180.0);
    
    /* Init default hall order */
    this->ui8HallStatesOrder[0] = 6U;
    this->ui8HallStatesOrder[1] = 2U;
    this->ui8HallStatesOrder[2] = 1U;
    this->ui8HallStatesOrder[3] = 4U;
    this->ui8HallStatesOrder[4] = 5U;
    this->ui8HallStatesOrder[5] = 3U;
    
    /* Init calibration filter */
    this->sCalibrationFilter[0].u16Sh = 6;
    this->sCalibrationFilter[1].u16Sh = 6;
    this->sCalibrationFilter[2].u16Sh = 6;
    this->sCalibrationFilter[3].u16Sh = 6;
    this->sCalibrationFilter[4].u16Sh = 6;
    this->sCalibrationFilter[5].u16Sh = 6;
        
    return(statusPass);
}

/*!
 * @brief   Calculates electrical position and speed
 *
 * @param   this   Pointer to the current object
 *
 * @return  boot_t true on success
 */
bool_t MCDRV_FtmHsGetSpeedPosition(mcdrv_ftm_hs_t *this)
{
    frac16_t f16PosElIntegrated;
    
    statusPass = TRUE;
    
    /* Read FTM counter */
    this->ui16FtmCounter =  (uint16_t) this->pui32FtmBase->CONTROLS[1].CnV;
    
    /* Check overspeed (too low counter) */
    if (this->ui16FtmCounter > this->ui16PeriodToSpeed)
    {
        /* Calculate positive/negative speed */
        if(this->bDirection)
            this->f16Speed = MLIB_Div_F16(this->ui16PeriodToSpeed, this->ui16FtmCounter);    
        else
            this->f16Speed = MLIB_Neg_F16(MLIB_Div_F16(this->ui16PeriodToSpeed, this->ui16FtmCounter));
    }
    else
    {
        /* If overspeed, return zero speed */
        this->f16Speed = FRAC16(0.0);
    }
    
    /* Filter mechanical angular speed */
    this->f16SpeedFilt = GDFLIB_FilterIIR1_F16(this->f16Speed, &this->sSpeedFilter);
        
    /* Calculate position (integrate speed) */
    f16PosElIntegrated = GFLIB_Integrator_F16(this->f16SpeedFilt, &this->sSpeedIntegrator);
    
    /* Estimate position using Tracking observer - effectively removes spikes */
    this->f16PosEl = AMCLIB_TrackObsrv_F16(MLIB_Sub_F16(f16PosElIntegrated, this->f16PosEl_K_1), &this->sPositionTO);
     
    /* Remember last position for the next loop step */
    this->f16PosEl_K_1 = this->f16PosEl;
    
    /* Pass non-filtered speed out of the driver */    
    *this->pfltSpeedOut = MLIB_ConvSc_FLTsf(this->f16Speed, (this->fltSpeedMaxScale));
    
    /* Pass Electrical position out of the driver */
    *this->pf16PosElOut = this->f16PosEl;
    
    return(statusPass);
}

/*!
 * @brief   Corrects (reads from array) electrical position at the moment of the edge based on previous calibration
 *
 * @param   this   Pointer to the current object
 *
 * @return  boot_t true on success
 */
bool_t MCDRV_FtmCorrectPosition(mcdrv_ftm_hs_t *this)
{
    statusPass = TRUE;
    
    /* Depending on direction, fill speed integrator's accumulator with calibrated position */
    if(this->bDirection)
        this->sSpeedIntegrator.f32IAccK_1 = MLIB_Conv_F32s(this->f16HallPlacementPos[this->ui8HallState-1]);
    else
        this->sSpeedIntegrator.f32IAccK_1 = MLIB_Conv_F32s(this->f16HallPlacementNeg[this->ui8HallState-1]);
    
    return(statusPass);
}

/*!
 * @brief   Determines hall sensors' order and calibrates position at the moment of edge
 *
 * @param   this   Pointer to the current object
 *
 * @return  boot_t true on success
 */
bool_t MCDRV_FtmHsCalibration(mcdrv_ftm_hs_t *this)
{
    statusPass = TRUE;
    
    /* At positiove direction (set from outside) determine also the hall sensors' order*/
    if(this->bCalibrationDirection)
    {
        /* At positive direction store order of Hall sensors' states */
        this->ui8HallStatesOrder[this->ui8HallState-1] = this->ui8HallStatesOrder[this->ui8HallStateK_1-1]%6 + 1;
        
        /* Save position at the moment of edge */
        this->f16HallPlacementPos[this->ui8HallState-1] = GDFLIB_FilterMA_F16(*this->pf16PosEstim, &(this->sCalibrationFilter[this->ui8HallState-1]));
    }
    
    else
    {
        /* Save position at the moment of edge */
        this->f16HallPlacementNeg[this->ui8HallState-1] = *this->pf16PosEstim;
    }
    
    return(statusPass);
}

/*!
 * @brief   Reads current status (sector) of the hall sensors
 *
 * @param   this   Pointer to the current object
 *
 * @return  boot_t true on success
 */
bool_t MCDRV_FtmHsGetStatus(mcdrv_ftm_hs_t *this)
{
    uint8_t ui8HallStateTemp;
  
    statusPass = TRUE;
    
    /* Remeber last hall state before it is overwritten */
    ui8HallStateTemp = this->ui8HallState;
    
    /* Hall state is determined as weighted logical OR of all hall sensors */
    /* Sensor C */
    if(this->pui32HS3Port->PDIR & (this->ui32HS3PinShift))
      this->ui8HallState |= (1 << 2);
    else
      this->ui8HallState &= ~(1 << 2);    
    
    /* Sensor B */
    if(this->pui32HS2Port->PDIR & (this->ui32HS2PinShift))
      this->ui8HallState |= (1 << 1);
    else
      this->ui8HallState &= ~(1 << 1);
    
    /*Sensor A */
    if(this->pui32HS1Port->PDIR & (this->ui32HS1PinShift))
      this->ui8HallState |= (1 << 0);
    else
      this->ui8HallState &= ~(1 << 0);

    /* If hall state changed, update state k-1 */
    if (this->ui8HallState != ui8HallStateTemp)
        this->ui8HallStateK_1 = ui8HallStateTemp;
    
    return(statusPass);
}

/*!
 * @brief   Calculates mean electrical position based on current hall state
 *
 * @param   this   Pointer to the current object
 *
 * @return  boot_t true on success
 */
bool_t MCDRV_FtmGetSectorMeanPos(mcdrv_ftm_hs_t *this)
{
    statusPass = TRUE;
    
    /* Read hall sensor state */
    MCDRV_FtmHsGetStatus(this);
    
    /* Calculate average (mean) position in current sector */
    this->f16PosElSector = MLIB_Add_F16(this->f16HallPlacementPos[this->ui8HallState-1], FRAC16(30.0/180.0));
      
    return(statusPass);
}

/*!
 * @brief   Clears hall driver structure accumulators
 *
 * @param   this   Pointer to the current object
 *
 * @return  boot_t true on success
 */
bool_t MCDRV_FtmHsClear(mcdrv_ftm_hs_t *this)
{
    statusPass = TRUE;
    
    this->f16Speed                    = FRAC16(0.0);
    this->f16SpeedFilt                = FRAC16(0.0);
    this->f16PosEl                      = FRAC16(0.0);
    this->sSpeedFilter.f16FltBfrX[0]    = FRAC16(0.0);
    this->sSpeedFilter.f32FltBfrY[0]    = FRAC32(0.0);
    this->sSpeedIntegrator.f32IAccK_1   = FRAC32(0.0);
    this->sSpeedIntegrator.f16InValK_1  = FRAC16(0.0);
    this->sCalibrationFilter[0].a32Acc  = ACC32(0.0);
    this->sCalibrationFilter[1].a32Acc  = ACC32(0.0);
    this->sCalibrationFilter[2].a32Acc  = ACC32(0.0);
    this->sCalibrationFilter[3].a32Acc  = ACC32(0.0);
    this->sCalibrationFilter[4].a32Acc  = ACC32(0.0);
    this->sCalibrationFilter[5].a32Acc  = ACC32(0.0);
    this->ui16EdgeCounter               = 0;
    this->ui8HallState                  = 1U;
    this->ui8HallStateK_1               = 1U;
      
    return(statusPass);
}

/*!
 * @brief   Deterines motor direction based on hall sensor order
 *
 * @param   this   Pointer to the current object
 *
 * @return  boot_t true on success
 */
bool_t MCDRV_FtmGetDirection(mcdrv_ftm_hs_t *this)
{
    statusPass = TRUE;
    
    /* Determine motor direction based on hall sensor order. If the difference between current and last hall state is negative, motor direction is positive */
    if((this->ui8HallStatesOrder[this->ui8HallState-1] - this->ui8HallStatesOrder[this->ui8HallStateK_1-1]) == -1)
        this->bDirection = 0;
    
    else if((this->ui8HallStatesOrder[this->ui8HallState-1] - this->ui8HallStatesOrder[this->ui8HallStateK_1-1]) == 1)
        this->bDirection = 1;
    
//    else
//      ERROR
    
    return(statusPass);
}

