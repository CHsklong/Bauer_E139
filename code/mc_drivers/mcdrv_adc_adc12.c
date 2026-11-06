/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
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

#include "mcdrv_adc_adc12.h"
#include "adc.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
* Variables
******************************************************************************/

static bool_t s_statusPass;
extern uint16_t u_av_adc ;
extern uint16_t v_av_adc ;
extern uint16_t w_av_adc ;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Reads and calculates 3 phase samples based on SVM sector
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
//frac16_t g_tempA,g_tempB,g_tempC;
bool_t MCDRV_Curr3Ph2ShGet(mcdrv_adc12_t *this)
{
    GMCLIB_3COOR_T_F16 sIABCtemp;

    s_statusPass = TRUE;
#if (BOARD_TYPE == BOARD_E145) //2phase
    sIABCtemp.f16A = MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec45.pui16ClistPhaA))) -this->sCurrSec45.ui16OffsetPhaA ),0);
    sIABCtemp.f16B = MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec45.pui16ClistPhaB))) -this->sCurrSec45.ui16OffsetPhaB ),0);
    sIABCtemp.f16C = 0 ;
    #if 0
	switch (*this->pui16SVMSector)
    {
        case 2:
        case 3:
            /* direct sensing of phase A and C, calculation of B */
            sIABCtemp.f16A =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec23.pui16ClistPhaA))) -
                                 this->sCurrSec23.ui16OffsetPhaA ),
                                0);
            sIABCtemp.f16C =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec23.pui16ClistPhaC))) -
                                 this->sCurrSec23.ui16OffsetPhaC),
                                0);
            sIABCtemp.f16B = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16A, sIABCtemp.f16C));
            break;

        case 4:
        case 5:
            /* direct sensing of phase A and B, calculation of C */
            sIABCtemp.f16A =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec45.pui16ClistPhaA))) -
                                 this->sCurrSec45.ui16OffsetPhaA),
                                0);
            sIABCtemp.f16B =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec45.pui16ClistPhaB))) -
                                 this->sCurrSec45.ui16OffsetPhaB),
                                0);
            sIABCtemp.f16C = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16A, sIABCtemp.f16B));
    
            break;

        case 1:
        case 6:
        default:
            /* direct sensing of phase B and C, calculation of A */
            sIABCtemp.f16B =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec16.pui16ClistPhaB))) -
                                 this->sCurrSec16.ui16OffsetPhaB),
                                0);
            sIABCtemp.f16C =
                MLIB_ShLSat_F16(((frac16_t)((int16_t)(*(this->sCurrSec16.pui16ClistPhaC))) -
                                 this->sCurrSec16.ui16OffsetPhaC),
                                0);
            sIABCtemp.f16A = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16B, sIABCtemp.f16C));
            break;
    }
    #endif
#else
    switch (*this->pui16SVMSector)
    {
        case 2:
        case 3:
            /* direct sensing of phase A and C, calculation of B */
            sIABCtemp.f16A =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec23.pui16ClistPhaA))) -
                                 this->sCurrSec23.ui16OffsetPhaA ),
                                0);
            sIABCtemp.f16C =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec23.pui16ClistPhaC))) -
                                 this->sCurrSec23.ui16OffsetPhaC),
                                0);
            sIABCtemp.f16B = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16A, sIABCtemp.f16C));
            break;

        case 4:
        case 5:
            /* direct sensing of phase A and B, calculation of C */
            sIABCtemp.f16A =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec45.pui16ClistPhaA))) -
                                 this->sCurrSec45.ui16OffsetPhaA),
                                0);
            sIABCtemp.f16B =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec45.pui16ClistPhaB))) -
                                 this->sCurrSec45.ui16OffsetPhaB),
                                0);
            sIABCtemp.f16C = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16A, sIABCtemp.f16B));
    
            break;

        case 1:
        case 6:
        default:
            /* direct sensing of phase B and C, calculation of A */
            sIABCtemp.f16B =
                MLIB_ShLSat_F16(((frac16_t)((int16_t) (*(this->sCurrSec16.pui16ClistPhaB))) -
                                 this->sCurrSec16.ui16OffsetPhaB),
                                0);
            sIABCtemp.f16C =
                MLIB_ShLSat_F16(((frac16_t)((int16_t)(*(this->sCurrSec16.pui16ClistPhaC))) -
                                 this->sCurrSec16.ui16OffsetPhaC),
                                0);
            sIABCtemp.f16A = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16B, sIABCtemp.f16C));
            break;
    }
#endif
   

    /* pass measured phase currents to the main module structure */
    this->psIABC->f16A = sIABCtemp.f16A;
    this->psIABC->f16B = sIABCtemp.f16B;
    this->psIABC->f16C = sIABCtemp.f16C;

    return (s_statusPass);
}

/*!
 * @brief Set initial channel assignment for phase currents & DCB voltage
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_Curr3Ph2ShChanAssignInit(mcdrv_adc12_t *this)
{
    s_statusPass = TRUE;
    #if (BOARD_TYPE == BOARD_E145) //2phase
    this->sCurrSec45.pui16ClistPhaA = (uint16_t *)(&u_av_adc) ;
    this->sCurrSec45.pui16ClistPhaB = (uint16_t *)(&v_av_adc) ;
    #else
    this->sCurrSec45.pui16ClistPhaA = (uint16_t *)(&u_av_adc) ;
    this->sCurrSec45.pui16ClistPhaB = (uint16_t *)(&v_av_adc) ;
    this->sCurrSec16.pui16ClistPhaB = (uint16_t *)(&v_av_adc) ;
    this->sCurrSec16.pui16ClistPhaC = (uint16_t *)(&w_av_adc) ;
    this->sCurrSec23.pui16ClistPhaA = (uint16_t *)(&u_av_adc) ;
    this->sCurrSec23.pui16ClistPhaC = (uint16_t *)(&w_av_adc) ;
    #endif
  //  this->pf16UDcBus = (frac16_t *)(&adc0_sample[4]) ;

    return (s_statusPass);
}

/*!
 * @brief Set new channel assignment for next sampling based on SVM sector
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_Curr3Ph2ShChanAssign(mcdrv_adc12_t *this)
{
    s_statusPass = TRUE;

    return (s_statusPass);
}

/*!
 * @brief Initializes phase current channel offset measurement
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_Curr3Ph2ShCalibInit(mcdrv_adc12_t *this)
{
    s_statusPass = TRUE;

    /* clear offset values */
    this->sCurrSec16.ui16OffsetPhaB = 0;
    this->sCurrSec16.ui16OffsetPhaC = 0;
    this->sCurrSec23.ui16OffsetPhaA = 0;
    this->sCurrSec23.ui16OffsetPhaC = 0;
    this->sCurrSec45.ui16OffsetPhaA = 0;
    this->sCurrSec45.ui16OffsetPhaB = 0;

    this->sCurrSec16.ui16CalibPhaB = 0;
    this->sCurrSec16.ui16CalibPhaC = 0;
    this->sCurrSec23.ui16CalibPhaA = 0;
    this->sCurrSec23.ui16CalibPhaC = 0;
    this->sCurrSec45.ui16CalibPhaA = 0;
    this->sCurrSec45.ui16CalibPhaB = 0;

    /* initialize offset filters */
    this->sCurrSec16.ui16FiltPhaB.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec16.ui16FiltPhaC.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec23.ui16FiltPhaA.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec23.ui16FiltPhaC.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec45.ui16FiltPhaA.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec45.ui16FiltPhaB.u16Sh = this->ui16OffsetFiltWindow;

    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec16.ui16FiltPhaB);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec16.ui16FiltPhaC);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec23.ui16FiltPhaA);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec23.ui16FiltPhaC);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec45.ui16FiltPhaA);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec45.ui16FiltPhaB);

    return (s_statusPass);
}

/*!
 * @brief Function reads current samples and filter them based on SVM sector
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_Curr3Ph2ShCalib(mcdrv_adc12_t *this)
{
    s_statusPass = TRUE;
#if (BOARD_TYPE == BOARD_E145) //2phase
    this->sCurrSec45.ui16CalibPhaA = GDFLIB_FilterMA_F16(
                (frac16_t) ((int16_t)( (*(this->sCurrSec45.pui16ClistPhaA))-32767)), 
                          &this->sCurrSec45.ui16FiltPhaA);
    this->sCurrSec45.ui16CalibPhaB = GDFLIB_FilterMA_F16(
                (frac16_t) ((int16_t)( (*(this->sCurrSec45.pui16ClistPhaB))-32767)), 
                          &this->sCurrSec45.ui16FiltPhaB);
#else
    switch (*this->pui16SVMSector)
    {
        case 2:
        case 3:
            /* direct sensing of measurement offsets on phases A and C */
            this->sCurrSec23.ui16CalibPhaA = GDFLIB_FilterMA_F16(
                (frac16_t) ((int16_t) ((*(this->sCurrSec23.pui16ClistPhaA))-32767)), 
                          &this->sCurrSec23.ui16FiltPhaA);
            this->sCurrSec23.ui16CalibPhaC = GDFLIB_FilterMA_F16(
                (frac16_t) ((int16_t) ((*(this->sCurrSec23.pui16ClistPhaC))-32767)), 
                           &this->sCurrSec23.ui16FiltPhaC);
            break;

        case 4:
        case 5:
            /* direct sensing of measurement offsets on phases A and B */
            this->sCurrSec45.ui16CalibPhaA = GDFLIB_FilterMA_F16(
                (frac16_t) ((int16_t)( (*(this->sCurrSec45.pui16ClistPhaA))-32767)), 
                          &this->sCurrSec45.ui16FiltPhaA);
            this->sCurrSec45.ui16CalibPhaB = GDFLIB_FilterMA_F16(
                (frac16_t) ((int16_t)( (*(this->sCurrSec45.pui16ClistPhaB))-32767)), 
                          &this->sCurrSec45.ui16FiltPhaB);
            
          //  this->sCurrSec45.ui16CalibPhaB = (frac16_t) ((int16_t)( (*(this->sCurrSec45.pui16ClistPhaB))-32767));
             
                           
            break;

        case 1:
        case 6:
        default:
            /* direct sensing of measurement offsets on phases B and C */
            this->sCurrSec16.ui16CalibPhaB = GDFLIB_FilterMA_F16(
                (frac16_t) ((int16_t) ((*(this->sCurrSec16.pui16ClistPhaB))-32767)), 
                          &this->sCurrSec16.ui16FiltPhaB);
            this->sCurrSec16.ui16CalibPhaC = GDFLIB_FilterMA_F16(
                (frac16_t) ((int16_t) ((*(this->sCurrSec16.pui16ClistPhaC))-32767)), 
                          &this->sCurrSec16.ui16FiltPhaC);
            break;
    }
#endif
    return (s_statusPass);
}

/*!
 * @brief Function passes measured offset values to main structure
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_Curr3Ph2ShCalibSet(mcdrv_adc12_t *this)
{
    s_statusPass = TRUE;

    /* pass calibration data for sector 1 and 6 */
    this->sCurrSec16.ui16OffsetPhaB = this->sCurrSec16.ui16CalibPhaB+32767 ;
    this->sCurrSec16.ui16OffsetPhaC = this->sCurrSec16.ui16CalibPhaC+32767;

    /* pass calibration data for sector 2 and 3 */
    this->sCurrSec23.ui16OffsetPhaA = this->sCurrSec23.ui16CalibPhaA+32767;
    this->sCurrSec23.ui16OffsetPhaC = this->sCurrSec23.ui16CalibPhaC+32767;

   
    /* pass calibration data for sector 4 and 5 */
    this->sCurrSec45.ui16OffsetPhaA = this->sCurrSec45.ui16CalibPhaA+32767;
    this->sCurrSec45.ui16OffsetPhaB = this->sCurrSec45.ui16CalibPhaB+32767;
   /* this->sCurrSec45.ui16OffsetPhaA = 32768;
    this->sCurrSec45.ui16OffsetPhaB = 32768;*/
    return (s_statusPass);
}

/*!
 * @brief Function reads and passes DCB voltage sample
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_VoltDcBusGet(mcdrv_adc12_t *this)
{
    s_statusPass = TRUE;

    /* read DC-bus voltage sample from defined ADCx result register */
   // *this->pf16UDcBus = (frac16_t)(this->pui32AdcBase->RSLT[this->ui16AdcSmplVDcb]);
   	#if (BOARD_TYPE == BOARD_E145 || BOARD_TYPE == BOARD_E139 || BOARD_TYPE == BOARD_E884) 
    *this->pf16UDcBus = (adc0_buff[4] & 0xffff)>>1;
  	#else
    *this->pf16UDcBus =adc0_buff[4] & 0xffff;
	#endif
    return (s_statusPass);
}

/*!
 * @brief Function reads and passes auxiliary sample
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_AuxValGet(mcdrv_adc12_t *this)
{
    s_statusPass = TRUE;

    /* read Auxiliary channel sample from defined ADCx result register */
  //  *this->pui16AuxChan = (frac16_t)(this->pui32AdcBase->RSLT[this->ui16AdcSmplAux]);

    return (s_statusPass);
}

