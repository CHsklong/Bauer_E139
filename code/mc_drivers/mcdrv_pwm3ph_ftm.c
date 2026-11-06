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
 
#include "mcdrv_pwm3ph_ftm.h"
#include "pwm.h"
#include "UserTypeDef.h"
#include "mcu.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static bool_t s_statusPass;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Initialization function of 3-phase PWM FTM structure initialization
 *
 * @param this   Pointer to the current object
 * @param init   Pointer to initialization structure
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmPwm3PhInit(mcdrv_ftm_pwm3ph_t *this, mcdrv_ftm_pwm3ph_init_t *init)
{
    s_statusPass = TRUE;
#if 0
    this->pui32PwmBase = init->pui32PwmBase; /* FTM Base address */

    this->ui16ChanTopPhA = 0 + init->ui16ChanPairNumPhA;    /* FTM channel number for Phase A top */
    this->ui16ChanBottomPhA = 1 + init->ui16ChanPairNumPhA; /* FTM channel number for Phase A bottom */
    this->ui16ChanTopPhB = 0 + init->ui16ChanPairNumPhB;    /* FTM channel number for Phase B top */
    this->ui16ChanBottomPhB = 1 + init->ui16ChanPairNumPhB; /* FTM channel number for Phase B bottom */
    this->ui16ChanTopPhC = 0 + init->ui16ChanPairNumPhC;    /* FTM channel number for Phase C top */
    this->ui16ChanBottomPhC = 1 + init->ui16ChanPairNumPhC; /* FTM channel number for Phase C bottom */

    this->ui16FaultFixNum = init->ui16FaultFixNum; /* FTM fixed-value over-current fault number */
    this->ui16FaultAdjNum = init->ui16FaultAdjNum; /* FTM adjustable over-current fault number */
#endif
    #if (BOARD_TYPE == BOARD_E145)
    this->ui16ChanTopPhA = 1;    /* FTM channel number for Phase A top */
    this->ui16ChanBottomPhA = 7; /* FTM channel number for Phase A bottom */
    this->ui16ChanTopPhB = 3;    /* FTM channel number for Phase B top */
    this->ui16ChanBottomPhB = 5; /* FTM channel number for Phase B bottom */
    #endif
    return (s_statusPass);
}

/*!
 * @brief FTM value register updates
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
frac16_t f16DutyA ,f16DutyB ;
bool_t MCDRV_FtmPwm3PhSet(mcdrv_ftm_pwm3ph_t *this)
{
    frac16_t f16DutyCycle;
    GMCLIB_3COOR_T_F16 sUABCtemp;
    s_statusPass = TRUE;

    /* pointer to duty cycle structure */
    sUABCtemp = *this->psUABC;

    #if (BOARD_TYPE == BOARD_E145)
    f16DutyCycle = MLIB_Mul_F16((PWM_RELOAD), sUABCtemp.f16A) ;
      f16DutyCycle = PWM_RELOAD - f16DutyCycle ;
    f16DutyA = f16DutyCycle ;
      if(f16DutyCycle >= PWM_RELOAD/2)  //
      {
          pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_0, CMP_INDEX_1, f16DutyCycle, PWM_RELOAD - f16DutyCycle);   
        //  pwm_update_raw_cmp_central_aligned(HPM_PWM0, CMP_INDEX_2, CMP_INDEX_3, 0, PWM_RELOAD );
          pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_2, CMP_INDEX_3, PWM_RELOAD/2, PWM_RELOAD/2 );

      }
      else//璐?
      {
          pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_0, CMP_INDEX_1, PWM_RELOAD/2, PWM_RELOAD/2 );
          pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_2, CMP_INDEX_3, f16DutyCycle, PWM_RELOAD-f16DutyCycle );
      }

     /* phase B */  
      f16DutyCycle = MLIB_Mul_F16((PWM_RELOAD), sUABCtemp.f16B);
      f16DutyCycle = PWM_RELOAD - f16DutyCycle ;
      f16DutyB = f16DutyCycle ;
      if(f16DutyCycle >= PWM_RELOAD/2)  //
      {
          pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_4, CMP_INDEX_5, f16DutyCycle, PWM_RELOAD - f16DutyCycle);
        
          pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_6, CMP_INDEX_7, PWM_RELOAD/2, PWM_RELOAD/2 );
         
      }
      else//璐?
      {
          pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_4, CMP_INDEX_5, PWM_RELOAD/2, PWM_RELOAD/2);
          pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, CMP_INDEX_6, CMP_INDEX_7, f16DutyCycle, PWM_RELOAD-f16DutyCycle );
      }
    #elif (BOARD_TYPE == BOARD_E139 || BOARD_TYPE == BOARD_E884)
      /* phase A */  
    f16DutyCycle = MLIB_Mul_F16((PWM_RELOAD/2), sUABCtemp.f16A);

    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, UH_PWM_OUTPIN, UL_PWM_OUTPIN, f16DutyCycle, PWM_RELOAD - f16DutyCycle);
     /* phase B */  
    f16DutyCycle = MLIB_Mul_F16((PWM_RELOAD/2), sUABCtemp.f16B);

    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, VH_PWM_OUTPIN, VL_PWM_OUTPIN, f16DutyCycle, PWM_RELOAD - f16DutyCycle);
     /* phase C*/
    f16DutyCycle = MLIB_Mul_F16((PWM_RELOAD/2), sUABCtemp.f16C);

    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, WH_PWM_OUTPIN, WL_PWM_OUTPIN, f16DutyCycle, PWM_RELOAD - f16DutyCycle);  
    #elif (BOARD_TYPE == BOARD_E914)  
      /* phase A */
    f16DutyCycle = MLIB_Mul_F16((PWM_RELOAD/2), sUABCtemp.f16A);

    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, UH_PWM_OUTPIN, UL_PWM_OUTPIN, f16DutyCycle, PWM_RELOAD - f16DutyCycle);
     /* phase B */  
    f16DutyCycle = MLIB_Mul_F16((PWM_RELOAD/2), sUABCtemp.f16B);

    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, VH_PWM_OUTPIN, VL_PWM_OUTPIN, f16DutyCycle, PWM_RELOAD - f16DutyCycle);
     /* phase C*/
    f16DutyCycle = MLIB_Mul_F16((PWM_RELOAD/2), sUABCtemp.f16C);


    pwm_update_raw_cmp_central_aligned(BOARD_PWM_TYPE, WH_PWM_OUTPIN, WL_PWM_OUTPIN, f16DutyCycle, PWM_RELOAD - f16DutyCycle); 
    #endif

    return (s_statusPass);
}

/*!
 * @brief Function enables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmPwm3PhOutEn(mcdrv_ftm_pwm3ph_t *this)
{
    s_statusPass = TRUE;
  
    enable_all_pwm_output() ;
    return (s_statusPass);
}

/*!
 * @brief Function disables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmPwm3PhOutDis(mcdrv_ftm_pwm3ph_t *this)
{
    s_statusPass = TRUE;
    disable_all_pwm_output() ;
  
    return (s_statusPass);
}

/*!
 * @brief Function return actual value of over current flag
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmPwm3PhFltGet(mcdrv_ftm_pwm3ph_t *this) //报警接口
{
#if 0
    /* Read fixed-value over-current flag */
    s_statusPass = this->pui32PwmBase->FMS & (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum);
    /* Clear fault flags */
    this->pui32PwmBase->FMS &= ~FTM_FMS_FAULTF_MASK;

    return ((s_statusPass > 0));
#else
    return over_current_flag;
#endif
}

