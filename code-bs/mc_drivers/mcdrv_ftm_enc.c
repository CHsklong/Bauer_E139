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

#include "mcdrv_ftm_enc.h"
#include "m1_pmsm_appconfig.h"
#include "mcdrv.h"
#include "abs_motor.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
static bool_t    statusPass;
uint16_t g_cnt =0 ;
uint16_t g_cnt1 =0 ;

int16_t old_cnt =0 ;
int16_t check_cnt =0 ;
#define MAX_PERIOD_ENC  200 
uint8_t test_flag =0 ;
/*******************************************************************************
 * Code   incoder: 12bit
 ******************************************************************************/
static uint32_t calib_EncCnt(mcdrv_ftm_enc_t *this, uint32_t cnt)
{
     
    
    if(this->encType == ENC_TYPE_ABS)
    {
        uint16_t  curAngle ;
        uint16_t high6Bit, low8Bit;
        
        high6Bit = (cnt>>8)&0x3f;
        low8Bit = cnt&0xff;
        g_cnt = cnt &0x3fff  ;
	//curAngle = g_cnt ;		  
        curAngle = ((cnt + this->encAlignData.sAbcAlignData.s16Zero + this->encAlignData.sAbcAlignData.s16AlignPos[0][high6Bit] + this->encAlignData.sAbcAlignData.s16AlignPosDista[high6Bit]*low8Bit/256)&0x3fff);
     //   curAngle = ((cnt + this->encAlignData.sAbcAlignData.s16Zero + this->encAlignData.sAbcAlignData.s16AlignPos[0] )&0x3fff);
     //   curAngle = ((g_cnt + this->encAlignData.sAbcAlignData.s16Zero + 10984 )&0x3fff);
       // g_cnt1 = ((cnt + this->encAlignData.sAbcAlignData.s16AlignPos[high6Bit] + this->encAlignData.sAbcAlignData.s16AlignPosDista[high6Bit]*low8Bit/256)&0x3fff);    
        g_cnt1 = curAngle ;
        return curAngle ;
    }
    return cnt;
}
uint16_t get_clib_encode(uint8_t type)
{
    if(type==0)
      return g_cnt ;
    else
      return g_cnt1 ;
	  
}
uint32_t MCDRV_FtmEncGetCnt(mcdrv_ftm_enc_t *this)
{
#if 0
    if(this->i8EncDir > 0)
        return ((this->pui32FtmBase->CNT & FTM_CNT_COUNT_MASK) >> FTM_CNT_COUNT_SHIFT);

    return (this->pui32FtmBase->MOD - ((this->pui32FtmBase->CNT & FTM_CNT_COUNT_MASK) >> FTM_CNT_COUNT_SHIFT) + 1);
#endif
    return 0 ;
}

uint32_t MCDRV_FtmEncGetAbsCnt(mcdrv_ftm_enc_t *this)
{
   
    uint32_t cnt ;
    M1_MCDRV_ENC_GET_ABS(cnt);
       
    if(this->i8EncDir > 0)
        return cnt;
    
    return (((M1_POSPE_ENC_PULSES<<2) - 1) - cnt);
}
frac32_t cnt32 =0 ;
frac16_t MCDRV_FtmCheckEnc(frac16_t cnt )
{
    volatile static frac16_t f16OldCnt = 0;
    
    cnt32 = cnt - f16OldCnt ;
    if(cnt32 >65536)
    {
        if(f16OldCnt < 32768 - MAX_PERIOD_ENC) //如果不在边沿变化
        {
            cnt = f16OldCnt ;
        }
        else
        {
            cnt32 = cnt32 -65536 ;
            if(cnt32 > MAX_PERIOD_ENC) //如果编码器的变化大于最大速度下的编码器差值
            {
                  cnt = f16OldCnt ;
            }
        }
    }
    else if(cnt32 < -65536)
    {
        if(f16OldCnt > MAX_PERIOD_ENC)//如果不在边沿变化
        {
            cnt = f16OldCnt ;
        }
        else
        {
            cnt32 = cnt32 +65536 ;
            if(cnt32 < -MAX_PERIOD_ENC)//如果编码器的变化大于最大速度下的编码器差值
            {
                  cnt = f16OldCnt ;
            }
        }
    }
    f16OldCnt = cnt ;
    return cnt ;
}
frac16_t MCDRV_FtmTestEnc(frac16_t cnt)
{
    static uint8_t test_cnt =0 ;
    if(test_flag)
    {
        if(test_cnt >13)
        {
            test_cnt =0 ;
        }
        else
        {
            if(test_cnt <3)
            {
               cnt =cnt + (rand()&0xffff) ; 
            }
            test_cnt ++ ;  
        }
        
    }
    return cnt ;
}
/*!
 * @brief Function returns actual position and speed
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */


 #if 0
bool_t MCDRV_FtmEncGet(mcdrv_ftm_enc_t *this)
{
    volatile static frac16_t f16OldPosMe = 0;
    volatile static frac16_t f16OldPosMe1 = 0;
    uint32_t cnt;
	
    statusPass = TRUE;

    /* Read number of pulses and get mechanical position */
    //cnt = MCDRV_FtmEncGetCnt(this);
    cnt = MCDRV_FtmEncGetAbsCnt(this);
	   
    cnt = calib_EncCnt(this, cnt) ;
  /*  old_cnt = (frac16_t)(MLIB_Mul_F16as(this->a32PosMeGain, 
        (frac16_t)(cnt)));
    old_cnt = MCDRV_FtmTestEnc(old_cnt) ; //经过扰动

    check_cnt = MCDRV_FtmCheckEnc(old_cnt) ; //经过过滤
    this->f16PosMe =  old_cnt ;
    this->f16PosMe1 = check_cnt;
    */
      this->f16PosMe =MCDRV_FtmCheckEnc((frac16_t)(MLIB_Mul_F16as(this->a32PosMeGain, 
        (frac16_t)(cnt)))); 
          
   
    

    if(this->f16PosMe1 - f16OldPosMe1 > 32768) this->i16ZCnt1 --;
    else if(this->f16PosMe1 - f16OldPosMe1 < -32768) this->i16ZCnt1 ++;
        
    /* */
    if(this->f16PosMe - f16OldPosMe > 32768) this->i16ZCnt --;
    else if(this->f16PosMe - f16OldPosMe < -32768) this->i16ZCnt ++;
    
    f16OldPosMe1 = this->f16PosMe1;
    f16OldPosMe = this->f16PosMe;
       
    /* Tracking observer calculation */			
    this->f16PosMeEst = (frac16_t)AMCLIB_TrackObsrv_A32af(this->a32PosErr, &this->sTo);

    /* Calculation of error function for tracking observer */  
    this->a32PosErr  = (acc32_t)MLIB_Sub_F16(this->f16PosMe, this->f16PosMeEst);

    /* Speed estimation by the tracking observer */
    this->fltSpdMeEst = this->sTo.fltSpeed;
 
    /* Store results to user-defined variables */
    //*this->pf16PosElEst = (frac16_t)(this->f16PosMeEst * this->ui16Pp );
    *this->pf16PosElEst = (frac16_t)(this->f16PosMeEst * (*this->pui16Pp));
    
    *this->pfltSpdMeEst = (this->fltSpdMeEst);
        
    return(statusPass);
}
#else
bool_t MCDRV_FtmEncGet(mcdrv_ftm_enc_t *this)
{
    volatile static frac16_t f16OldPosMe = 0;
    uint32_t cnt;
	
    statusPass = TRUE;

    /* Read number of pulses and get mechanical position */
    //cnt = MCDRV_FtmEncGetCnt(this);
    cnt = MCDRV_FtmEncGetAbsCnt(this);
	   
    cnt = calib_EncCnt(this, cnt) ;
	   
    this->f16PosMe =MCDRV_FtmCheckEnc((frac16_t)(MLIB_Mul_F16as(this->a32PosMeGain, 
        (frac16_t)(cnt)))); 
          
	/* */
    if(this->f16PosMe - f16OldPosMe > 32768) this->i16ZCnt --;
    else if(this->f16PosMe - f16OldPosMe < -32768) this->i16ZCnt ++;
    
    f16OldPosMe = this->f16PosMe;
       
    /* Tracking observer calculation */			
    this->f16PosMeEst = (frac16_t)AMCLIB_TrackObsrv_A32af(this->a32PosErr, &this->sTo);

    /* Calculation of error function for tracking observer */  
    this->a32PosErr  = (acc32_t)MLIB_Sub_F16(this->f16PosMe, this->f16PosMeEst);

    /* Speed estimation by the tracking observer */
    this->fltSpdMeEst = this->sTo.fltSpeed;
 
    /* Store results to user-defined variables */
    //*this->pf16PosElEst = (frac16_t)(this->f16PosMeEst * this->ui16Pp );
    *this->pf16PosElEst = (frac16_t)(this->f16PosMeEst * (*this->pui16Pp));
    
    *this->pfltSpdMeEst = (this->fltSpdMeEst);
        
    return(statusPass);
}
#endif
/*!
 * @brief Function clears internal variables and decoder counter
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmEncClear(mcdrv_ftm_enc_t *this)
{
    statusPass = TRUE;
  
    this->f16PosMe      = 0;
    this->f16PosMeEst   = 0;
    this->fltSpdMeEst   = 0;
    
    /* Initilize tracking observer */
    this->sTo.f32Theta  = 0;
    this->sTo.fltSpeed  = 0;
    this->sTo.fltI_1    = 0;
           
    /* Clear FTM counter */
 //   this->pui32FtmBase->CNT = 0x0000;
    
    return(statusPass);
}

/*!
 * @brief Function set mechanical position of quadrature encoder
 *
 * @param this     Pointer to the current object
 *        f16PosMe Mechanical position
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmEncSetPosMe(mcdrv_ftm_enc_t *this, frac16_t f16PosMe)
{
#if 0
    frac16_t f16CntMod;
  
    statusPass = TRUE;
    
    f16CntMod = (frac16_t)((((this->pui32FtmBase->MOD) & FTM_MOD_MOD_MASK) >> FTM_MOD_MOD_SHIFT) >> 1);

    /* Set mechnical position */
    this->f16PosMe = f16PosMe;
    this->sTo.f32Theta = MLIB_Conv_F32s(f16PosMe);

    this->pui32FtmBase->CNT = ((this->pui32FtmBase->CNT) & ~(FTM_CNT_COUNT_MASK))|
                               (FTM_CNT_COUNT((uint16_t)MLIB_Mul_F16(f16PosMe, f16CntMod) + (uint16_t)f16CntMod));    

    return(statusPass);		
#else
    return TRUE ;
#endif
}
