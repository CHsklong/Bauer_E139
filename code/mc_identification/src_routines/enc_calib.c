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

#include "enc_calib.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Pp measurement routine
 *
 * @param *sEncCalibFcn   input structure of type #mid_get_pp_t for passing
 *                      all necessary parameters.
 *
 * @return None
 */
uint32_t cur_pos ;
float pre_pos ;
float avg;
extern uint16_t g_cnt ;
void MID_calibEnc(mid_calib_enc_t* sEncCalibFcn)
{
   float pos;
    /* Initialisation */
    if(sEncCalibFcn->ui16Active == 0)
    {
        sEncCalibFcn->ui16Active        = 1;
        sEncCalibFcn->ui16Step          = 0;
        sEncCalibFcn->ui32LoopCounter = MID_TIME_1200MS;
        *sEncCalibFcn->pf16PosEl = 0;
        sEncCalibFcn->clib_dir = 0 ;
        sEncCalibFcn->fAngleCirclePulse = (M1_POSPE_ENC_PULSES<<2)/((float)sEncCalibFcn->ui8Pp) ;
        sEncCalibFcn->ui16Microsteps = 64 ;
    }
    else if(sEncCalibFcn->ui16Active == 2)            //反向标定
    {
        sEncCalibFcn->ui16Active        = 3;
        sEncCalibFcn->ui16Step          = 0;
        sEncCalibFcn->ui32LoopCounter = MID_TIME_1200MS;
        *sEncCalibFcn->pf16PosEl = 0;
        sEncCalibFcn->clib_dir = 1 ;
        sEncCalibFcn->fAngleCirclePulse = (M1_POSPE_ENC_PULSES<<2)/((float)sEncCalibFcn->ui8Pp) ;
        sEncCalibFcn->ui16Microsteps = 64 ;
    }
    /* Set Id required */
    *(sEncCalibFcn->pfltIdReq) = sEncCalibFcn->fltIdReqOpenLoop;

    if(g_sMID.sMIDEnc.clib_ext ==0) //
    {
          sEncCalibFcn->ui32LoopCounter--;

          /* Escape waiting in steady position after 300 ms */
         if(sEncCalibFcn->ui32LoopCounter == 0)
         {
       //       uint32_t temp, min = (M1_POSPE_ENC_PULSES<<2), max = 0;
              avg = 0.0f;
        // get align position
       // if(sEncCalibFcn->fltPosEl == 0.0f)
             {
              if(g_sMID.sMIDEnc.encType == ENC_TYPE_ABS)
              {

	          /*  for(int32_t i = 0; i < 10; i ++)
	            {
	                avg = avg/2.0f;
	                M1_MCDRV_ENC_GET_ABS();
	                if(min > temp)
	                    min = temp;
	                if(max < temp)
	                    max = temp;
	                avg += temp/2.0f;
	            }*/

                    avg = MCDRV_FtmEncGetAbsCnt(&g_sM1Enc) ;

                    #if 0
	            sEncCalibFcn->pui16AlignPos[sEncCalibFcn->ui8AlignPosIdx] = avg;//(min+max) / 2;
	            sEncCalibFcn->ui8AlignPosIdx ++;
	            sEncCalibFcn->ui8AlignPosIdx %= sEncCalibFcn->ui8Pp;

	            //for test
	            sEncCalibFcn->pui16AlignPos[sEncCalibFcn->ui8Pp] = min;
	            sEncCalibFcn->pui16AlignPos[sEncCalibFcn->ui8Pp+1] = max;
                    #endif
              }
               else if(g_sMID.sMIDEnc.encType == ENC_TYPE_UVW)
               {
                  if(*sEncCalibFcn->ps32ZeroCnt != 0)
                  {
                    int32_t encVal;

                    M1_MCDRV_ENC_GET_CNT(encVal);
                    *sEncCalibFcn->ps32ZeroOffset = encVal - *sEncCalibFcn->ps32ZeroCnt;
                    *sEncCalibFcn->ps32ZeroCnt = 0;
                    sEncCalibFcn->ui16Active = FALSE;
                  }
               }

            }
      pre_pos = sEncCalibFcn->ui16Step*360.0f/sEncCalibFcn->ui16Microsteps;               //计算出当前理论上的电角度
      cur_pos = ((uint32_t)(pre_pos*sEncCalibFcn->fAngleCirclePulse)) / 360.0f ;          //换算成理论上的编码器脉冲数
      cur_pos = cur_pos %(M1_POSPE_ENC_PULSES<<2) ; //
      if(sEncCalibFcn->clib_dir==1)
        cur_pos =(M1_POSPE_ENC_PULSES<<2) - cur_pos ;
      sEncCalibFcn->pui16AlignPos[sEncCalibFcn->clib_dir][(((uint16_t)avg)>>8)&0x3f] = cur_pos - avg ;
     // sEncCalibFcn->pui16AlignPos[sEncCalibFcn->ui16Step&0x3f] = cur_pos - avg ;
      sEncCalibFcn->ui16Step ++;                                              //进行下一步，步数增加
      pos = sEncCalibFcn->ui16Step*360.0f/sEncCalibFcn->ui16Microsteps;      // 计算出下一步的目标电角度
      sEncCalibFcn->fltPosEl = pos - ((uint16_t)((pos+180)/360))*360.0f;      //把角度限制在 -180 到 180 之间
      if(sEncCalibFcn->clib_dir==1)
        sEncCalibFcn->fltPosEl = - sEncCalibFcn->fltPosEl ;
      *sEncCalibFcn->pf16PosEl = MLIB_ConvSc_F16ff(sEncCalibFcn->fltPosEl, 180.0f);   //赋值，将位置发给foc拖动电机转动
      if(sEncCalibFcn->fltPosEl == 0.0f)
            sEncCalibFcn->ui32LoopCounter = MID_TIME_300MS;
      else
            sEncCalibFcn->ui32LoopCounter = MID_TIME_50MS;
      if(sEncCalibFcn->ui16Step >= sEncCalibFcn->ui16Microsteps*(sEncCalibFcn->ui8Pp+2))
       {
            if(sEncCalibFcn->ui16Active ==1)
            {
                sEncCalibFcn->ui16Active = 2;
            }
            else if(sEncCalibFcn->ui16Active ==3)
            {
                sEncCalibFcn->ui16Active = 0;
            }

       }
    }
  }
}

