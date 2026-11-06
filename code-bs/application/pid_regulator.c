/**
 ******************************************************************************
 * @file    pid_regulator.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the PID regulator component of the Motor Control SDK:
 *
 *           * proportional, integral and derivative computation funcions
 *           * read and write gain functions
 *
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "pid_regulator.h"
#include <stdio.h>
#include <math.h>

int32_t pid_p_speed,pid_i_speed,pid_d_speed;

/** @addtogroup MCSDK
 * @{
 */

/**
 * @defgroup PIDRegulator PID Regulator
 * @brief PID regulator component of the Motor Control SDK
 *
 * The PID regulator component implements the following control function:
 *
 * @f[
 * u(t) = K_{p} e(t) + K_{i} \int_0^t e(\tau) \,d\tau + K_{d} \frac{de(t)}{dt}
 * @f]
 *
 * with the proportional, integral and derivative gains expressed as rational numbers, with a gain and a divisor parameter :
 *
 * @f[
 * K_{p} = \frac{K_{pg}}{K_{pd}}
 * @f]
 * @f[
 * K_{i} = \frac{K_{ig}}{K_{id}}
 * @f]
 * @f[
 * K_{d} = \frac{K_{dg}}{K_{dd}}
 * @f]
 *
 *  Each of the gain and divisor parameters, @f$K_{{p}g}@f$, @f$K_{{i}g}@f$, @f$K_{{d}g}@f$, @f$K_{{p}d}@f$,
 * @f$K_{id}@f$, @f$K_{dd}@f$, can be set independently. via the PID_SetKP(), PID_SetKPDivisorPOW2(), PID_SetKI(),
 * PID_SetKIDivisorPOW2(), PID_SetKD()
 *
 * @{
 */

/**
 * @brief  It initializes the handle
 * @param  pHandle: handler of the current instance of the PID component
 * @retval None
 */
void PID_HandleInit( PID_Handle_t * pHandle )
{
  pHandle->hKpGain =  pHandle->hDefKpGain;
  pHandle->hKiGain =  pHandle->hDefKiGain;
  pHandle->hKdGain =  pHandle->hDefKdGain;
  pHandle->hLowerOutputLimit = pHandle->hLowerOutputLimit ;
  pHandle->hUpperOutputLimit = pHandle->hUpperOutputLimit ;
  pHandle->wIntegralTerm = 0x00000000UL;
  pHandle->wPrevProcessVarError = 0x00000000UL;
}

/**
 * @brief  It updates the Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKpGain: new Kp gain
 * @retval None
 */
void PID_SetKP( PID_Handle_t * pHandle, int16_t hKpGain )
{
  pHandle->hKpGain = hKpGain;
}

/**
 * @brief  It updates the Ki gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKiGain: new Ki gain
 * @retval None
 */
void PID_SetKI( PID_Handle_t * pHandle, int16_t hKiGain )
{
  pHandle->hKiGain = hKiGain;
}

/**
 * @brief  It returns the Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kp gain
 */
int16_t PID_GetKP( PID_Handle_t * pHandle )
{
  return ( pHandle->hKpGain );
}

/**
 * @brief  It returns the Ki gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Ki gain
 */
int16_t PID_GetKI( PID_Handle_t * pHandle )
{
  return ( pHandle->hKiGain );
}

/**
 * @brief  It returns the Default Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval default Kp gain
 */
int16_t PID_GetDefaultKP( PID_Handle_t * pHandle )
{
  return ( pHandle->hDefKpGain );
}

/**
 * @brief  It returns the Default Ki gain of the passed PI object
 * @param  pHandle: handler of the current instance of the PID component
 * @retval default Ki gain
 */
int16_t PID_GetDefaultKI( PID_Handle_t * pHandle )
{
  return ( pHandle->hDefKiGain );
}

/**
 * @brief  It set a new value into the PI integral term
 * pHandle: handler of the current instance of the PID component
 * @param  wIntegralTermValue: new integral term value
 * @retval None
 */
void PID_SetIntegralTerm( PID_Handle_t * pHandle, int32_t wIntegralTermValue )
{
  pHandle->wIntegralTerm = wIntegralTermValue;

  return;
}

/**
 * @brief  It returns the Kp gain divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kp gain divisor
 */
uint16_t PID_GetKPDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKpDivisor );
}

/**
 * @brief  It updates the Kp divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKpDivisorPOW2: new Kp divisor expressed as power of 2
 * @retval None
 */
void PID_SetKPDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKpDivisorPOW2 )
{
  pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
  pHandle->hKpDivisor = ( ( uint16_t )( 1u ) << hKpDivisorPOW2 );
}

/**
 * @brief  It returns the Ki gain divisor of the passed PI object
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Ki gain divisor
 */
uint16_t PID_GetKIDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKiDivisor );
}

/**
 * @brief  It updates the Ki divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKiDivisorPOW2: new Ki divisor expressed as power of 2
 * @retval None
 */
void PID_SetKIDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKiDivisorPOW2 )
{
  int32_t wKiDiv = ( ( int32_t )( 1u ) << hKiDivisorPOW2 );
  pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
  pHandle->hKiDivisor = ( uint16_t )( wKiDiv );
  PID_SetUpperIntegralTermLimit( pHandle, ( int32_t )INT16_MAX * wKiDiv );
  PID_SetLowerIntegralTermLimit( pHandle, ( int32_t ) - INT16_MAX * wKiDiv );
}

/**
 * @brief  It set a new value for lower integral term limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wLowerLimit: new lower integral term limit value
 * @retval None
 */
void PID_SetLowerIntegralTermLimit( PID_Handle_t * pHandle, int32_t wLowerLimit )
{
  pHandle->wLowerIntegralLimit = wLowerLimit;
}

/**
 * @brief  It set a new value for upper integral term limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wUpperLimit: new upper integral term limit value
 * @retval None
 */
void PID_SetUpperIntegralTermLimit( PID_Handle_t * pHandle, int32_t wUpperLimit )
{
  pHandle->wUpperIntegralLimit = wUpperLimit;
}

/**
 * @brief  It set a new value for lower output limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hLowerLimit: new lower output limit value
 * @retval None
 */
void PID_SetLowerOutputLimit( PID_Handle_t * pHandle, int16_t hLowerLimit )
{
  pHandle->hLowerOutputLimit = hLowerLimit;
}

/**
 * @brief  It set a new value for upper output limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hUpperLimit: new upper output limit value
 * @retval None
 */
void PID_SetUpperOutputLimit( PID_Handle_t * pHandle, int16_t hUpperLimit )
{
  pHandle->hUpperOutputLimit = hUpperLimit;
}

/**
 * @brief  It set a new value into the PID Previous error variable required to
 *         compute derivative term
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wPrevProcessVarError: New previous error variable
 * @retval None
 */
void PID_SetPrevError( PID_Handle_t * pHandle, int32_t wPrevProcessVarError )
{
  pHandle->wPrevProcessVarError = wPrevProcessVarError;
  return;
}

/**
 * @brief  It updates the Kd gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKdGain: new Kd gain
 * @retval None
 */
void PID_SetKD( PID_Handle_t * pHandle, int16_t hKdGain )
{
  pHandle->hKdGain = hKdGain;
}

/**
 * @brief  It returns the Kd gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kd gain
 */
int16_t PID_GetKD( PID_Handle_t * pHandle )
{
  return pHandle->hKdGain;
}

/**
 * @brief  It returns the Kd gain divisor of the PID object passed
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kd gain divisor
 */
uint16_t PID_GetKDDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKdDivisor );
}
void PID_CLEAR( PID_Handle_t * pHandle )
{
    pHandle->wIntegralTerm = 0 ; 
}

/**
 * @brief Sets @f$K_{dd}@f$, the derivative divisor parameter of the PID component
 *
 * @param pHandle handle on the instance of the PID component to update
 * @param hKdDivisorPOW2
 */
void PID_SetKDDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKdDivisorPOW2 )
{
  pHandle->hKdDivisorPOW2 = hKdDivisorPOW2;
  pHandle->hKdDivisor = ( ( uint16_t )( 1u ) << hKdDivisorPOW2 );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
 * @brief  This function compute the output of a PI regulator sum of its
 *         proportional and integral terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the reference
 *         value minus the present process variable value
 * @retval computed PI output
 */

int16_t PI_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError )
{
  int32_t wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
  int32_t wDischarge = 0;
  int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
  int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

  /* Proportional term computation*/
  wProportional_Term = pHandle->hKpGain * wProcessVarError;
   
  /* Integral term computation */
  if ( pHandle->hKiGain == 0 )
  {
    pHandle->wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = pHandle->hKiGain * wProcessVarError;
    wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

    if ( wIntegral_sum_temp < 0 )
    {
      if ( pHandle->wIntegralTerm > 0 )
      {
        if ( wIntegral_Term > 0 )
        {
          wIntegral_sum_temp = INT32_MAX;
        }
      }
    }
    else
    {
      if ( pHandle->wIntegralTerm < 0 )
      {
        if ( wIntegral_Term < 0 )
        {
          wIntegral_sum_temp = -INT32_MAX;
        }
      }
    }

    if ( wIntegral_sum_temp > pHandle->wUpperIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
    }
    else if ( wIntegral_sum_temp < pHandle->wLowerIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
    }
    else
    {
      pHandle->wIntegralTerm = wIntegral_sum_temp;
    }
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wOutput_32 = ( wProportional_Term / ( int32_t )pHandle->hKpDivisor ) + ( pHandle->wIntegralTerm /
               ( int32_t )pHandle->hKiDivisor );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/
 // wOutput_32 = ( wProportional_Term >> pHandle->hKpDivisorPOW2 ) + ( pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2 );
 #if 1
 	pid_p_speed =  ( wProportional_Term / ( int32_t )pHandle->hKpDivisor ) ;
	pid_i_speed = ( pHandle->wIntegralTerm /( int32_t )pHandle->hKiDivisor ) ;
	wOutput_32 =pid_p_speed + pid_i_speed;
#else
	wOutput_32 = ( wProportional_Term / ( int32_t )pHandle->hKpDivisor ) + ( pHandle->wIntegralTerm /
               ( int32_t )pHandle->hKiDivisor );
#endif
#endif

  if ( wOutput_32 > hUpperOutputLimit )
  {

    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;
  }
  else if ( wOutput_32 < hLowerOutputLimit )
  {

    wDischarge = hLowerOutputLimit - wOutput_32;
    wOutput_32 = hLowerOutputLimit;
  }
  else { /* Nothing to do here */ }

  pHandle->wIntegralTerm += wDischarge;
  pHandle->hPidOut = ( ( int16_t )( wOutput_32 ) );
  return ( ( int16_t )( wOutput_32 ) );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
 * @brief  This function compute the output of a PID regulator sum of its
 *         proportional, integral and derivative terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the
 *         reference value minus the present process variable value
 * @retval PID computed output
 */

int16_t PID_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError )
{
  int32_t wDifferential_Term;
  int32_t wDeltaError;
  int32_t wTemp_output;

  if ( pHandle->hKdGain != 0 ) /* derivative terms not used */
  {
    wDeltaError = wProcessVarError - pHandle->wPrevProcessVarError;
    wDifferential_Term = pHandle->hKdGain * wDeltaError;
	wDifferential_Term /= ( int32_t )pHandle->hKdDivisor;
#if 0
#ifdef FULL_MISRA_C_COMPLIANCY
    wDifferential_Term /= ( int32_t )pHandle->hKdDivisor;
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right)
    is used by the compiler to perform the shifts (instead of LSR
    logical shift right)*/
    wDifferential_Term >>= pHandle->hKdDivisorPOW2;
#endif
#endif
    pHandle->wPrevProcessVarError = wProcessVarError;

	
	pid_d_speed = wDifferential_Term ;
	//wDifferential_Term =0 ;
    wTemp_output = PI_Controller( pHandle, wProcessVarError ) + wDifferential_Term;

    if ( wTemp_output > pHandle->hUpperOutputLimit )
    {
      wTemp_output = pHandle->hUpperOutputLimit;
    }
    else if ( wTemp_output < pHandle->hLowerOutputLimit )
    {
      wTemp_output = pHandle->hLowerOutputLimit;
    }
    else
    {}
  }
  else
  {
    wTemp_output = PI_Controller( pHandle, wProcessVarError );
  }
  return ( ( int16_t ) wTemp_output );
}




// 三角形隶属度函数
float membership(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x > a && x <= b) return (x - a) / (b - a);
    if (x > b && x < c) return (c - x) / (c - b);
    return 0.0f;
}

// ====================== 模糊规则库 ======================
// 模糊规则表 (5x5矩阵)
#if 0
const FuzzySet Kp_rules[5][5] = {
    // de\e |  NB   NS   ZO   PS   PB
    /* NB */ { PB,  PB,  PS,  PS,  ZO },
    /* NS */ { PB,  PS,  PS,  ZO,  NS },
    /* ZO */ { PS,  PS,  ZO,  NS,  NS },
    /* PS */ { PS,  ZO,  NS,  NS,  NB },
    /* PB */ { ZO,  NS,  NS,  NB,  NB }
};
#else
const FuzzySet Kp_rules[5][5] = {
    // de\e |  NB   NS   ZO   PS   PB
    /* NB */ { ZO,  ZO,  ZO,  ZO,  ZO },
    /* NS */ { ZO,  ZO,  ZO,  ZO,  ZO },
    /* ZO */ { ZO,  ZO,  ZO,  ZO,  ZO },
    /* PS */ { PS,  PS,  PS,  PS,  PS },
    /* PB */ { PB,  PB,  PB,  PB,  PB }
};

#endif
const FuzzySet Ki_rules[5][5] = {
    // de\e |  NB   NS   ZO   PS   PB
    /* NB */ { PB,  PB,  PB,  PS,  ZO },
    /* NS */ { PB,  PB,  PB,  PS,  ZO },
    /* ZO */ { PB,  PB,  PB,  PS,  ZO },
    /* PS */ { PB,  PB,  PB,  PS,  ZO },
    /* PB */ { PB,  PB,  PB,  PS,  ZO }
};

const FuzzySet Kd_rules[5][5] = {
    // de\e |  NB   NS   ZO   PS   PB
    /* NB */ { PS,  ZO,  ZO,  ZO,  PB },
    /* NS */ { PS,  PS,  ZO,  ZO,  ZO },
    /* ZO */ { PB,  PS,  ZO,  PS,  PB },
    /* PS */ { ZO,  ZO,  ZO,  PS,  PS },
    /* PB */ { PB,  ZO,  ZO,  ZO,  PS }
};

// ====================== 模糊推理引擎 ======================
void fuzzy_inference(float e, float de, FuzzyPID *pid, 
                     float *dKp, float *dKi, float *dKd) {
    // 标准化输入 (映射到[0,4]区间)
    float norm_e = 4.0f * (e - pid->e_min) / (pid->e_max - pid->e_min);
    float norm_de = 4.0f * (de - pid->de_min) / (pid->de_max - pid->de_min);
    
    // 初始化隶属度矩阵
    float mem_e[5] = {0}; // [NB, NS, ZO, PS, PB]
    float mem_de[5] = {0};
    
    // 计算e的隶属度
    mem_e[NB] = membership(norm_e, -1.0f, 0.0f, 1.0f);
    mem_e[NS] = membership(norm_e, 0.0f, 1.0f, 2.0f);
    mem_e[ZO] = membership(norm_e, 1.0f, 2.0f, 3.0f);
    mem_e[PS] = membership(norm_e, 2.0f, 3.0f, 4.0f);
    mem_e[PB] = membership(norm_e, 3.0f, 4.0f, 5.0f);
    
    // 计算de的隶属度
    mem_de[NB] = membership(norm_de, -1.0f, 0.0f, 1.0f);
    mem_de[NS] = membership(norm_de, 0.0f, 1.0f, 2.0f);
    mem_de[ZO] = membership(norm_de, 1.0f, 2.0f, 3.0f);
    mem_de[PS] = membership(norm_de, 2.0f, 3.0f, 4.0f);
    mem_de[PB] = membership(norm_de, 3.0f, 4.0f, 5.0f);
    
    // 模糊推理 - 使用Mamdani最小-最大法
    float dKp_set[5] = {0}; // 输出隶属度
    float dKi_set[5] = {0};
    float dKd_set[5] = {0};
    
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 5; j++) {
            float fire_strength =  fmin(mem_e[i], mem_de[j]);
            
            // Kp规则
            FuzzySet kp_out = Kp_rules[j][i];
            if(fire_strength > dKp_set[kp_out]) 
                dKp_set[kp_out] = fire_strength;
                
            // Ki规则
            FuzzySet ki_out = Ki_rules[j][i];
            if(fire_strength > dKi_set[ki_out]) 
                dKi_set[ki_out] = fire_strength;
                
            // Kd规则
            FuzzySet kd_out = Kd_rules[j][i];
            if(fire_strength > dKd_set[kd_out]) 
                dKd_set[kd_out] = fire_strength;
        }
    }
    
    // 解模糊 - 重心法 (加权平均)
    float sum_kp = 0.0f, weight_kp = 0.0f;
    float sum_ki = 0.0f, weight_ki = 0.0f;
    float sum_kd = 0.0f, weight_kd = 0.0f;
    
    // 输出值对应表 [-2,-1,0,1,2] 映射到实际范围
    const float output_values[5] = {
        -1.0f, -0.5f, 0.0f, 0.5f, 1.0f
    };
    
    for(int i = 0; i < 5; i++) {
        // Kp解模糊
        float kp_val = output_values[i];
        sum_kp += dKp_set[i] * kp_val;
        weight_kp += dKp_set[i];
        
        // Ki解模糊
        float ki_val = output_values[i];
        sum_ki += dKi_set[i] * ki_val;
        weight_ki += dKi_set[i];
        
        // Kd解模糊
        float kd_val = output_values[i];
        sum_kd += dKd_set[i] * kd_val;
        weight_kd += dKd_set[i];
    }
    
    // 计算最终调整量 (防除零)
    *dKp = (weight_kp > 0.001f) ? sum_kp / weight_kp : 0.0f;
    *dKi = (weight_ki > 0.001f) ? sum_ki / weight_ki : 0.0f;
    *dKd = (weight_kd > 0.001f) ? sum_kd / weight_kd : 0.0f;
    
    // 映射到实际调整范围
    *dKp = (*dKp) * (pid->dKp_max - pid->dKp_min)/2.0f;
    *dKi = (*dKi) * (pid->dKi_max - pid->dKi_min)/2.0f;
    *dKd = (*dKd) * (pid->dKd_max - pid->dKd_min)/2.0f;
}
float g_scale = 1.0f ;
// ====================== 模糊PID控制器 ======================
float fuzzy_pid_update(float setpoint, float actual, FuzzyPID *pid) {
    // 计算误差和误差变化率
    float error = setpoint - actual;
    float de = error - pid->prev_error;
    pid->de = de ;
	pid->error = error ;
    // 保存当前误差
    pid->prev_error = error;
    
    // 模糊推理获取参数调整量
    float dKp, dKi, dKd;
    fuzzy_inference(error, de, pid, &dKp, &dKi, &dKd);
    
    // 更新PID参数
    pid->Kp = pid->Kp0 + dKp;
    pid->Ki = pid->Ki0 + dKi;
   /* pid->Kd = pid->Kd0 + dKd;*/
   // pid->Kp = pid->Kp0 ;
	//pid->Ki = pid->Ki0;
    pid->Kd = pid->Kd0 ;
    // 抗积分饱和处理
    pid->integral += pid->Ki * error;
    //if(pid->integral > pid->wUpperIntegralLimit) pid->integral = pid->wUpperIntegralLimit;
    //if(pid->integral > pid->Kp * error*g_scale) pid->integral = pid->Kp * error *g_scale ;
	if(pid->integral > pid->wUpperIntegralLimit) pid->integral = pid->wUpperIntegralLimit;
    if(pid->integral < pid->wLowerIntegralLimit) pid->integral = pid->wLowerIntegralLimit;
    pid_p_speed = pid->Kp * error/65536 ;
	pid_i_speed = pid->integral /65536 ;
    // 计算PID输出
    pid->output = pid_p_speed + 
                 pid_i_speed + 
                 pid->Kd * de;
   	
    if(pid->output >pid->hUpperOutputLimit) pid->output = pid->hUpperOutputLimit ;
	if(pid->output <pid->hLowerOutputLimit) pid->output = pid->hLowerOutputLimit ;
    return pid->output;
}

// ====================== 初始化函数 ======================
void fuzzy_pid_init(FuzzyPID *pid, float kp0, float ki0, float kd0, 
                   float e_min, float e_max, 
                   float de_min, float de_max,
                   float dKp_min, float dKp_max,
                   float dKi_min, float dKi_max,
                   float dKd_min, float dKd_max){
    pid->Kp0 = kp0;
    pid->Ki0 = ki0;
    pid->Kd0 = kd0;
    pid->Kp = kp0;
    pid->Ki = ki0;
    pid->Kd = kd0;
    
    pid->e_min = e_min;
    pid->e_max = e_max;
    pid->de_min = de_min;
    pid->de_max = de_max;
    pid->dKp_min = dKp_min;
    pid->dKp_max = dKp_max;
    pid->dKi_min = dKi_min;
    pid->dKi_max = dKi_max;
	pid->dKd_min = dKd_min;
    pid->dKd_max = dKd_max;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}


/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
