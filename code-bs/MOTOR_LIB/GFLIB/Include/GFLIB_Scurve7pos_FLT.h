#ifndef _GFLIB_SCURVE7POS_FLT_H_
#define _GFLIB_SCURVE7POS_FLT_H_

#if defined(__cplusplus)
extern "C" {
#endif

/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_FP.h"
#include "gflib_FP.h"

/******************************************************************************
* Macros
******************************************************************************/
#define GFLIB_FlexSPos7Init_FLT_C(fltInitPos,psParam) \
        GFLIB_FlexSPos7Init_FLT_FC(fltInitPos,psParam)

#define GFLIB_FlexSPos7CalcIncr_FLT_C(fltTarget,fltV0,psParam) \
        GFLIB_FlexSPos7CalcIncr_FLT_FC(fltTarget,fltV0,psParam)

#define GFLIB_FlexSPos7_FLT_C(psParam) \
        GFLIB_FlexSPos7_FLT_FC(psParam)

/******************************************************************************
* Types
******************************************************************************/


typedef struct
{
    float_t vp;
    float_t t4;
    float_t t1a, t2a, t3a;
    float_t t1d, t2d, t3d;
    float_t Ttotal;
} pos7_plan_out_t;

/*! @brief 7-seg position planner structure */
typedef struct
{
    /* 状态 */
    float_t fltState;      /* x(t) 当前位置规划输出 */
    float_t fltVel;        /* v(t) 速度状态 */
    float_t fltAcc;        /* a(t) 加速度状态 */
    float_t fltJerk;       /* j(t) 当前段 jerk */

    /* 命令设置 */
    float_t fltTarget;     /* 目标位置 */
    float_t fltTs;         /* 采样周期 [s] */
    float_t fltVmax;       /* 速度上限 */

    /* 加速组 */
    float_t fltAmaxAcc;       /* 加速度上限 */
    float_t fltJmaxAcc;       /* 加加速度上限 */
    float_t fltAuseAcc;       /* 本次规划加速段使用的加速度 */
    float_t fltJuseAcc;       /* 本次规划加速段使用的加加速度 */

    /* 减速组 */
    float_t fltAmaxDec;    /* 减速段 加速度上限 */
    float_t fltJmaxDec;    /* 减速段 加加速度上限 */
    float_t fltAuseDec;    /* 本次规划减速段使用的加速度 */
    float_t fltJuseDec;    /* 本次规划减速段使用的加速度 */
    /* 计划表 */
    float_t fltV0Abs;      /* 起始速度绝对值（用于规划，沿方向投影后） */
    float_t fltVpAbs;      /* 峰值速度绝对值（规划结果） */
    float_t fltT1;         /* 第 1 段  */
    float_t fltT2;         /* 第 2 段  */
    float_t fltT3;         /* 第 3 段  */
    float_t fltT4;         /* 第 4 段  */
    float_t fltT5;         /* 第 5 段  */
    float_t fltT6;         /* 第 6 段  */
    float_t fltT7;         /* 第 7 段  */
    float_t fltTimeTotal;  /* 计划总时间 */
    float_t fltTimeExec;   /* 已执行时间 */

    /* 内部标志 */
    float_t fltStart;      /* 计划起点位置 */
    float_t fltDir;        /* 方向：+1 或 -1 */
    bool_t  bReachFlag;    /* 到达标志 */

    float_t segTimeBegin[8];      /* 每段起始时间(累加), segTimeBegin[0]=0, segTimeBegin[7]=Ttotal */
    float_t segTime[7];       /* 每段时长 */
    float_t segJ[7];       /* 每段 jerk（沿规划方向，非乘dir） */

    float_t segX0[7];      /* 每段起点位移 x（沿规划方向，>=0） */
    float_t segV0[7];      /* 每段起点速度 v（沿规划方向） */
    float_t segA0[7];      /* 每段起点加速度 a（沿规划方向） */

    /* 约定速度 */
    float_t fltVmaxReq;
    /* 约定时间 */
    float_t fltTreq;
    uint8_t test_numeber;

    /* 输入前馈速度 */
    float_t fltAdvSpd;

} GFLIB_FLEXS_POS7_T_FLT;

/******************************************************************************
* Exported function prototypes
******************************************************************************/
extern void  GFLIB_FlexSPos7Init_FLT_FC(float_t fltInitPos, GFLIB_FLEXS_POS7_T_FLT *psParam);

extern bool_t GFLIB_FlexSPos7CalcIncr_FLT_FC(float_t fltTarget,
                                             float_t fltV0,
                                             GFLIB_FLEXS_POS7_T_FLT *psParam);

extern float_t GFLIB_FlexSPos7_FLT_FC(GFLIB_FLEXS_POS7_T_FLT *psParam);

#if defined(__cplusplus)
}
#endif

#endif /* _GFLIB_SCURVE7POS_FLT_H_ */
