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
    float_t fltAmax;       /* 加速度上限 */
    float_t fltJmax;       /* 加加速度上限 */

    /* 计划表 */
    float_t fltV0Abs;      /* 起始速度绝对值（用于规划） */
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
    uint16_t u16Stage;     /* 0~6 对应 7段 */
    float_t fltStageRem;   /* 当前段剩余时间 */
    bool_t  bReachFlag;    /* 到达标志 */
    uint16_t u16ReplanCnt; /* 扫尾计数 */
} GFLIB_FLEXS_POS7_T_FLT;

/******************************************************************************
* Exported function prototypes
******************************************************************************/

extern void  GFLIB_FlexSPos7Init_FLT_FC(float_t fltInitPos, GFLIB_FLEXS_POS7_T_FLT *psParam);

extern bool_t GFLIB_FlexSPos7CalcIncr_FLT_FC(float_t fltTarget, float_t fltV0,
                                             GFLIB_FLEXS_POS7_T_FLT *psParam);

extern float_t GFLIB_FlexSPos7_FLT_FC(GFLIB_FLEXS_POS7_T_FLT *psParam);

#if defined(__cplusplus)
}
#endif

#endif /* _GFLIB_SCURVE7POS_FLT_H_ */
