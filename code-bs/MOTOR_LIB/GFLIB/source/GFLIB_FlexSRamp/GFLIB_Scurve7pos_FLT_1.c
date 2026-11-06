#include "GFLIB_Scurve7pos_FLT.h"

/* ---------  (好像没有库函数。。。no stdlib) --------- */
static inline float_t min_f(float_t a, float_t b) { return (a < b) ? a : b; }
static inline float_t max_f(float_t a, float_t b) { return (a > b) ? a : b; }

/* 3段（+J/0/-J）从 v_start 增加 dv 的“加速形状距离”与时间 */
static void distance_and3ts(float_t v_start, float_t dv,
                             float_t Amax, float_t Jmax,
                             float_t *t1, float_t *t2, float_t *t3,
                             float_t *s_total)
{
    if (dv <= 0.0F) {
        *t1 = 0.0F;
        *t2 = 0.0F;
        *t3 = 0.0F;
        *s_total = 0.0F;
        return;
    }

    dv = MLIB_Abs_FLT(dv);

    /* dv_trap = Amax^2 / Jmax */
    const float_t A2      = MLIB_Mul_FLT(Amax, Amax);
    const float_t dv_trap = MLIB_Div_FLT(A2, Jmax);

    if (dv >= dv_trap) {
        /* 梯形加速度：+J(t1)->Amax(t2)->-J(t1) */
        const float_t t1_loc = MLIB_Div_FLT(Amax, Jmax);
        const float_t t2_loc = MLIB_Div_FLT(MLIB_Sub_FLT(dv, dv_trap), Amax);
        const float_t t3_loc = t1_loc;

        *t1 = t1_loc;
        *t2 = t2_loc;
        *t3 = t3_loc;

        /* 预计算幂次 */
        const float_t t1_2 = MLIB_Mul_FLT(t1_loc, t1_loc);
        const float_t t1_3 = MLIB_Mul_FLT(t1_2,  t1_loc);
        const float_t t2_2 = MLIB_Mul_FLT(t2_loc, t2_loc);
        const float_t t3_2 = MLIB_Mul_FLT(t3_loc, t3_loc);
        const float_t t3_3 = MLIB_Mul_FLT(t3_2,  t3_loc);

        /* phase1: s1 = v0*t1 + (1/6)*J*t1^3 ; v1 = v0 + 0.5*J*t1^2 */
        const float_t s1 = MLIB_Add_FLT(
                                MLIB_Mul_FLT(v_start, t1_loc),
                                MLIB_Mul_FLT((1.0F/6.0F), MLIB_Mul_FLT(Jmax, t1_3))
                           );

        const float_t v1 = MLIB_Add_FLT(
                                v_start,
                                MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(Jmax, t1_2))
                           );

        /* phase2: s2 = v1*t2 + 0.5*A*t2^2 ; v2 = v1 + A*t2 */
        const float_t s2 = MLIB_Add_FLT(
                                MLIB_Mul_FLT(v1, t2_loc),
                                MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(Amax, t2_2))
                           );

        const float_t v2 = MLIB_Add_FLT(v1, MLIB_Mul_FLT(Amax, t2_loc));

        /* phase3: s3 = v2*t3 + 0.5*A*t3^2 - (1/6)*J*t3^3 */
        const float_t s3 = MLIB_Sub_FLT(
                                MLIB_Add_FLT(
                                    MLIB_Mul_FLT(v2, t3_loc),
                                    MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(Amax, t3_2))
                                ),
                                MLIB_Mul_FLT((1.0F/6.0F), MLIB_Mul_FLT(Jmax, t3_3))
                           );

        *s_total = MLIB_Add_FLT(MLIB_Add_FLT(s1, s2), s3);
    } else {
        /* 三角加速度：+J(t1)->-J(t1)，峰值加速度 < Amax */
        /* t1 = sqrt(dv / Jmax) */
        const float_t t1_loc = GFLIB_Sqrt_FLT(MLIB_Div_FLT(dv, Jmax));
        const float_t t2_loc = 0.0F;
        const float_t t3_loc = t1_loc;

        *t1 = t1_loc;
        *t2 = t2_loc;
        *t3 = t3_loc;

        const float_t t1_2 = MLIB_Mul_FLT(t1_loc, t1_loc);
        const float_t t1_3 = MLIB_Mul_FLT(t1_2,  t1_loc);
        const float_t t3_2 = t1_2; /* t3==t1 */
        const float_t t3_3 = t1_3;

        /* a_peak = J * t1 */
        const float_t a_peak = MLIB_Mul_FLT(Jmax, t1_loc);

        /* s1 = v0*t1 + (1/6)*J*t1^3 ; v1 = v0 + 0.5*J*t1^2 */
        const float_t s1 = MLIB_Add_FLT(
                                MLIB_Mul_FLT(v_start, t1_loc),
                                MLIB_Mul_FLT((1.0F/6.0F), MLIB_Mul_FLT(Jmax, t1_3))
                           );

        const float_t v1 = MLIB_Add_FLT(
                                v_start,
                                MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(Jmax, t1_2))
                           );

        /* s3 = v1*t3 + 0.5*a_peak*t3^2 - (1/6)*J*t3^3 */
        const float_t s3 = MLIB_Sub_FLT(
                                MLIB_Add_FLT(
                                    MLIB_Mul_FLT(v1, t3_loc),
                                    MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(a_peak, t3_2))
                                ),
                                MLIB_Mul_FLT((1.0F/6.0F), MLIB_Mul_FLT(Jmax, t3_3))
                           );

        *s_total = MLIB_Add_FLT(s1, s3);
    }
}


/* 无巡航：给定 vp，计算 s_acc(v0->vp) + s_dec(vp->vf) - D */
static float_t f_no_cruise(float_t D, float_t v0, float_t vf,
                           float_t vp, float_t Amax, float_t Jmax)
{
    if (vp < max_f(v0, vf)) {
        return 1e6F + (max_f(v0, vf) - vp);
    }

    float_t t1a,t2a,t3a, s_acc;
    distance_and3ts(v0, (vp - v0), Amax, Jmax, &t1a,&t2a,&t3a, &s_acc);

    float_t t1d,t2d,t3d, s_dec_accshape;
    distance_and3ts(vp, (vp - vf), Amax, Jmax, &t1d,&t2d,&t3d, &s_dec_accshape);

    const float_t Tdec  = t1d + t2d + t3d;
    const float_t extra = s_dec_accshape - vp * Tdec;     /*  加速形状额外距离 */
    const float_t s_dec = vp * Tdec - extra;              /*  减速真实距离 */

    return (s_acc + s_dec) - D;
}

void GFLIB_FlexSPos7Init_FLT_FC(float_t fltInitPos, GFLIB_FLEXS_POS7_T_FLT *psParam)
{
    psParam->fltState = fltInitPos;
    psParam->fltVel   = 0.0F;
    psParam->fltAcc   = 0.0F;
    psParam->fltJerk  = 0.0F;

    psParam->fltTarget = fltInitPos;
    psParam->fltStart  = fltInitPos;
    psParam->fltDir    = 1.0F;

    psParam->u16Stage   = 0;
    psParam->fltStageRem= 0.0F;

    psParam->fltT1 = psParam->fltT2 = psParam->fltT3 = 0.0F;
    psParam->fltT4 = psParam->fltT5 = psParam->fltT6 = psParam->fltT7 = 0.0F;
    psParam->fltVpAbs   = 0.0F;
    psParam->fltTimeTotal = 0.0F;
    psParam->fltTimeExec  = 0.0F;
    psParam->u16ReplanCnt = 0;
    psParam->bReachFlag = TRUE;
}

/* 规划：输入目标位置 + 起始速度，终点速度默认 0 */
bool_t GFLIB_FlexSPos7CalcIncr_FLT_FC(float_t fltTarget, float_t fltV0,
                                      GFLIB_FLEXS_POS7_T_FLT *psParam)
{
    const float_t Ts   = psParam->fltTs;
    const float_t Vmax = psParam->fltVmax;
    const float_t Amax = psParam->fltAmax;
    const float_t Jmax = psParam->fltJmax;

    psParam->fltStart  = psParam->fltState;
    psParam->fltTarget = fltTarget;

    /* 方向与距离 */
    float_t dx = fltTarget - psParam->fltStart;
    if (dx >= 0.0F) {
        psParam->fltDir = 1.0F;
    } else {
        psParam->fltDir = -1.0F;
        dx = -dx;
    }

    /* 起始速度投影到计划方向（加方向） */
    float_t v0 = fltV0 * psParam->fltDir;
    if (v0 < 0.0F) {
        /* 若起始速度反向，简单为 0 */
        v0 = 0.0F;
    }
    const float_t vf = 0.0F;

    psParam->fltV0Abs = v0;

    /* 1 最短刹停距离检查：从 v0->0 的最短距离 <= dx */
    float_t t1s,t2s,t3s, s_stop_accshape;
    distance_and3ts(v0, MLIB_Abs_FLT(v0 - vf), Amax, Jmax, &t1s,&t2s,&t3s, &s_stop_accshape);
    {
        const float_t Tstop  = t1s + t2s + t3s;
        const float_t extra  = s_stop_accshape - v0 * Tstop;
        const float_t s_stop = v0 * Tstop - extra;
        if (dx + 1e-9F < s_stop) {
            /* 不可达（距离太短） */
            psParam->bReachFlag = TRUE;
            //psParam->fltState   = fltTarget;
            psParam->fltVel     = 0.0F;
            psParam->fltAcc     = 0.0F;
            psParam->fltJerk    = 0.0F;
            psParam->fltTimeTotal = 0.0F;
            psParam->fltTimeExec  = 0.0F;
            return FALSE;
        }
    }

    /* 2 无巡航峰值速度 vp_star */
    float_t v_low = max_f(v0, vf);
    float_t v_high = v_low + 0.1F;
    float_t f_low  = f_no_cruise(dx, v0, vf, v_low,  Amax, Jmax);
    float_t f_high = f_no_cruise(dx, v0, vf, v_high, Amax, Jmax);

    /* 扩展上界直到 f_high >= 0 */
    uint16_t it;
    for (it = 0; (f_high < 0.0F) && (v_high < 1e6F); it++) {
        v_high = v_high * 2.0F + 1e-3F;
        f_high = f_no_cruise(dx, v0, vf, v_high, Amax, Jmax);
        if (it > 30) break;
    }

    float_t vp_star = v_high;
    if (f_low > 0.0F) {
        vp_star = v_low;
    } else {
        /* 两端收敛 */
        float_t lo = v_low, hi = v_high;
        for (it = 0; it < 40; it++) {
            float_t mid = 0.5F * (lo + hi);
            float_t fm  = f_no_cruise(dx, v0, vf, mid, Amax, Jmax);
            if (fm >= 0.0F) {
                hi = mid;
            } else {
                lo = mid;
            }
            if (MLIB_Abs_FLT(hi - lo) < 1e-6F) break;
        }
        vp_star = 0.5F * (lo + hi);
    }

    /* 3) 速度上限：若 vp_star > Vmax，则卡住，引入巡航 */
    float_t vp = vp_star;
    float_t t4 = 0.0F;

    if (vp > Vmax) {
        vp = Vmax;

        float_t t1a,t2a,t3a,s_acc;
        distance_and3ts(v0, max_f(0.0F, vp - v0), Amax, Jmax, &t1a,&t2a,&t3a, &s_acc);

        float_t t1d,t2d,t3d,s_dec_accshape;
        distance_and3ts(vp, max_f(0.0F, vp - vf), Amax, Jmax, &t1d,&t2d,&t3d, &s_dec_accshape);

        const float_t Tdec  = t1d + t2d + t3d;
        const float_t extra = s_dec_accshape - vp * Tdec;
        const float_t s_dec = vp * Tdec - extra;

        const float_t s_min = s_acc + s_dec;
        if (dx + 1e-9F < s_min) {
            /* vmax 太低也不可达 */
            psParam->bReachFlag = TRUE;
            psParam->fltState   = fltTarget;
            psParam->fltVel     = 0.0F;
            psParam->fltAcc     = 0.0F;
            psParam->fltJerk    = 0.0F;
            psParam->fltTimeTotal = 0.0F;
            psParam->fltTimeExec  = 0.0F;
            return FALSE;
        }
        t4 = (dx - s_min) / vp;
    }

    /* 4) 根据 v0->vp 与 vp->0 算出 7 段时间 */
    float_t t1a,t2a,t3a, dum;
    float_t t1d,t2d,t3d;
    distance_and3ts(v0, max_f(0.0F, vp - v0), Amax, Jmax, &t1a,&t2a,&t3a, &dum);
    distance_and3ts(vp, max_f(0.0F, vp - vf), Amax, Jmax, &t1d,&t2d,&t3d, &dum);

    psParam->fltVpAbs = vp;

    psParam->fltT1 = t1a;
    psParam->fltT2 = t2a;
    psParam->fltT3 = t3a;
    psParam->fltT4 = t4;
    psParam->fltT5 = t1d;
    psParam->fltT6 = t2d;
    psParam->fltT7 = t3d;

    psParam->fltTimeTotal = psParam->fltT1 + psParam->fltT2 + psParam->fltT3 + psParam->fltT4 +
                            psParam->fltT5 + psParam->fltT6 + psParam->fltT7;
    psParam->fltTimeExec  = 0.0F;

    /* 给下面用 */
    psParam->u16Stage = 0;
    psParam->fltStageRem = psParam->fltT1;
    psParam->bReachFlag  = FALSE;

    /* 初始化内部状态：速度从 v0 开始，加速度从 0 开始 */
    psParam->fltVel  = v0;
    psParam->fltAcc  = 0.0F;
    psParam->fltJerk = +Jmax; /* stage0 默认 +J */

    /* 扫尾初始化 */
    psParam->u16ReplanCnt = 0;
    return TRUE;
}

static float_t stage_time(const GFLIB_FLEXS_POS7_T_FLT *p, uint16_t stage)
{
    switch (stage) {
    default:
    case 0: return p->fltT1;
    case 1: return p->fltT2;
    case 2: return p->fltT3;
    case 3: return p->fltT4;
    case 4: return p->fltT5;
    case 5: return p->fltT6;
    case 6: return p->fltT7;
    }
}

static float_t stage_jerk(const GFLIB_FLEXS_POS7_T_FLT *p, uint16_t stage)
{
    /* 7段 jerk： +J,0,-J,0,-J,0,+J */
    switch (stage) {
    default:
    case 0: return +p->fltJmax;
    case 1: return 0.0F;
    case 2: return -p->fltJmax;
    case 3: return 0.0F;
    case 4: return -p->fltJmax;
    case 5: return 0.0F;
    case 6: return +p->fltJmax;
    }
}

/* 火箭同款推进器：输出位置 ramp */
float_t GFLIB_FlexSPos7_FLT_FC(GFLIB_FLEXS_POS7_T_FLT *psParam)
{
    if (psParam->bReachFlag) {
        return psParam->fltState;
    }

    const float_t Ts   = psParam->fltTs;
    const float_t Amax = psParam->fltAmax;
    const float_t Vmax = psParam->fltVmax;

    /* 当前段 jerk */
    psParam->fltJerk = stage_jerk(psParam, psParam->u16Stage);

    /* 积分：a = a + j*Ts */
    psParam->fltAcc = psParam->fltAcc + psParam->fltJerk * Ts;
    psParam->fltAcc = min_f(max_f(psParam->fltAcc, -Amax), Amax);

    /* v = v + a*Ts */
    psParam->fltVel = psParam->fltVel + psParam->fltAcc * Ts;
    psParam->fltVel = min_f(max_f(psParam->fltVel, 0.0F), Vmax); 

    /* x = x + v*Ts */
    psParam->fltState = psParam->fltState + (psParam->fltDir * psParam->fltVel) * Ts;

    /* 计时推进 */
    psParam->fltTimeExec += Ts;
    psParam->fltStageRem -= Ts;

    /* 段结束：切换到下一段 */
    while (psParam->fltStageRem <= 0.0F) {
            if (psParam->u16Stage >= 6) {
                
                const float_t err = psParam->fltTarget - psParam->fltState;
                const float_t aerr = MLIB_Abs_FLT(err);
                           
                const float_t POS_EPS    = 5.0F;    /* 误差边界 */
            
                if (aerr <= POS_EPS) {
                    psParam->fltState = psParam->fltTarget;
                    psParam->fltVel   = 0.0F;
                    psParam->fltAcc   = 0.0F;
                    psParam->fltJerk  = 0.0F;
                    psParam->bReachFlag = TRUE;
                    return psParam->fltState;
                }          
                /* 自校准 */
                if ((aerr > POS_EPS) && (psParam->u16ReplanCnt < 2U)) {
                    psParam->u16ReplanCnt++;
                
                    (void)GFLIB_FlexSPos7CalcIncr_FLT_FC(psParam->fltTarget,
                                                         0,
                                                         psParam);
                    /* 重新规划 bReachFlag 变 FALSE */
                    return psParam->fltState;
                }           
                psParam->fltVel   = 0.0F;
                psParam->fltAcc   = 0.0F;
                psParam->fltJerk  = 0.0F;
                psParam->bReachFlag = TRUE;
                return psParam->fltState;
            }
        psParam->u16Stage++;
        psParam->fltStageRem += stage_time(psParam, psParam->u16Stage);

        /* 若新段时间为0，继续跳 */
        if (stage_time(psParam, psParam->u16Stage) <= 0.0F) {
            continue;
        } else {
            break;
        }
    }

    /* 到位保护：越界就顶住 */
    if (psParam->fltDir > 0.0F) {
        if (psParam->fltState >= psParam->fltTarget) {
            psParam->fltState = psParam->fltTarget;
            psParam->fltVel   = 0.0F;
            psParam->fltAcc   = 0.0F;
            psParam->fltJerk  = 0.0F;
            psParam->bReachFlag = TRUE;
        }
    } else {
        if (psParam->fltState <= psParam->fltTarget) {
            psParam->fltState = psParam->fltTarget;
            psParam->fltVel   = 0.0F;
            psParam->fltAcc   = 0.0F;
            psParam->fltJerk  = 0.0F;
            psParam->bReachFlag = TRUE;
        }
    }

    return psParam->fltState;
}


