#include "GFLIB_Scurve7pos_FLT.h"

/* ---------  (好像没有库函数。。。no stdlib) --------- */
static inline float_t min_f(float_t a, float_t b) { return (a < b) ? a : b; }
static inline float_t max_f(float_t a, float_t b) { return (a > b) ? a : b; }


#define POS7_SEG_NUM (7u)

static void predict_next_state(float_t j, float_t dt, float_t *x, float_t *v, float_t *a)
{
    /* 解析推进：x,v,a 从段起点推进 dt */
    const float_t dt2 = MLIB_Mul_FLT(dt, dt);
    const float_t dt3 = MLIB_Mul_FLT(dt2, dt);

    /* x = x + v*dt + 0.5*a*dt^2 + (1/6)*j*dt^3 */
    *x = MLIB_Add_FLT(*x,
            MLIB_Add_FLT(
                MLIB_Mul_FLT(*v, dt),
                MLIB_Add_FLT(
                    MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(*a, dt2)),
                    MLIB_Mul_FLT((1.0F/6.0F), MLIB_Mul_FLT(j, dt3))
                )
            )
        );

    /* v = v + a*dt + 0.5*j*dt^2 */
    *v = MLIB_Add_FLT(*v,
            MLIB_Add_FLT(
                MLIB_Mul_FLT(*a, dt),
                MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(j, dt2))
            )
        );

    /* a = a + j*dt */
    *a = MLIB_Add_FLT(*a, MLIB_Mul_FLT(j, dt));
}

static void build_pos7_table(GFLIB_FLEXS_POS7_T_FLT *psParam)
{
    /* 7段 jerk： +J,0,-J,0,-J,0,+J  */
    psParam->segTime[0] = psParam->fltT1; psParam->segJ[0] = +psParam->fltJuseAcc;
    psParam->segTime[1] = psParam->fltT2; psParam->segJ[1] = 0.0F;
    psParam->segTime[2] = psParam->fltT3; psParam->segJ[2] = -psParam->fltJuseAcc;
    psParam->segTime[3] = psParam->fltT4; psParam->segJ[3] = 0.0F;
    psParam->segTime[4] = psParam->fltT5; psParam->segJ[4] = -psParam->fltJuseDec;
    psParam->segTime[5] = psParam->fltT6; psParam->segJ[5] = 0.0F;
    psParam->segTime[6] = psParam->fltT7; psParam->segJ[6] = +psParam->fltJuseDec;

    /* 累加时间边界 */
    psParam->segTimeBegin[0] = 0.0F;
    for (uint16_t i = 0; i < POS7_SEG_NUM; i++) {
        psParam->segTimeBegin[i + 1] = MLIB_Add_FLT(psParam->segTimeBegin[i], psParam->segTime[i]);
    }

    /* 段起点状态：x从0开始，v从v0Abs开始，a从0开始 */
    float_t x = 0.0F;
    float_t v = psParam->fltV0Abs;
    float_t a = 0.0F;

    for (uint16_t i = 0; i < POS7_SEG_NUM; i++) {
        psParam->segX0[i] = x;
        psParam->segV0[i] = v;
        psParam->segA0[i] = a;

        if (psParam->segTime[i] > 0.0F) {
            predict_next_state(psParam->segJ[i], psParam->segTime[i], &x, &v, &a);
        }
    }
}

static uint16_t find_stage(GFLIB_FLEXS_POS7_T_FLT *psParam, float_t t) 
{
    /* t (0, Ttotal) */
    for (uint16_t i = 0; i < POS7_SEG_NUM; i++) {
        if (t < psParam->segTimeBegin[i + 1]) {
            return i;
        }
    }
    return (POS7_SEG_NUM - 1u);
}

static void calc_motion_stage(GFLIB_FLEXS_POS7_T_FLT *psParam, uint16_t st, float_t tau,
                              float_t *x, float_t *v, float_t *a, float_t *j)
{
    const float_t jv = psParam->segJ[st];
    const float_t x0 = psParam->segX0[st];
    const float_t v0 = psParam->segV0[st];
    const float_t a0 = psParam->segA0[st];

    const float_t t2 = MLIB_Mul_FLT(tau, tau);
    const float_t t3 = MLIB_Mul_FLT(t2, tau);

    *j = jv;
    *a = MLIB_Add_FLT(a0, MLIB_Mul_FLT(jv, tau));

    /* v = v0 + a0*t + 0.5*j*t^2 */
    *v = MLIB_Add_FLT(v0,
            MLIB_Add_FLT(
                MLIB_Mul_FLT(a0, tau),
                MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(jv, t2))
            )
        );

    /* x = x0 + v0*t + 0.5*a0*t^2 + (1/6)*j*t^3 */
    *x = MLIB_Add_FLT(x0,
            MLIB_Add_FLT(
                MLIB_Mul_FLT(v0, tau),
                MLIB_Add_FLT(
                    MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(a0, t2)),
                    MLIB_Mul_FLT((1.0F/6.0F), MLIB_Mul_FLT(jv, t3))
                )
            )
        );
}

/* 3段（+J/0/-J）从 v_start 增加 dv 的“加速形状距离”与时间 */
/* 速度空间决定第2，6段 */
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
        const float_t t1_loc = GFLIB_Sqrt_FLT(MLIB_Div_FLT(dv, Jmax));
        const float_t t2_loc = 0.0F;
        const float_t t3_loc = t1_loc;

        *t1 = t1_loc;
        *t2 = t2_loc;
        *t3 = t3_loc;

        const float_t t1_2 = MLIB_Mul_FLT(t1_loc, t1_loc);
        const float_t t1_3 = MLIB_Mul_FLT(t1_2,  t1_loc);
        const float_t t3_2 = t1_2;
        const float_t t3_3 = t1_3;

        const float_t a_peak = MLIB_Mul_FLT(Jmax, t1_loc);

        const float_t s1 = MLIB_Add_FLT(
                                MLIB_Mul_FLT(v_start, t1_loc),
                                MLIB_Mul_FLT((1.0F/6.0F), MLIB_Mul_FLT(Jmax, t1_3))
                           );

        const float_t v1 = MLIB_Add_FLT(
                                v_start,
                                MLIB_Mul_FLT(0.5F, MLIB_Mul_FLT(Jmax, t1_2))
                           );

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
/* 路程空间决定第四段 */
static float_t f_no_cruise(float_t D, float_t v0, float_t vf,
                           float_t vp,
                           float_t Aacc, float_t Jacc,
                           float_t Adec, float_t Jdec)
{
    if (vp < max_f(v0, vf)) {
        return 1e6F + (max_f(v0, vf) - vp);
    }

    float_t t1a,t2a,t3a, s_acc;
    distance_and3ts(v0, (vp - v0), Aacc, Jacc, &t1a,&t2a,&t3a, &s_acc);

    float_t t1d,t2d,t3d, s_dec_accshape;
    distance_and3ts(vp, (vp - vf), Adec, Jdec, &t1d,&t2d,&t3d, &s_dec_accshape);

    const float_t Tdec  = t1d + t2d + t3d;
    const float_t extra = s_dec_accshape - vp * Tdec;     /* 加速形状额外距离 */
    const float_t s_dec = vp * Tdec - extra;              /* 减速真实距离 */

    return (s_acc + s_dec) - D;
}


/* 给定 vp，计算(加速+减速+巡航)总时间；返回 s_min(加减速距离) */
static float_t time_with_vp(float_t D, float_t v0, float_t vf, float_t vp,
                            float_t Aacc, float_t Jacc,
                            float_t Adec, float_t Jdec,
                            float_t *s_min_out)
{

    if (vp+1e-2F < max_f(v0, vf)) {
        if (s_min_out) *s_min_out = 0.0F;
        return 1e6F;
    }

    float_t t1a,t2a,t3a, s_acc;
    distance_and3ts(v0, (vp - v0), Aacc, Jacc, &t1a,&t2a,&t3a, &s_acc);
    const float_t Ta = t1a + t2a + t3a;

    float_t t1d,t2d,t3d, s_dec_accshape;
    distance_and3ts(vp, (vp - vf), Adec, Jdec, &t1d,&t2d,&t3d, &s_dec_accshape);
    const float_t Td = t1d + t2d + t3d;

    /* 加速形状距离 换算成 减速真实距离 */
    const float_t extra = s_dec_accshape - vp * Td;
    const float_t s_dec = vp * Td - extra;

    const float_t s_min = s_acc + s_dec;
    if (s_min_out) *s_min_out = s_min;

    if (D + 1e-9F < s_min) {
        /* 达不到该 vp */
        return 1e6F;
    }

    /* 巡航时间 */
    float_t t4 = 0.0F;
    if (vp > 1e-9F) {
        t4 = (D - s_min) / vp;
    }
    return Ta + Td + t4;
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


    psParam->fltT1 = psParam->fltT2 = psParam->fltT3 = 0.0F;
    psParam->fltT4 = psParam->fltT5 = psParam->fltT6 = psParam->fltT7 = 0.0F;
    psParam->fltVpAbs   = 0.0F;
    psParam->fltTimeTotal = 0.0F;
    psParam->fltTimeExec  = 0.0F; 
    psParam->bReachFlag = TRUE;
    psParam->test_numeber = 0;
    psParam->fltVmaxReq = 8000.0F;   /* <=0 不限速 */
    psParam->fltTreq    = 0.0F;   /* <=0 不约束时间 */

    /* 加速段 */
    psParam->fltAuseAcc = psParam->fltAmaxAcc;
    psParam->fltJuseAcc = psParam->fltJmaxAcc;

    /* 减速段 */
    psParam->fltAuseDec = psParam->fltAmaxDec;
    psParam->fltJuseDec = psParam->fltJmaxDec;

    /* 缓存清零 */
    for (uint16_t i = 0; i < 8u; i++) psParam->segTimeBegin[i] = 0.0F;
    for (uint16_t i = 0; i < 7u; i++) {
        psParam->segTime[i]  = 0.0F;
        psParam->segJ[i]  = 0.0F;
        psParam->segX0[i] = 0.0F;
        psParam->segV0[i] = 0.0F;
        psParam->segA0[i] = 0.0F;
    }
}

 
/* 在给定 Ause/Juse 下做一次规划 （vp_star + Vmax限幅 + 巡航） */
static bool_t pos7_plan_once(float_t dx, float_t v0, float_t vf,
                             float_t Vmax_use,
                             float_t Aacc, float_t Jacc,
                             float_t Adec, float_t Jdec,
                             pos7_plan_out_t *out)
{
    const float_t EPS_V = 1e-6F;

    /* 最短刹停距离：用减速能力 */
    {
        float_t t1s,t2s,t3s, s_stop_accshape;
        distance_and3ts(v0, MLIB_Abs_FLT(v0 - vf), Adec, Jdec, &t1s,&t2s,&t3s, &s_stop_accshape);

        const float_t Tstop  = t1s + t2s + t3s;
        const float_t extra  = s_stop_accshape - v0 * Tstop;
        const float_t s_stop = v0 * Tstop - extra;

        if (dx < s_stop) return FALSE;
    }

    /* 求 vp_star 时：f_no_cruise 用 Aacc/Jacc + Adec/Jdec */
    {
        float_t v_low = max_f(v0, vf);
        if (v_low < EPS_V) v_low = EPS_V;

        float_t v_high = v_low + 0.1F;
        float_t f_low  = f_no_cruise(dx, v0, vf, v_low,  Aacc, Jacc, Adec, Jdec);
        float_t f_high = f_no_cruise(dx, v0, vf, v_high, Aacc, Jacc, Adec, Jdec);

        uint16_t it;
        for (it = 0; (f_high < 0.0F) && (v_high < 1e6F); it++) {
            v_high = v_high * 2.0F + 1e-3F;
            f_high = f_no_cruise(dx, v0, vf, v_high, Aacc, Jacc, Adec, Jdec);
            if (it > 30u) break;
        }

        float_t vp_star = v_high;
        if (f_low <= 0.0F) {
            float_t lo = v_low, hi = v_high;
            for (it = 0; it < 40u; it++) {
                float_t mid = 0.5F * (lo + hi);
                float_t fm  = f_no_cruise(dx, v0, vf, mid, Aacc, Jacc, Adec, Jdec);
                if (fm >= 0.0F) hi = mid; else lo = mid;
                if (MLIB_Abs_FLT(hi - lo) < 1e-6F) break;
            }
            vp_star = 0.5F * (lo + hi);
        } else {
            vp_star = v_low;
        }

        /* Vmax限幅与巡航 */
        float_t vp = vp_star;
        float_t t4 = 0.0F;

        if (vp > Vmax_use) {
            vp = Vmax_use;
            float_t s_min;
            float_t Tplan = time_with_vp(dx, v0, vf, vp, Aacc, Jacc, Adec, Jdec, &s_min);
            if (Tplan > 9e5F) return FALSE;
            if (vp > EPS_V) t4 = (dx - s_min) / vp;
        } else {
            float_t s_min;
            (void)time_with_vp(dx, v0, vf, vp, Aacc, Jacc, Adec, Jdec, &s_min);
            if (vp > EPS_V && (dx > s_min)) t4 = (dx - s_min) / vp;
        }

        /* 分别算加速三段 和 减速三段 */
        {
            float_t dum;
            distance_and3ts(v0, max_f(0.0F, vp - v0), Aacc, Jacc, &out->t1a,&out->t2a,&out->t3a, &dum);
            distance_and3ts(vp, max_f(0.0F, vp - vf), Adec, Jdec, &out->t1d,&out->t2d,&out->t3d, &dum);

            out->vp = vp;
            out->t4 = t4;
            out->Ttotal = out->t1a + out->t2a + out->t3a + t4 + out->t1d + out->t2d + out->t3d;
            return TRUE;
        }
    }
}


static void pos7_clear_cache(GFLIB_FLEXS_POS7_T_FLT *psParam)
{
    psParam->fltT1 = psParam->fltT2 = psParam->fltT3 = 0.0F;
    psParam->fltT4 = psParam->fltT5 = psParam->fltT6 = psParam->fltT7 = 0.0F;
    psParam->fltVpAbs     = 0.0F;
    psParam->fltTimeTotal = 0.0F;
    psParam->fltTimeExec  = 0.0F;

    for (uint16_t i = 0; i < 8u; i++) psParam->segTimeBegin[i] = 0.0F;
    for (uint16_t i = 0; i < 7u; i++) {
        psParam->segTime[i] = 0.0F;
        psParam->segJ[i]    = 0.0F;
        psParam->segX0[i]   = 0.0F;
        psParam->segV0[i]   = 0.0F;
        psParam->segA0[i]   = 0.0F;
    }


    psParam->fltAcc  = 0.0F;
    psParam->fltJerk = 0.0F;
    /* use值恢复默认 */
    psParam->fltAuseAcc    = psParam->fltAmaxAcc;
    psParam->fltJuseAcc    = psParam->fltJmaxAcc;
    psParam->fltAuseDec = psParam->fltAmaxDec;
    psParam->fltJuseDec = psParam->fltJmaxDec;

    psParam->bReachFlag = FALSE;
}

/* psParam->fltTreq：最大允许总时间（<=0 表示不约束）；
   在 Ttotal<=psParam->fltTreq 前提下尽量“变软”(缩小 A/J) */
bool_t GFLIB_FlexSPos7CalcIncr_FLT_FC(float_t fltTarget, float_t fltV0,                                  
                                      GFLIB_FLEXS_POS7_T_FLT *psParam)
{
    float_t EPS_T = 1e-6F;
    float_t EPS_V = 1e-6F;
 
    float_t Vmax_use = psParam->fltVmax;

    /* 最大速度限制：<=0 不限制 */
    if (psParam->fltVmaxReq > 0.0F) {
        Vmax_use = min_f(Vmax_use, psParam->fltVmaxReq);
    }

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

    /* 起始速度投影到计划方向（反向速度简化为0） */
    float_t v0 = fltV0 * psParam->fltDir;
    if (v0 < 0.0F) v0 = 0.0F;
    const float_t vf = 0.0F;

    psParam->fltV0Abs = v0;

    /* 参数保护 */
    if ((psParam->fltAmaxAcc <= 0.0F)
        || (psParam->fltJmaxAcc <= 0.0F)
        || (psParam->fltAmaxDec <= 0.0F)
        || (psParam->fltJmaxDec <= 0.0F)
        || (Vmax_use <= 0.0F) || (psParam->fltTs <= 0.0F)) {
        psParam->bReachFlag = TRUE;
        psParam->fltVel = 0.0F;
        psParam->fltAcc = 0.0F;
        psParam->fltJerk = 0.0F;
        psParam->fltTimeTotal = 0.0F;
        psParam->fltTimeExec = 0.0F;
        psParam->test_numeber = 1;
        return FALSE;
    }

    pos7_clear_cache(psParam);
    /* 先算最快（Amax/Jmax） */
    float_t Aacc_max = psParam->fltAmaxAcc;
    float_t Jacc_max = psParam->fltJmaxAcc;
    float_t Adec_max = psParam->fltAmaxDec;
    float_t Jdec_max = psParam->fltJmaxDec;

    pos7_plan_out_t fast_plan;
    if (!pos7_plan_once(dx, v0, vf, Vmax_use,
                    Aacc_max, Jacc_max,
                    Adec_max, Jdec_max,
                    &fast_plan)) 
    {
        psParam->bReachFlag   = TRUE;
        psParam->fltVel       = 0.0F;
        psParam->fltAcc       = 0.0F;
        psParam->fltJerk      = 0.0F;
        psParam->fltTimeTotal = 0.0F;
        psParam->fltTimeExec  = 0.0F;
        psParam->test_numeber = 2   ;
        return FALSE;
    }

    /* 默认不缩放 */
    float_t Aacc_use_final = Aacc_max;
    float_t Jacc_use_final = Jacc_max;
    float_t Adec_use_final = Adec_max; 
    float_t Jdec_use_final = Jdec_max;
    /* 限定时间fltTreq：变软（最大 k） */
    if (psParam->fltTreq > 0.0F) {

        /* 不可达 */
        if (fast_plan.Ttotal > psParam->fltTreq + EPS_T) {
            psParam->bReachFlag   = TRUE;
            psParam->fltVel       = 0.0F;
            psParam->fltAcc       = 0.0F;
            psParam->fltJerk      = 0.0F;
            psParam->fltTimeTotal = 0.0F;
            psParam->fltTimeExec  = 0.0F;
            psParam->test_numeber = 3   ;
            return FALSE;
        }

        /* k 越大越软：A=Amax/k, J=Jmax/k^2 */
        float_t k_lo = 1.0F;   /* 一定可行（最快<=Treq） */
        float_t k_hi = 2.0F;

        pos7_plan_out_t standard_plan;

        /* 指数扩展 k_hi，直到不可行/超时 */
        for (uint16_t i = 0; i < 20u; i++) {
            float_t Aacc_try = Aacc_max / k_hi;
            float_t Jacc_try = Jacc_max / (k_hi*k_hi);
            float_t Adec_try = Adec_max / k_hi;
            float_t Jdec_try = Jdec_max / (k_hi*k_hi);

            /* 防异常 */
            if (Aacc_try < EPS_V || Jacc_try < EPS_V||Adec_try < EPS_V || Jdec_try < EPS_V) break;

            if (pos7_plan_once(dx, v0, vf, Vmax_use, Aacc_try, Jacc_try, Adec_try, Jdec_try, &standard_plan) &&
                (standard_plan.Ttotal <= psParam->fltTreq + EPS_T)) {
                fast_plan = standard_plan;
                k_lo = k_hi;
                k_hi = k_hi * 2.0F;
                if (k_hi > 1024.0F) break;
            } else {
                break;
            }
        }

        /* 找最大 k */
        for (uint16_t i = 0; i < 30u; i++) {
            float_t k_mid = 0.5F * (k_lo + k_hi);
            float_t Aacc_try  = Aacc_max / k_mid;
            float_t Jacc_try  = Jacc_max / (k_mid * k_mid);
            float_t Adec_try  = Adec_max / k_mid;
            float_t Jdec_try  = Jdec_max / (k_mid * k_mid);

            if (Aacc_try < EPS_V || Jacc_try < EPS_V || Adec_try < EPS_V|| Jdec_try < EPS_V) {
                k_hi = k_mid;
                continue;
            }

            if (pos7_plan_once(dx, v0, vf, Vmax_use, Aacc_try, Jacc_try, Adec_try, Jdec_try, &standard_plan) &&
                (standard_plan.Ttotal <= psParam->fltTreq + EPS_T)) {
                fast_plan = standard_plan;
                k_lo = k_mid; /* 更软 */
            } else {
                k_hi = k_mid;
            }

            if (MLIB_Abs_FLT(k_hi - k_lo) < 1e-4F) break;
        }

        Aacc_use_final = Aacc_max / k_lo;
        Jacc_use_final = Jacc_max / (k_lo * k_lo);
        Adec_use_final = Adec_max / k_lo;
        Jdec_use_final = Jdec_max / (k_lo * k_lo);
    }

    /* 写回实际A/J  */
    psParam->fltAuseAcc    = Aacc_use_final;
    psParam->fltJuseAcc    = Jacc_use_final;
    psParam->fltAuseDec = Adec_use_final;
    psParam->fltJuseDec = Jdec_use_final;

    psParam->fltVpAbs = fast_plan.vp;

    psParam->fltT1 = fast_plan.t1a;
    psParam->fltT2 = fast_plan.t2a;
    psParam->fltT3 = fast_plan.t3a;
    psParam->fltT4 = fast_plan.t4;
    psParam->fltT5 = fast_plan.t1d;
    psParam->fltT6 = fast_plan.t2d;
    psParam->fltT7 = fast_plan.t3d;

    psParam->fltTimeTotal = psParam->fltT1 + psParam->fltT2 + psParam->fltT3 + psParam->fltT4 +
                            psParam->fltT5 + psParam->fltT6 + psParam->fltT7;

    psParam->fltTimeExec  = 0.0F;
    psParam->bReachFlag   = FALSE;

    /* 初始化输出状态（起点） */
    psParam->fltVel       = psParam->fltDir * psParam->fltV0Abs;
    psParam->fltAcc       = 0.0F;
    psParam->fltJerk      = 0.0F;
    psParam->fltState     = psParam->fltStart;

    /* build写入 */
    build_pos7_table(psParam);

    return TRUE;
}




/* 火箭同款推进器：输出位置 ramp */
float_t GFLIB_FlexSPos7_FLT_FC(GFLIB_FLEXS_POS7_T_FLT *psParam)
{
    if (psParam->bReachFlag) {
        return psParam->fltState;
    }

    /*  推进执行时间 */
    float_t t = MLIB_Add_FLT(psParam->fltTimeExec, psParam->fltTs);
    if (t >= psParam->fltTimeTotal) {
        /* 末尾   尾差 */
        psParam->fltTimeExec = psParam->fltTimeTotal;
        psParam->fltState = psParam->fltTarget;
        psParam->fltVel   = 0.0F;
        psParam->fltAcc   = 0.0F;
        psParam->fltJerk  = 0.0F;
        psParam->bReachFlag = TRUE;
        psParam->test_numeber = 5;

        return psParam->fltState;
    }
    psParam->fltTimeExec = t;

    /*  找段计算段内时间 t_state */
    const uint16_t st = find_stage(psParam, t); 
    float_t t_state = MLIB_Sub_FLT(t, psParam->segTimeBegin[st]);

    /* t_state不超过该段时长 */
    t_state = min_f(max_f(t_state, 0.0F), psParam->segTime[st]);

    float_t x_now, v_now, a_now, j_now;
    calc_motion_stage(psParam, st, t_state, &x_now, &v_now, &a_now, &j_now);

    /* 3) 传回实际方向 */
    psParam->fltJerk  = psParam->fltDir * j_now;
    psParam->fltAcc   = psParam->fltDir * a_now;
    psParam->fltVel   = psParam->fltDir * v_now;
    psParam->fltState = MLIB_Add_FLT(psParam->fltStart, MLIB_Mul_FLT(psParam->fltDir, x_now));

    return psParam->fltState;
}
