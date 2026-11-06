/*=============================================================================
 * 交付阅读导引：C2--C4 5/7/11/13 次特定谐波抑制（多同步旋转坐标系）
 * 文件角色：定点算法实现；配套 PDF 位于同级算法文件夹的“说明文档”目录。
 *-----------------------------------------------------------------------------
 * 功能概述：把 5、7、11、13 次电流误差分别变换到对应同步旋转坐标系，提取直流分量，经 PI 和可选谐波阻抗解耦后反变换并合成补偿电压。
 *
 * 接口与调用顺序：
 *   1. 电流参考与反馈先在同一 αβ 坐标系相减得到误差，角度必须是同一方向的电角度。
 *   2. 先只使能一个通道，检查 ed_raw/eq_raw 与 ed_lpf/eq_lpf；确认提取出近似直流后再增加 PI。
 *   3. 将 u_alpha/u_beta 加到主环逆 Park 后的 αβ 电压，在 SVPWM 前做总电压限幅；不同通道逐个验证。
 *
 * 主要公开变量/参数（最终以本文件结构体声明为准）：
 *   - order[4]: 固定阶次 5、7、11、13
 *   - dir[4]: 旋转方向，推荐 -1、+1、-1、+1；与角度定义相反时需整体核对
 *   - enable[4]: 四通道独立使能；调试时一次只开一路
 *   - kp_q15[4] / ki_q15[4]: 每通道 PI 增益；Ki 为每采样周期增益
 *   - lpf_preset: 0/1/2/3 对应 25/50/100/200 rad/s；当前系数按 16 kHz 电流环计算
 *   - lpf_enable: 1 使用二阶低通后的 dq 直流量；0 直接使用原始旋转坐标量
 *   - impedance_enable: 1 启用 Rs、ωhLd/Lq 谐波坐标阻抗解耦
 *   - rs_q15 / ld_q10 / lq_q10: 电机参数，必须按本工程标幺规则换算
 *   - omega_e_q15: 基波电角速度，阻抗解耦使用
 *   - state_limit_q15 / out_limit_q15: PI 积分状态和合成补偿电压限幅
 *
 * 跨工程单位适配：
 *   名称带 _q15 的量采用有符号 Q15 标幺，1.0 对应 32768；角度 _q16
 *   采用一圈 65536 计数。移植前必须统一电流基值 Ibase、电压基值 Vbase、
 *   速度基值和电角度正方向。若甲方工程使用 A/V/rad/s、IQ 格式或不同基值，
 *   应在调用层换算，不建议在算法内部混用单位。int32_t 用于容纳中间超 1 pu
 *   的 Q15 量，不能仅因最终 PWM 为 16 位就改成 int16_t。
 *
 * 安全要求：所有补偿/控制输出在写入 PWM 前仍须经过母线归一化、矢量限幅、
 *   过流过压保护和功率级联锁。首次上机从小增益、小限幅、单算法使能开始。
 * 特别注意：若开启后目标谐波反而增大，优先检查 err=ref-fdb 的符号、角度方向、dir、补偿电压叠加符号和控制频率，而不是继续加大 PI。
 *============================================================================*/
/*=============================================================================
 * 与仿真模型匹配的 5/7/11/13 次选择性谐波电流抑制器。
 *
 * 采样频率适配：
 * PMSM_harmonic_571113.slx 的四个谐波坐标系使用同一型式的二阶低通，
 * 仿真采样率为 10 kHz；下列系数保持连续域极点和单位直流增益，
 * 并已换算到当前固件 16 kHz 电流环采样率。
 *============================================================================*/
#include "cl_srf_harm.h"

typedef struct {
    int32_t b0_q30;
    int32_t b1_q30;
    int32_t b2_q30;
    int32_t a1_q30;
    int32_t a2_q30;
} srf_lpf_coeff_t;

/* 16 kHz 二阶低通系数。四行分别对应 preset=0/1/2/3；保持模型极点比例，
 * 两个连续域极点按 preset/50 同比缩放，并保持精确单位直流增益。 */
static const srf_lpf_coeff_t sSrfLpfCoeff[4] = {
    {   9233,   18464,   9233, -2128638481, 1054933587 }, /* preset=0：25 rad/s */
    {  36608,   73217,  36608, -2110048290, 1036452899 }, /* preset=1：50 rad/s */
    { 143920,  287838, 143920, -2073610357, 1000444211 }, /* preset=2：100 rad/s */
    { 556461, 1112923, 556461, -2003556100,  932040121 }  /* preset=3：200 rad/s */
};

static int32_t srf_sat_i64(int64_t x)
{
    if (x > INT32_MAX) return INT32_MAX;
    if (x < INT32_MIN) return INT32_MIN;
    return (int32_t)x;
}

static int64_t srf_rshift_round(int64_t x, uint16_t shift)
{
    int64_t half = (int64_t)1 << (shift - 1U);
    if (x >= 0) {
        return (x + half) >> shift;
    }
    return -(((-x) + half) >> shift);
}

static int32_t srf_lpf_step(const srf_lpf_coeff_t *coeff,
                            int32_t input,
                            int32_t *x1, int32_t *x2,
                            int32_t *y1, int32_t *y2)
{
    int64_t feed_q15frac;
    int64_t fb1_q15frac;
    int64_t fb2_q15frac;
    int64_t state_q15frac;
    int32_t output;

    /*
     * y1/y2 不是公开的 Q15 样本：内部额外保留 15 位小数。
     * 若没有这些保护小数位，小幅谐波经过低通分子后可能小于 1 个 Q15 LSB，
     * 定点滤波器会长期停在 0，导致无法观察和调节小幅谐波。
     */
    feed_q15frac = srf_rshift_round((int64_t)coeff->b0_q30 * input
                                    + (int64_t)coeff->b1_q30 * (*x1)
                                    + (int64_t)coeff->b2_q30 * (*x2), 15U);
    fb1_q15frac = srf_rshift_round((int64_t)coeff->a1_q30 * (*y1), 30U);
    fb2_q15frac = srf_rshift_round((int64_t)coeff->a2_q30 * (*y2), 30U);
    state_q15frac = feed_q15frac - fb1_q15frac - fb2_q15frac;
    state_q15frac = srf_sat_i64(state_q15frac);
    output = srf_sat_i64(srf_rshift_round(state_q15frac, 15U));

    *x2 = *x1;
    *x1 = input;
    *y2 = *y1;
    *y1 = (int32_t)state_q15frac;
    return output;
}

static int32_t srf_cross_term(int32_t inductance_q10,
                              int32_t omega_h_q15,
                              int32_t current_q15)
{
    int64_t li_q15;
    int64_t cross_q15;

    li_q15 = ((int64_t)inductance_q10 * current_q15) >> 10;
    cross_q15 = (li_q15 * omega_h_q15) >> 15;
    return srf_sat_i64(cross_q15);
}

static void srf_reset_channel(cl_srf_harm_t *self, uint16_t k)
{
    self->xd[k] = 0;
    self->xq[k] = 0;
    self->ed_raw[k] = 0;
    self->eq_raw[k] = 0;
    self->ed_lpf[k] = 0;
    self->eq_lpf[k] = 0;
    self->pi_d[k] = 0;
    self->pi_q[k] = 0;
    self->vd_cmd[k] = 0;
    self->vq_cmd[k] = 0;
    self->ed_x1[k] = 0;
    self->ed_x2[k] = 0;
    self->ed_y1[k] = 0;
    self->ed_y2[k] = 0;
    self->eq_x1[k] = 0;
    self->eq_x2[k] = 0;
    self->eq_y1[k] = 0;
    self->eq_y2[k] = 0;
}

void SrfHarm_Init(cl_srf_harm_t *self, const cl_srf_harm_cfg_t *cfg)
{
    self->cfg = *cfg;
    if (self->cfg.n_harm > SRF_MAX_H) self->cfg.n_harm = SRF_MAX_H;
    SrfHarm_Reset(self);
}

void SrfHarm_Reset(cl_srf_harm_t *self)
{
    uint16_t k;
    for (k = 0; k < SRF_MAX_H; ++k) {
        srf_reset_channel(self, k);
    }
}

void SrfHarm_Apply(cl_srf_harm_t *self, cl_srf_harm_io_t *io)
{
    int32_t ea = io->err_alpha_q15;
    int32_t eb = io->err_beta_q15;
    int32_t state_lim = cl_abs32(self->cfg.state_limit_q15);
    int32_t out_lim;
    int64_t ua = 0;
    int64_t ub = 0;
    uint16_t k;
    uint16_t lpf_preset = self->cfg.lpf_preset;
    const srf_lpf_coeff_t *lpf_coeff;

    if (lpf_preset > 3U) lpf_preset = 1U;
    lpf_coeff = &sSrfLpfCoeff[lpf_preset];

    for (k = 0; k < self->cfg.n_harm; ++k)
    {
        uint16_t ang;
        int32_t c, s;
        int32_t ed, eq;
        int32_t ed_ctrl, eq_ctrl;
        int32_t xd, xq;
        int32_t pi_d, pi_q;
        int32_t vd, vq;

        if (!self->cfg.enable[k]) {
            srf_reset_channel(self, k);
            continue;
        }

        /* 谐波坐标角 = 方向 × 阶次 × 电角度；uint16_t 溢出自然实现 2π 环绕。 */
        ang = (uint16_t)((int32_t)self->cfg.dir[k]
                         * (int32_t)self->cfg.order[k]
                         * (int32_t)io->theta_q16);
        c = cl_cos_q15(ang);
        s = cl_sin_q15(ang);

        /* Park 变换 R(-ang)：目标阶次在该旋转坐标系中变为近似直流。 */
        ed =  cl_mulq15(ea, c) + cl_mulq15(eb, s);
        eq = -cl_mulq15(ea, s) + cl_mulq15(eb, c);
        self->ed_raw[k] = ed;
        self->eq_raw[k] = eq;

        self->ed_lpf[k] = srf_lpf_step(lpf_coeff, ed,
                                       &self->ed_x1[k], &self->ed_x2[k],
                                       &self->ed_y1[k], &self->ed_y2[k]);
        self->eq_lpf[k] = srf_lpf_step(lpf_coeff, eq,
                                       &self->eq_x1[k], &self->eq_x2[k],
                                       &self->eq_y1[k], &self->eq_y2[k]);
        ed_ctrl = self->cfg.lpf_enable ? self->ed_lpf[k] : ed;
        eq_ctrl = self->cfg.lpf_enable ? self->eq_lpf[k] : eq;

        xd = cl_sat32(srf_sat_i64((int64_t)self->xd[k]
                                  + cl_mulq15(self->cfg.ki_q15[k], ed_ctrl)),
                       state_lim, -state_lim);
        xq = cl_sat32(srf_sat_i64((int64_t)self->xq[k]
                                  + cl_mulq15(self->cfg.ki_q15[k], eq_ctrl)),
                       state_lim, -state_lim);
        self->xd[k] = xd;
        self->xq[k] = xq;

        pi_d = srf_sat_i64((int64_t)cl_mulq15(self->cfg.kp_q15[k], ed_ctrl) + xd);
        pi_q = srf_sat_i64((int64_t)cl_mulq15(self->cfg.kp_q15[k], eq_ctrl) + xq);
        self->pi_d[k] = pi_d;
        self->pi_q[k] = pi_q;

        if (self->cfg.impedance_enable) {
            int32_t cmd_d;
            int32_t cmd_q;
            int32_t omega_h;
            int32_t cross_d;
            int32_t cross_q;

            /* 与 Simulink 结构一致：谐波阻抗 Zh 作用于（滤波误差 + PI 输出）。 */
            cmd_d = srf_sat_i64((int64_t)ed_ctrl + pi_d);
            cmd_q = srf_sat_i64((int64_t)eq_ctrl + pi_q);
            omega_h = srf_sat_i64((int64_t)self->cfg.dir[k]
                                   * self->cfg.order[k]
                                   * self->cfg.omega_e_q15);
            cross_d = srf_cross_term(self->cfg.ld_q10, omega_h, cmd_d);
            cross_q = srf_cross_term(self->cfg.lq_q10, omega_h, cmd_q);

            /* 谐波坐标阻抗：vd=R·id-ωh·Lq·iq；vq=R·iq+ωh·Ld·id。 */
            vd = srf_sat_i64((int64_t)cl_mulq15(self->cfg.rs_q15, cmd_d) - cross_q);
            vq = srf_sat_i64((int64_t)cl_mulq15(self->cfg.rs_q15, cmd_q) + cross_d);
        } else {
            vd = pi_d;
            vq = pi_q;
        }
        self->vd_cmd[k] = vd;
        self->vq_cmd[k] = vq;

        /* 逆 Park 变换 R(+ang)，再累加所有已使能通道的 αβ 补偿电压。 */ 
        ua += (int64_t)cl_mulq15(vd, c) - cl_mulq15(vq, s);
        ub += (int64_t)cl_mulq15(vd, s) + cl_mulq15(vq, c);
    }

    out_lim = cl_abs32(self->cfg.out_limit_q15);
    io->u_alpha_q15 = cl_sat32(srf_sat_i64(ua), out_lim, -out_lim);
    io->u_beta_q15  = cl_sat32(srf_sat_i64(ub), out_lim, -out_lim);
}
