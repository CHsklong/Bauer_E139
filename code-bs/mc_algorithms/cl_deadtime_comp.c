/*=============================================================================
 * 交付阅读导引：B2 非线性死区与逆变器补偿
 * 文件角色：定点算法实现；配套 PDF 位于同级算法文件夹的“说明文档”目录。
 *-----------------------------------------------------------------------------
 * 功能概述：根据三相电流极性、母线电压和器件压降曲线计算相电压补偿，并用平滑过零与滞回避免零电流附近频繁翻转。
 *
 * 接口与调用顺序：
 *   1. 离线测量或根据器件数据表建立 current_bp_q15 与 drop_lut_q15，断点必须严格递增。
 *   2. 在 dq 电压经逆 Park 得到三相电压后调用 DeadtimeComp_Apply，把三相补偿分别相加。
 *   3. 补偿后仍需执行占空比/母线电压限幅；故障、换相序或长时间停机后调用 Reset。
 *
 * 主要公开变量/参数（最终以本文件结构体声明为准）：
 *   - deadtime_gain_q15: 死区时间、PWM 频率和电压基值合成的基础补偿幅值
 *   - vdc_nom_q15: 标称母线电压，用于母线变化时缩放死区补偿
 *   - zero_band_q15: 过零平滑区宽度；越大越平滑，但小电流补偿会变弱
 *   - zero_hyst_q15: 电流极性滞回宽度，用于抑制零点噪声造成的符号抖动
 *   - current_bp_q15[5]: 电流绝对值的五个查表断点，Q15 电流标幺
 *   - drop_lut_q15[5]: 对应断点的器件压降补偿，Q15 电压标幺
 *   - out_limit_q15: 单相最终补偿限幅
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
 * 特别注意：补偿必须加在相电压调制量处，不能直接当作 dq 电压；不同 PWM 极性、采样方向或相序可能要求整体反号。
 *============================================================================*/
/*=============================================================================
 * 死区/器件压降补偿实现阅读说明
 * lookup_drop 对 |i| 在五个断点间线性插值，断点外使用端点值；配置断点必须升序。
 * 每相先用迟滞状态确定稳定电流极性，再在 zero_band 内用三次 smoothstep
 * s(x)=x^2(3-2x) 生成连续软符号。该函数在 0 和软区边界处斜率均为 0，比线性
 * 过零更不易把采样噪声变成高频补偿电压。随后叠加死区等效压降和器件压降 LUT。
 * 三相计算完成后减去共同平均值，删除零序分量。
 * Apply 返回的是附加相电压，不修改原始调制指令，也不执行占空比限幅。
 * 若电流方向约定错误，本算法会加剧过零畸变；首次上机必须先小增益确认极性。
 *============================================================================*/
#include "cl_deadtime_comp.h"

static int32_t lookup_drop(const cl_deadtime_comp_cfg_t *cfg, int32_t current_abs)
{
    uint16_t k;
    if (current_abs <= cfg->current_bp_q15[0]) return cfg->drop_lut_q15[0];
    for (k = 1; k < CL_DEADTIME_LUT_SIZE; ++k)
    {
        int32_t x0 = cfg->current_bp_q15[k - 1u];
        int32_t x1 = cfg->current_bp_q15[k];
        if (current_abs <= x1)
        {
            int32_t y0 = cfg->drop_lut_q15[k - 1u];
            int32_t y1 = cfg->drop_lut_q15[k];
            if (x1 <= x0) return y1;
            return y0 + (int32_t)(((int64_t)(y1 - y0) * (current_abs - x0)) / (x1 - x0));
        }
    }
    return cfg->drop_lut_q15[CL_DEADTIME_LUT_SIZE - 1u];
}

static int32_t nonlinear_soft_sign(const cl_deadtime_comp_cfg_t *cfg,
                                   int32_t current,
                                   int8_t *polarity)
{
    int32_t band = cl_abs32(cfg->zero_band_q15);
    int32_t hyst = cl_abs32(cfg->zero_hyst_q15);
    int32_t mag;
    int32_t x;
    int32_t x2;

    if (hyst > band && band > 0) hyst = band;
    if (current > hyst) *polarity = 1;
    else if (current < -hyst) *polarity = -1;

    if (band == 0)
    {
        if (current > 0) return CL_Q15_ONE;
        if (current < 0) return -CL_Q15_ONE;
        return (int32_t)(*polarity) * CL_Q15_ONE;
    }

    mag = cl_abs32(current);
    if (mag >= band) return (int32_t)(*polarity) * CL_Q15_ONE;
    if (*polarity == 0) return 0;

    x = (int32_t)(((int64_t)mag * CL_Q15_ONE) / band);
    x = cl_sat32(x, CL_Q15_ONE, 0);
    x2 = cl_mulq15(x, x);
    /* smoothstep(x)=x^2*(3-2x)，定义域 x∈[0,1]。 */
    return (int32_t)(*polarity) * cl_mulq15(x2, 3 * CL_Q15_ONE - 2 * x);
}

static int32_t phase_comp(const cl_deadtime_comp_cfg_t *cfg,
                          int32_t current,
                          int32_t vdc,
                          int8_t *polarity)
{
    int32_t sign_q15;
    int32_t vdc_nom = (cfg->vdc_nom_q15 > 0) ? cfg->vdc_nom_q15 : CL_Q15_ONE;
    int32_t vdc_valid = (vdc > 0) ? vdc : 0;
    int32_t dead = (int32_t)(((int64_t)cfg->deadtime_gain_q15 * vdc_valid) / vdc_nom);
    int32_t amp = dead + lookup_drop(cfg, cl_abs32(current));

    sign_q15 = nonlinear_soft_sign(cfg, current, polarity);
    return cl_mulq15(sign_q15, amp);
}

void DeadtimeComp_Init(cl_deadtime_comp_t *self, const cl_deadtime_comp_cfg_t *cfg)
{
    self->cfg = *cfg;
    DeadtimeComp_Reset(self);
}

void DeadtimeComp_Reset(cl_deadtime_comp_t *self)
{
    self->polarity_u = 0;
    self->polarity_v = 0;
    self->polarity_w = 0;
}

void DeadtimeComp_Apply(cl_deadtime_comp_t *self, cl_deadtime_comp_io_t *io)
{
    int32_t u = phase_comp(&self->cfg, io->iu_q15, io->vdc_q15, &self->polarity_u);
    int32_t v = phase_comp(&self->cfg, io->iv_q15, io->vdc_q15, &self->polarity_v);
    int32_t w = phase_comp(&self->cfg, io->iw_q15, io->vdc_q15, &self->polarity_w);
    int32_t zero = (int32_t)(((int64_t)u + v + w) / 3);
    int32_t lim = cl_abs32(self->cfg.out_limit_q15);

    /* 三线制电机不需要零序电压，去除后可减小调制占空比占用。 */
    io->vu_ff_q15 = cl_sat32(u - zero, lim, -lim);
    io->vv_ff_q15 = cl_sat32(v - zero, lim, -lim);
    io->vw_ff_q15 = cl_sat32(w - zero, lim, -lim);
}
