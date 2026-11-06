/*=============================================================================
 * 交付阅读导引：B1 dq轴交叉耦合解耦前馈补偿
 * 文件角色：定点算法实现；配套 PDF 位于同级算法文件夹的“说明文档”目录。
 *-----------------------------------------------------------------------------
 * 功能概述：根据电机电感、磁链和电角速度计算 dq 轴耦合电压，将其作为前馈量叠加到主电流控制器输出，减小高速区 d、q 两轴相互影响。
 *
 * 接口与调用顺序：
 *   1. 用实际电机参数和工程基值计算 ld_gain_q15、lq_gain_q15、flux_gain_q15，调用 DqDecouple_Init。
 *   2. 在每次电流环中断取得 id、iq 和电角速度后调用 DqDecouple_Apply。
 *   3. 把 vd_ff、vq_ff 加到主电流控制器的 dq 电压输出，再做电压矢量限幅；停机、故障和控制模式切换时调用 Reset。
 *
 * 主要公开变量/参数（最终以本文件结构体声明为准）：
 *   - ld_gain_q15 / lq_gain_q15: Ld、Lq 与电流/转速/电压基值合成后的前馈系数，Q15
 *   - flux_gain_q15: 永磁磁链与转速/电压基值合成后的 q 轴反电势系数，Q15
 *   - filter_alpha_q15: 前馈一阶低通系数；越大响应越快，噪声也越容易进入
 *   - omega_deadband_q15: 电角速度死区；低于该值时关闭耦合前馈以抑制零速抖动
 *   - out_limit_q15: 每轴前馈电压绝对限幅，Q15 电压标幺
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
 * 特别注意：若主控制器已经内置同一组 dq 解耦项，不要重复叠加。正负转速都要验证 q 轴前馈符号。
 *============================================================================*/
/*=============================================================================
 * d/q 解耦前馈实现阅读说明
 * Apply 先对低速死区做平滑处理，再按 PMSM 模型计算 vd=-w*Lq*iq、
 * vq=w*(Ld*id+psi_f)，之后分别做一阶低通和单轴限幅。
 * filter_alpha=32768 时新计算值直接通过；alpha 越小，状态保留越多、延迟越大。
 * 输出只包含前馈，不包含 PI、电压矢量限幅或 SVPWM；调用方负责正确叠加。
 * 乘法使用 Q15 标幺，ld/lq/flux 的换算错误不会在本文件内被自动识别。
 *============================================================================*/
#include "cl_dq_decouple.h"

void DqDecouple_Init(cl_dq_decouple_t *self, const cl_dq_decouple_cfg_t *cfg)
{
    self->cfg = *cfg;
    DqDecouple_Reset(self);
}

void DqDecouple_Reset(cl_dq_decouple_t *self)
{
    self->vd_state_q15 = 0;
    self->vq_state_q15 = 0;
}

void DqDecouple_Apply(cl_dq_decouple_t *self, cl_dq_decouple_io_t *io)
{
    int32_t omega = io->omega_q15;
    int32_t alpha = cl_sat32(self->cfg.filter_alpha_q15, CL_Q15_ONE, 0);
    int32_t vd_raw = 0;
    int32_t vq_raw = 0;
    int32_t lim = cl_abs32(self->cfg.out_limit_q15);

    if (cl_abs32(omega) > cl_abs32(self->cfg.omega_deadband_q15))
    {
        int32_t lq_iq = cl_mulq15(self->cfg.lq_gain_q15, io->iq_q15);
        int32_t ld_id = cl_mulq15(self->cfg.ld_gain_q15, io->id_q15);
        vd_raw = -cl_mulq15(omega, lq_iq);
        vq_raw =  cl_mulq15(omega, ld_id + self->cfg.flux_gain_q15);
    }

    self->vd_state_q15 += cl_mulq15(alpha, vd_raw - self->vd_state_q15);
    self->vq_state_q15 += cl_mulq15(alpha, vq_raw - self->vq_state_q15);
    io->vd_ff_q15 = cl_sat32(self->vd_state_q15, lim, -lim);
    io->vq_ff_q15 = cl_sat32(self->vq_state_q15, lim, -lim);
}
