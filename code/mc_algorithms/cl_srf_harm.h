/*=============================================================================
 * 交付阅读导引：C2--C4 5/7/11/13 次特定谐波抑制（多同步旋转坐标系）
 * 文件角色：公开接口与数据结构声明；配套 PDF 位于同级算法文件夹的“说明文档”目录。
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
 * 5/7/11/13 次选择性谐波电流抑制器（多同步旋转坐标系实现）。
 *
 * 实现结构与已验证的 PMSM_harmonic_571113.slx 仿真模型一致：
 *   1. 将静止 αβ 电流误差旋转到 -5/+7/-11/+13 次坐标系。
 *   2. 每个通道使用独立二阶低通提取对应坐标系中的直流分量。
 *   3. 用独立 d/q PI 调节提取后的谐波电流误差。
 *   4. 可选应用谐波坐标系 R/ωL 阻抗和交叉耦合模型。
 *   5. 将各次谐波电压反变换到 αβ 坐标系后求和。
 *
 * 定点数约定：
 *   电流误差、PI 信号和电压输出：int32_t 保存的 Q15。
 *   电角度：uint16_t，0..65535 表示一电角周期。
 *   定子电阻：固件标幺 Q15。
 *   电机电感：固件标幺 Q10。
 *============================================================================*/
#ifndef CL_SRF_HARM_H
#define CL_SRF_HARM_H

#include "cl_fixed.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SRF_MAX_H 4

typedef struct {
    uint16_t n_harm;                 /* 实际配置的谐波通道数，最大为 4。 */
    uint16_t order[SRF_MAX_H];       /* 谐波阶次；索引 0/1/2/3 对应 5/7/11/13。 */
    int16_t  dir[SRF_MAX_H];         /* 旋转方向：-1 负序，+1 正序。 */
    uint16_t enable[SRF_MAX_H];      /* 通道使能；关闭时清零该通道全部滤波/积分状态。 */
    int32_t  kp_q15[SRF_MAX_H];      /* 每通道 PI 比例增益，Q15。 */
    int32_t  ki_q15[SRF_MAX_H];      /* 每采样周期积分增益，Q15；改变中断频率后需重算。 */
    int32_t  rs_q15;                 /* 定子电阻标幺值，Q15。 */
    int32_t  ld_q10;                 /* d 轴电感标幺值，Q10。 */
    int32_t  lq_q10;                 /* q 轴电感标幺值，Q10。 */
    int32_t  omega_e_q15;            /* 基波电角速度标幺值，Q15。 */
    uint16_t lpf_enable;              /* 二阶低通使能；0 时 PI 直接使用 ed_raw/eq_raw。 */
    uint16_t lpf_preset;              /* 预设 0/1/2/3 = 25/50/100/200 rad/s；系数按 16 kHz。 */
    uint16_t impedance_enable;        /* 谐波坐标阻抗解耦使能。 */
    int32_t  state_limit_q15;        /* 每通道 d/q PI 积分状态绝对限幅。 */
    int32_t  out_limit_q15;          /* 四通道合成后 αβ 补偿电压绝对限幅。 */
} cl_srf_harm_cfg_t;

typedef struct {
    int32_t  err_alpha_q15;
    int32_t  err_beta_q15;
    uint16_t theta_q16;
    int32_t  u_alpha_q15;
    int32_t  u_beta_q15;
} cl_srf_harm_io_t;

typedef struct {
    cl_srf_harm_cfg_t cfg;
    int32_t xd[SRF_MAX_H];
    int32_t xq[SRF_MAX_H];
    int32_t ed_raw[SRF_MAX_H];
    int32_t eq_raw[SRF_MAX_H];
    int32_t ed_lpf[SRF_MAX_H];
    int32_t eq_lpf[SRF_MAX_H];
    int32_t pi_d[SRF_MAX_H];
    int32_t pi_q[SRF_MAX_H];
    int32_t vd_cmd[SRF_MAX_H];
    int32_t vq_cmd[SRF_MAX_H];
    int32_t ed_x1[SRF_MAX_H], ed_x2[SRF_MAX_H];
    /* IIR 的 y 状态比公开 Q15 信号多保留 15 位小数，防止小谐波量化为零。 */
    int32_t ed_y1[SRF_MAX_H], ed_y2[SRF_MAX_H];
    int32_t eq_x1[SRF_MAX_H], eq_x2[SRF_MAX_H];
    int32_t eq_y1[SRF_MAX_H], eq_y2[SRF_MAX_H];
} cl_srf_harm_t;

void SrfHarm_Init(cl_srf_harm_t *self, const cl_srf_harm_cfg_t *cfg);
void SrfHarm_Reset(cl_srf_harm_t *self);
void SrfHarm_Apply(cl_srf_harm_t *self, cl_srf_harm_io_t *io);

#ifdef __cplusplus
}
#endif
#endif
