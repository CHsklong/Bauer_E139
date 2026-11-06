/*=============================================================================
 * 交付阅读导引：B1 dq轴交叉耦合解耦前馈补偿
 * 文件角色：公开接口与数据结构声明；配套 PDF 位于同级算法文件夹的“说明文档”目录。
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
/*==============================================================================
 * cl_dq_decouple.h — PMSM d/q 轴交叉耦合解耦与反电动势前馈
 *------------------------------------------------------------------------------
 * 作用: 根据实测 id/iq 和电角速度计算
 *       vd_ff = -omega * Lq * iq
 *       vq_ff =  omega * (Ld * id + psi_f)
 * 并通过一阶低通平滑后输出。调用方把 vd_ff/vq_ff 叠加到 PI 输出即可。
 *
 * 标幺约定: 电流、速度、增益和输出均为 Q15。ld/lq/flux_gain 已由电机参数和
 * Ibase/Vbase 折算成标幺系数，避免实时使用浮点。
 *
 * 工程接入:
 *   - 调用位置：完成 Park 变换并得到本拍 id/iq 后、执行 d/q 电压限幅之前调用。
 *   - 典型连接：vd_cmd=vd_pi+vd_ff，vq_cmd=vq_pi+vq_ff。若 CurrentPi 的 ff 字段
 *     已接入本模块输出，就不要在外部再次相加，否则会重复补偿。
 *   - omega_q15 必须是电角速度而非机械速度；机械速度需先乘极对数再标幺化。
 *   - filter_alpha_q15 越接近 32768 响应越快；参数变化、停机或方向突变时 Reset。
 *   - 低速下速度噪声可能大于真实前馈，建议通过 omega_deadband_q15 平滑退出。
 *============================================================================*/
#ifndef CL_DQ_DECOUPLE_H
#define CL_DQ_DECOUPLE_H
#include "cl_fixed.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t ld_gain_q15;       /* Ld 交叉耦合标幺系数 */
    int32_t lq_gain_q15;       /* Lq 交叉耦合标幺系数 */
    int32_t flux_gain_q15;     /* 永磁磁链反电动势标幺系数 */
    int32_t filter_alpha_q15;  /* 前馈低通系数，32768=不滤波 */
    int32_t omega_deadband_q15;/* 低速死区，防速度噪声放大 */
    int32_t out_limit_q15;     /* 单轴前馈电压限幅 */
} cl_dq_decouple_cfg_t;

typedef struct {
    int32_t id_q15;       /* [in] d 轴反馈电流 */
    int32_t iq_q15;       /* [in] q 轴反馈电流 */
    int32_t omega_q15;    /* [in] 电角速度标幺值，可正可负 */
    int32_t vd_ff_q15;    /* [out] d 轴解耦前馈 */
    int32_t vq_ff_q15;    /* [out] q 轴解耦+反电动势前馈 */
} cl_dq_decouple_io_t;

typedef struct {
    cl_dq_decouple_cfg_t cfg;
    int32_t vd_state_q15;
    int32_t vq_state_q15;
} cl_dq_decouple_t;

/* 根据 cfg 初始化一个独立实例并清零滤波状态；仅在非实时初始化阶段调用。 */
void DqDecouple_Init(cl_dq_decouple_t *self, const cl_dq_decouple_cfg_t *cfg);
/* 清零 vd/vq 前馈滤波状态；故障、停机和速度方向突变时调用。 */
void DqDecouple_Reset(cl_dq_decouple_t *self);
/* 单拍计算。调用前写入 id/iq/omega，返回后读取 vd_ff/vq_ff。 */
void DqDecouple_Apply(cl_dq_decouple_t *self, cl_dq_decouple_io_t *io);

#ifdef __cplusplus
}
#endif
#endif
