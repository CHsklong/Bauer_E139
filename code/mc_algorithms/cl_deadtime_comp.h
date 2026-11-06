/*=============================================================================
 * 交付阅读导引：B2 非线性死区与逆变器补偿
 * 文件角色：公开接口与数据结构声明；配套 PDF 位于同级算法文件夹的“说明文档”目录。
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
/*==============================================================================
 * cl_deadtime_comp.h — 逆变器死区、管压降和过零非线性补偿
 *------------------------------------------------------------------------------
 * 作用: 按三相电流方向生成相电压前馈。过零区使用带迟滞极性记忆的三次
 * smoothstep 非线性软符号，避免硬切换和电流噪声造成的极性抖动；管压降用
 * 5 点电流幅值查表插值；最后去除三相零序分量，便于直接叠加到三相调制电压。
 *
 * 工程接入:
 *   - 调用位置：逆 Park/Clarke 得到三相电压指令之后、SVPWM 占空比计算之前。
 *   - iu/iv/iw 的正方向必须与电压正方向一致；方向反接会使补偿变成正反馈。
 *   - current_bp_q15 必须非负且严格升序；drop_lut_q15 对应实测压降幅值。
 *   - zero_hyst_q15 应小于 zero_band_q15。前者决定电流极性翻转门槛，后者决定
 *     非线性平滑过渡宽度；典型可分别从额定电流的 0.2% 和 1% 开始标定。
 *   - vdc_q15 与 vdc_nom_q15 必须使用同一母线基值，vdc 无效或过小时上层应禁用。
 *   - 输出已去零序，但仍需与原三相指令相加后统一做调制度/占空比限幅。
 *   - 初次标定先令 drop_lut=0，只确认死区项极性；再逐步加入管压降曲线。
 *============================================================================*/
#ifndef CL_DEADTIME_COMP_H
#define CL_DEADTIME_COMP_H
#include "cl_fixed.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CL_DEADTIME_LUT_SIZE 5

typedef struct {
    int32_t deadtime_gain_q15; /* 额定母线下的死区等效电压 */
    int32_t vdc_nom_q15;       /* 母线额定值，通常取 32768 */
    int32_t zero_band_q15;     /* 非线性过零软区半宽，必须 >=0 */
    int32_t zero_hyst_q15;     /* 电流极性迟滞门槛，建议 < zero_band */
    int32_t current_bp_q15[CL_DEADTIME_LUT_SIZE]; /* 管压降查表电流断点，升序 */
    int32_t drop_lut_q15[CL_DEADTIME_LUT_SIZE];   /* 对应管压降补偿幅值 */
    int32_t out_limit_q15;     /* 每相补偿限幅 */
} cl_deadtime_comp_cfg_t;

typedef struct {
    int32_t iu_q15;      /* [in] U 相电流 */
    int32_t iv_q15;      /* [in] V 相电流 */
    int32_t iw_q15;      /* [in] W 相电流 */
    int32_t vdc_q15;     /* [in] 当前母线电压，和 vdc_nom 同标幺 */
    int32_t vu_ff_q15;   /* [out] U 相补偿电压 */
    int32_t vv_ff_q15;   /* [out] V 相补偿电压 */
    int32_t vw_ff_q15;   /* [out] W 相补偿电压 */
} cl_deadtime_comp_io_t;

typedef struct {
    cl_deadtime_comp_cfg_t cfg;
    int8_t polarity_u;         /* 过零迟滞极性记忆: -1/0/+1 */
    int8_t polarity_v;
    int8_t polarity_w;
} cl_deadtime_comp_t;

/* 保存配置。cfg 可来自 Flash 标定区；初始化后调用方可释放临时 cfg。 */
void DeadtimeComp_Init(cl_deadtime_comp_t *self, const cl_deadtime_comp_cfg_t *cfg);
/* 清零三相极性记忆；故障、停机或电流方向约定变化后调用。 */
void DeadtimeComp_Reset(cl_deadtime_comp_t *self);
/* 单拍计算。调用前写三相电流和母线电压，返回三相补偿电压。 */
void DeadtimeComp_Apply(cl_deadtime_comp_t *self, cl_deadtime_comp_io_t *io);

#ifdef __cplusplus
}
#endif
#endif
