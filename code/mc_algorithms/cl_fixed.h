/*==============================================================================
 * cl_fixed.h  —  电流环算法函数包·公共定点数学库
 *  统一 Q15 定点约定 / 饱和 / 定点乘法 / 整数平方根 / 定点三角
 *  所有算法模块仅依赖本头文件，纯 C（C99），无动态内存、无浮点（可选）。
 *  与现有固件约定一致：Q15 表示 x -> round(x*2^15)，乘法 (a*b)>>15。
 *
 * 工程接入注意:
 *   - 本库的 Q15 计算值存放在 int32_t 中，CL_Q15_ONE=32768。不要把所有 _q15
 *     字段改成 int16_t，否则无法表达 +1.0 和大于 1pu 的中间增益。
 *   - 物理量转 Q15：q15=round(physical/base*32768)；Q15 转物理量：
 *     physical=q15*base/32768。建议上位机或初始化代码完成换算，ISR 只用整数。
 *   - cl_mulq15 使用 int64_t 中间乘积，但多个结果直接用 int32_t 相加仍可能溢出；
 *     新增算法时应先用 int64_t 累加，再显式饱和回 int32_t。
 *   - cl_abs32 不接受 INT32_MIN；当前算法参数/信号均不应达到该异常值。外部通信
 *     数据进入算法前必须做范围校验。
 *   - 定点三角为轻量近似，角度单位是 uint16 一圈而不是弧度。若项目已有更高精度
 *     CORDIC/查表，可保持相同输入输出格式替换实现，但需重新做闭环回归测试。
 *============================================================================*/
#ifndef CL_FIXED_H
#define CL_FIXED_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CL_Q15            15
#define CL_Q15_ONE        32768        /* 1.0 in Q15 */
#define CL_Q15_MAX        32767
#define CL_Q15_MIN        (-32768)

/* 饱和 --------------------------------------------------------------------- */
static inline int32_t cl_sat32(int32_t x, int32_t hi, int32_t lo)
{ return (x > hi) ? hi : ((x < lo) ? lo : x); }

static inline int16_t cl_sat16(int32_t x)
{ return (int16_t)((x > CL_Q15_MAX) ? CL_Q15_MAX : ((x < CL_Q15_MIN) ? CL_Q15_MIN : x)); }

static inline int32_t cl_abs32(int32_t x) { return x < 0 ? -x : x; }

/* Q15 定点乘法（对称移位，避免负数右移偏置；int64 中间量防溢出） ------------- */
static inline int32_t cl_mulq15(int32_t a, int32_t b)
{
    int64_t p = (int64_t)a * (int64_t)b;
    return (p >= 0) ? (int32_t)(p >> 15) : -(int32_t)((-p) >> 15);
}
/* 通用 Q 乘法：结果右移 q 位 */
static inline int32_t cl_mulq(int32_t a, int32_t b, int q)
{
    int64_t p = (int64_t)a * (int64_t)b;
    return (p >= 0) ? (int32_t)(p >> q) : -(int32_t)((-p) >> q);
}

/* 32位整数平方根（牛顿法，x>=0，返回 floor(sqrt(x))） ---------------------- */
static inline int32_t cl_isqrt32(int32_t x)
{
    int32_t s, r;
    if (x <= 0) return 0;
    s = x; r = 1;
    while (s > 3) { s >>= 2; r <<= 1; }
    r = (r + x / r) >> 1;
    r = (r + x / r) >> 1;
    r = (r + x / r) >> 1;
    r = (r + x / r) >> 1;
    r = (r + x / r) >> 1;
    r = (r + x / r) >> 1;
    if ((int64_t)r * r > x) r--;
    return r;
}
/* 64位整数平方根（x>=0，返回 floor(sqrt(x))），用于 Q30 求根得到 Q15 -------- */
static inline uint32_t cl_isqrt64(uint64_t x)
{
    uint64_t bit = (uint64_t)1 << 62;
    uint64_t res = 0;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) { x -= res + bit; res = (res >> 1) + bit; }
        else                {                  res >>= 1; }
        bit >>= 2;
    }
    return (uint32_t)res;
}

/* Q15 定点正/余弦：输入电角度为 uint16（0..65535 = 0..2π），与固件 Q16 角一致。
 * 采用 Bhaskara I 逼近（最大误差约 0.16%）；生产环境可替换为固件的 SINECOSINE 查表。*/
static inline int32_t cl_sin_q15(uint16_t theta)
{
    /* 归一到 [0, 2^16) -> 象限处理 */
    uint32_t a = theta;
    int32_t sign = 1;
    if (a >= 32768u) { a -= 32768u; sign = -1; }      /* 后半周期取负 */
    /* a in [0,32768) 对应 [0,π)；令 x = a/32768*π。Bhaskara: sin = 16x(π-x)/(5π^2-4x(π-x)) */
    /* 用归一角 u = a/32768 in [0,1)，sin(πu) = 16u(1-u)/(5-4u(1-u)) */
    int64_t u  = a;                       /* Q15 (0..1) */
    int64_t um = (int64_t)32768 - u;      /* (1-u) Q15 */
    int64_t num = 16 * u * um;            /* Q30 * ... -> scale later */
    int64_t den = (int64_t)5 * 32768 * 32768 - 4 * u * um; /* Q30 */
    /* sin = num/den in [0,1] -> Q15 */
    int32_t s = (int32_t)((num << 15) / den);
    return sign * s;
}
static inline int32_t cl_cos_q15(uint16_t theta)
{ return cl_sin_q15((uint16_t)(theta + 16384u)); }   /* cos = sin(θ+π/2) */

#ifdef __cplusplus
}
#endif
#endif /* CL_FIXED_H */
