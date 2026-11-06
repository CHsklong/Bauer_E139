#ifndef PWM_H
#define PWM_H
#include "hpm_common.h"
#include "hpm_pwm_regs.h"
#include "hpm_trgm_regs.h"
#include "mcdrv.h"

extern int32_t motor_clock_hz;


#define SYSTEM_FREQUENCY 	160 		//mHZ						// Define the system frequency (MHz)
#define  INT_FREQ               20 //16              //KHZ

#define PWM_FREQUENCY           (20000)  //    (16000)
#define PWM_RELOAD                  (motor_clock_hz/PWM_FREQUENCY)

#define POS_LOOP_FRE  		 10  	//8  // 	//KHZ


#if (BOARD_TYPE == BOARD_E139 || BOARD_TYPE == BOARD_E884)
#define UH_PWM_OUTPIN         (0U)
#define UL_PWM_OUTPIN         (1U)
#define VH_PWM_OUTPIN         (2U)
#define VL_PWM_OUTPIN         (3U)

#define WH_PWM_OUTPIN         (6U)  //5300
#define WL_PWM_OUTPIN         (7U)	//


#define CMP_INDEX_0         (0U)
#define CMP_INDEX_1         (1U)
#define CMP_INDEX_2         (2U)
#define CMP_INDEX_3         (3U)
#define CMP_INDEX_4         (4U)
#define CMP_INDEX_5         (5U)
#define BOARD_PWM_TYPE                    HPM_PWM0
#define BOARD_USER_PWM_IRQ             IRQn_PWM0
#elif (BOARD_TYPE == BOARD_E914)
#define UH_PWM_OUTPIN         (0U)
#define UL_PWM_OUTPIN         (1U)
#define VH_PWM_OUTPIN         (2U)
#define VL_PWM_OUTPIN         (3U)

#define WH_PWM_OUTPIN         (4U)  //5300
#define WL_PWM_OUTPIN         (5U)	//


#define CMP_INDEX_0         (0U)
#define CMP_INDEX_1         (1U)
#define CMP_INDEX_2         (2U)
#define CMP_INDEX_3         (3U)
#define CMP_INDEX_4         (4U)
#define CMP_INDEX_5         (5U)
#define BOARD_PWM_TYPE                    HPM_PWM1
#define BOARD_USER_PWM_IRQ             IRQn_PWM1

#elif (BOARD_TYPE == BOARD_E145)
#define AH_PWM_OUTPIN         (0U)
#define AL_PWM_OUTPIN         (1U)
#define BH_PWM_OUTPIN         (4U)
#define BL_PWM_OUTPIN         (5U)
#define CH_PWM_OUTPIN         (6U)  //5300
#define CL_PWM_OUTPIN         (7U)	//
#define DH_PWM_OUTPIN         (2U)  //5300
#define DL_PWM_OUTPIN         (3U)	//


#define CMP_INDEX_0         (0U)
#define CMP_INDEX_1         (1U)
#define CMP_INDEX_2         (2U)
#define CMP_INDEX_3         (3U)
#define CMP_INDEX_4         (4U)
#define CMP_INDEX_5         (5U)
#define CMP_INDEX_6         (6U)
#define CMP_INDEX_7         (7U)
#define BOARD_PWM_TYPE                    HPM_PWM0
#define BOARD_USER_PWM_IRQ             IRQn_PWM0
#endif





#define CMP_TRIG_CMP        (15U)

#define ZL_PWM_OUTPIN       (2U)


#define BOARD_BLDCPWM_TRGM                HPM_TRGM0



#define   SVPWMOFF()      disable_all_pwm_output()
#define   SVPWMON()       enable_all_pwm_output()

extern uint32_t pwm3_reload;
extern uint32_t pwm3_reload_half;
extern uint32_t pwm3_cmp_value ;
extern uint16_t pwm_ratio_1000 ;




void pwm_pins_init(PWM_Type *ptr);
void reset_pwm_counter(void);
void pwm_central_aligned_waveform_in_pair_init(void);
void enable_all_pwm_output(void);
void close_high_pwm_output();

void disable_all_pwm_output(void);

void close_high_open_low_pwm_output(void);



void Turnoff_allupper(PWM_Type *pwm_x);
void Turnoff_alllower(PWM_Type *pwm_x);
void Turnoff_all(PWM_Type *pwm_x);
void Recover_all(PWM_Type *pwm_x);

void pwm_clear_all_fault(void) ;
void pwm_init(void) ;



/* 刹车 */
#define BRAKE_PWM_FREQ_HZ       (10000U)                    /* 默认保持频率 10kHz */
#define BRAKE_PWM_DUTY_PCT      (20U)                       /* 默认保持占空比 50% */
#define BRAKE_BOOST_MS          (1000U)                     /* 松闸前全压强励磁持续时间(ms) */
void brake_pwm_init(void);                                  /* 初始化PWM, 默认抱闸 */
void brake_pwm_config(uint32_t freq_hz, uint8_t duty_pct);  /* 配置PWM频率/占空比 */
void brake_release(void);                                   /* 保持阶段: 切 GPTMR 输出 freq/duty 的 PWM(打开刹车) */
void brake_boost(void);                                     /* 强励磁: PA13 恒高(全压), 松闸前的 1s 阶段 */
void brake_engage(void);                                    /* 抱闸: 输出恒低 */
void brake_set(uint8_t release);                            /* 1=松闸 0=抱闸 */
uint8_t brake_is_released(void);                            /* 当前状态 */
//void generate_edge_aligned_waveform(void);

extern int32_t motor_clock_hz;

#endif
