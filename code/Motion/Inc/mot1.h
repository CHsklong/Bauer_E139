

#ifndef MOT1_H_
#define MOT1_H_

//----------------------------------
#define     MT_ANALOG       0
#define     MT_SPI          1

#define     MT_TYPE         MT_SPI
//-------------------------------------
#define     ANG_MAX      gan_pos.ang_max   //400

#define         GAN_ERR_ZONE     5
#define         GEN_ERR_NEG     -GAN_ERR_ZONE

//#define     KGAN2MENC       0.008064516   //      1/124

#define     KGAN2MENC       0.0122222   //      1/82

#define     SYSTEM_HQ       1
enum {

    KP_MODE = 1,
    KI_MODE = 2,

    KP_KI_MODE = 3

};

enum {

    ACC_STATE = 1,
    BAN_GAN_HUI_ZHENG_STATE = 2,
    AVG_STATE = 3,
    DEC_STATE = 4,
    STOP_STATE = 5


};


typedef struct {
    uint8_t     index;  //  jtag set id
    uint8_t     gan_set_zero;
    uint8_t     mach_id;
    uint8_t     dx_err_cnt;
    s32         motor_abs_zero;
    s32         gan_abs_zero;

    int32_t     gan_pos_lower;
    int32_t     gan_pos_uper;
    int32_t     tbd_chi_len ; //同步带齿距校正  19026
    int16_t     needle_num; // 总针数  1872
    int8_t      tab_use;
    int8_t      res;

    int16_t     tab[610];
    int16_t     comp_tor;
    float       kb;
    float       kg;
    int8_t      kflag;
    int8_t     res2;

} EEPROM_PAR;
#define EE_SIZE_BYTE    sizeof(EEPROM_PAR)

typedef struct {
    uint8_t     index;
    uint8_t     mach_id;

} EEPROM_PAR2;

#define EE2_SIZE_BYTE    sizeof(EEPROM_PAR)

extern  uint8_t  cmd_sel  ;



typedef struct {
    int16_t     index;
    int16_t     encInc;
    int8_t      start;
    int8_t      stop;
    int8_t      cnt1;
    int8_t      cnt2;
    int16_t     clearCnt;
//  -----can ---
    int32_t     QepData_bk;
    int32_t     cur_pos_bk;

    int32_t     QepData_inc;
    int32_t     cur_pos_inc;

    int32_t     QepData_delt;
    int32_t     cur_pos_delt;
    int32_t     cur_pos2mt;  //*k 转为电机编码器单位
//--------------
    int32_t     mt_enc_add;  //
    float       mt_enc_add_mm;

    int32_t     pos_enc_add;
    float       pos_enc_add_mm;
    float       com_add_mm;
    float       com_add_mm_old;

    float      com_mm_10ms;

    int64_t     mt_enc_add64;
    int32_t     motor_len_mm;

    int32_t     bai_gan_mm;

    int32_t     send402_len_mm;
    int32_t     sing_line_mm;

//----------------
    int32_t     encSum;
    int32_t     pulsSum;

    int32_t     sum[4];
    float       lenf[4];
    float       psumf ;
    float       cir_dia;
    float       Kp2mm;
    float       Kmm2p;
    //---捕获数据------ 发送--------
    int8_t      send_flag;

    int8_t      id;
    int16_t     send_line_num;   //行号
    int32_t     send_len_mm;
    int32_t     cap_len_mm;



} LEN_MEAS_TYPEDEF;

//---------------------------------
//
#define  XCNUM      10
typedef struct {

    int16_t     i;
    int8_t      j;
    int8_t      avg_num;

    int32_t     p_sum;
    int32_t     n_sum;
    int16_t     pavg;
    int16_t     navg;
    int16_t     lenp[XCNUM];
    int16_t     lenn[XCNUM];

    int16_t     ppenct[XCNUM];
    int16_t     npenct[XCNUM];
} XC_MEMORY_TYPEDEF;




//---------------------------------
/*
typedef struct {
    float  Ref;             // Input: Reference input
    float  Fbk;             // Input: Feedback input
    float  Err;             // Variable: Error
    float  Err2;

    float  Kp;              // Parameter: Proportional gain
    float  Up;              // Variable: Proportional output
    float  ui;              // Variable: Integral output
    float  OutPreSat;       // Variable: Pre-saturated output
    float  Umax;            // Parameter: Maximum output
    float  Umin;            // Parameter: Minimum output
    float  Out;             // Output: PID output
    float  SatErr;          // Variable: Saturated difference
    float  Ki;              // Parameter: Integral gain
    float  Kc;              // Parameter: Integral correction gain
    float  uKi;
    float  tmp;
    float  VbusRatio;
    float  Emf;
} PID_CLA;
*/
typedef struct {

    int16_t spd;  //高速fang'xiang
    int8_t  k1;
    int8_t  k2;
    int8_t  k3;
    int8_t  k4;
    int8_t  k5;
    int8_t  k6;

    int16_t  line_num;   //行号
    int16_t  line_num_old ;
    int16_t  needle_range ; //针运行范围

    int16_t  mach_spd;
    int16_t  shax_spd;

    int16_t  sampleCnt;
    int16_t  spdSample;
    int16_t  spdH;
    int16_t  spdL;
    float  coefH;  // 线
    float  coefL;     //成圈

    float     kspd;

} MACHINE_DATA_TYPE;



typedef struct {
    int16_t  cntp;
    int16_t  cntn;
    int16_t  cntp_mt;
    int16_t  cntn_mt;

} POS_CNT_TYPE;


typedef struct {

    int32_t     pos_enc_add;
    float       pos_enc_add_cmd;

} GAN_POS_LOOP_CMD_TYPE;


extern GAN_POS_LOOP_CMD_TYPE    posloop;

extern POS_CNT_TYPE         poscnt;
extern MACHINE_DATA_TYPE       mach_spd;

extern ADC_CLA             Adc_gan_pos;
extern FILTER_TYPE_1       gFilter_gan_pos;



extern LEN_MEAS_TYPEDEF    len_meas;

// extern int16_t     gan_encoder[1000];

extern EEPROM_PAR      eep_par;
extern EEPROM_PAR2      eep_par2;

void gan_pos_var_init(void);
void gan_pos_pid_loop(void);
void gan_pos_pid_loop2(void);

void length_meas_cal(void);
void learn_angle_gan_encode(void);

void spd_test_run(void);
void power_son_logic(void);

void  cofe_calculate_handle (void);

ATTR_RAMFUNC  void print_encoder_per_can_rcv(void);
ATTR_RAMFUNC  void length_meas_motor_encode_add(void);
ATTR_RAMFUNC  void length_meas_motor_encode_clear(void);
ATTR_RAMFUNC  void length_meas_motor_encode_sample(void);

#endif /* MOT1_H_ */
