

#ifndef MOT4_H_
#define MOT4_H_

typedef struct {
    int32_t     pos_err;
    int16_t    spdRpmFkNeg;
    int16_t    tor_cur ;
    int32_t     send402_len_mm;
    float       com_add_mm;
    float       tab_cur_mm;

} MONITOR2_TYPEDEF;

extern MONITOR2_TYPEDEF monitor;


typedef  struct {
    uint32_t    out;
    uint32_t    cur;
    uint32_t    old;
    uint32_t    inc;
    uint32_t    incsum;
    uint32_t    max;

} GAN_ABS_POS_TYPEDEF;



typedef  struct {

    float  Kp;
    float  Ki;
    float  Kd;


    float       ref;
    float       fbk;
    float       kspd_max_limit;

    float       kAng_max;
    int16_t     ang_max ;

    uint16_t    cmd_pos;
    uint16_t    cmd_pos_new;
    uint16_t    cmd_pos_orig;
    uint16_t    cur_pos;
    uint16_t    cur_pos_old;
    uint16_t    stop_pos;
    uint16_t    up_pos;
    uint16_t    low_pos;
    uint16_t    mid_pos;


    int16_t     pos_err;
    int16_t     err[10];
    int16_t     err2;

    int16_t     spd;
    int16_t     spdOut;
    int16_t     spdCmd;
    int16_t     spd2;
    int16_t     encInc;
    int16_t     encSum;
    int16_t     encVal;

    int16_t  spdUpposdMax;
    int16_t  spd_max_limit;
    int16_t  spd_min_limit;



    uint16_t  max_zone;
    uint16_t  min_zone;
    uint16_t  add_zone;

    uint16_t  kfilter;




    int16_t   spdOld;
    int16_t   spdInc;
    int16_t   spdIncOUt;
    int16_t   spdPool;
    int16_t   spdi;
    int8_t      upStop;
    int8_t      lowStop;
    int8_t      accFlag;
    int8_t      decFlag;
    uint8_t      cnt1;
    uint8_t      cmp1;
    uint16_t      cnt2;
    uint16_t      cmp2;

    uint16_t    ang_step  ;
    uint16_t    index  ;

    int16_t    a1  ;
    int16_t    a2  ;
    int16_t    a1sum;

    int16_t    j1  ;
    int16_t    jisum  ;

    uint16_t    powcnt;
    uint16_t    powstp;

    float  ferr;

    float  Up;              // Variable: Proportional output
    float  ui;              // Variable: Integral output
    float  OutPreSat;       // Variable: Pre-saturated output
    float  Umax;            // Parameter: Maximum output
    float  Umin;            // Parameter: Minimum output
    float  Out;             // Output: PID output
    float  SatErr;          // Variable: Saturated difference
    float  uKi;


    float  temp;


} GAN_POS_ABS_TYPEDEF;


extern FILTER_TYPE_1  filter_gan;
void gan_pos_pid_loop_abs(void);

void speed_forwd_cmd(void);
void monitor_handle(void);




#endif /* MOT1_H_ */
