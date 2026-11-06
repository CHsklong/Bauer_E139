

#ifndef TOR_H_
#define TOR_H_


#define  MAX_DUTY   300

typedef  struct {
    int16_t  a1;
    int16_t  a2;
    float    f1;
    float    f2;
    float    kb;
    float    kg;

    float    fx;
    int16_t  ax;

    float    fax;
} ZL_CALI_TYPEDEF;


typedef  struct {

    float  Kp;
    float  Ki;
    float  ref;
    float  out_old;
    float  ref_k;
    float  out;


    uint16_t  max_duty;


} ZL_TOR_TYPEDEF;



extern ADC_CLA              Adc_ZL;

extern FILTER_TYPE_1       gFilterZL;
extern ZL_TOR_TYPEDEF      tor_z;

extern ZL_CALI_TYPEDEF     zl_cali;




void zl_var_init(void);
void zl_current_loop(void);

void zl_cali_init(void);
void zl_cali_handle(void);
int16_t zl_fx_2_ax_handle(float fx);
void zl_cali_save_handle(void);


#endif /* TOR_H_ */
