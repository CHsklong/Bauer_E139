

#ifndef MOT3_H_
#define MOT3_H_



typedef struct {
	int8_t  flag;  //学习标志
	int8_t  sts;

	int8_t  trig;
	int8_t  stop;
	int8_t  firstLock;
	int8_t  mode;


	int16_t  i_mid;
	int16_t  i_max;

	int16_t  i;
	int16_t  i_old;
	int16_t spd;
	int16_t spd_set;

	int16_t tab_delt_plus;


	int16_t dly;
	int16_t cnt;
	int16_t delt;
	int16_t enc_plus16;

	int32_t enc_plus_add;
	int32_t min;
	int32_t max;
	int32_t pos_1000;
	int32_t pos_cur;
	int32_t pos_cur_old;
	int32_t pos_res;
	int32_t yarn_length_plus;
	int32_t yarn_res_plus;
	int32_t tab_cur_plus;

	float 	len_mm;
	float	k;
	float  enc_mm;
	float   yarn_length_mm;
	float  tab_cur_mm;



} LEARN_TABLE_TYPEDEF;

extern LEARN_TABLE_TYPEDEF learn_tab;

void learn_tab_init(void);
void learn_ganAngle_handle(void);
void motor_encoder_add_handle(void);
void caculate_gan_yarn_lenght(void);
void caculate_gan_cur_pos2motor_pos_handle(void);

void gan_goto_zero_handle(void);
void gan_goto_postion_handle(float fpos_mm);



#endif

