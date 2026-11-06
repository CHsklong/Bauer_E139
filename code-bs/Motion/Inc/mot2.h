
#ifndef MOT2_H_
#define MOT2_H_

#define QUAN_CHENG_XING     1
#define PU_TONG_JI			2
#define MACHINE_TYPE   PU_TONG_JI

#define DIR_RIGHT     1
#define DIR_LEFT      2

#define DIR_ADD     1
#define DIR_DEC     2

#define GAN_TOR_MIN  	500  //700
#define GAN_TOR_MID		750
#define GAN_TOR_MAX		2000  //950

#define JT_SAVE_NUM		1200  //1024  //900

// #define SWCESHI_MODE    1

typedef struct {
	int16_t 	cur_ndl_pos ;		//针位置
	int16_t  	zone_min;
	int16_t  	zone_max;
	int16_t   	zone_mid;
	int16_t  	yarn_num;

	int16_t  	learn_pos_min;
	int16_t  	learn_pos_max;

	int16_t  	start_ndl_pos;
	int16_t  	end_ndl_pos;

	int8_t  	dir;
	int8_t  	dir_old;
	int8_t  	cali_enable;
	int8_t  	go_zero_trig;
	int8_t   	go_zero_trig_old;
	int8_t  	init_flag;
	int8_t  	add_dir_stop;
	int8_t  	add_dir_stop_old;
	int8_t      dec_dir_stop;
	int8_t      dec_dir_stop_old;


	int16_t  	yarn_len_big;
	int16_t  	yarn_len_less;

	int16_t     lead ;
	int16_t     accstep;
	int16_t  	avgstep;
	int16_t   	decstep;

	int16_t  	curstep;
	int16_t     oldstep;
	int16_t  	cnt;
	int16_t     index;
	int16_t     i;
	int16_t     offset_start ;
	int16_t  	offset_stop;


	float		meas_lens;
	float  		actual_lens;
	float		actual_dia;
	float		kadd ;
	float		kdec;

	float		acc;
	float		avg;

	float  		add_value ;
	float 		dec_value ;

	float		sv_add[JT_SAVE_NUM];
	float		sv_dec[JT_SAVE_NUM];
	uint8_t     time_add[JT_SAVE_NUM];
	uint8_t		time_dec[JT_SAVE_NUM];

	int8_t      delt_add[JT_SAVE_NUM];
	int8_t  	delt_dec[JT_SAVE_NUM];

	int16_t  	delt_add_show;
	int16_t  	delt_dec_show;

} YARN_POS_CMD;

typedef struct {

	int8_t  	self_run;
	int8_t  	zbuflag;
	int8_t   	fbuflag;
	int8_t   	res;
	int32_t  	plus_delt;
	int32_t 	yarn_cmd_mm;

	int64_t   	plus64;
	int64_t		plus64_old;

	int64_t     plus_cmd_preg_sum;


	int16_t 	delt_mm;   //err
	int16_t     cmd_moter_mm;
} SELF_POS_CMD_TYPEDEF;
//---------------------------------------------
typedef struct {
	int16_t  min;
	int16_t  max;
	int16_t  add;
	int16_t  out;

	int16_t  learn_dumu;
	int16_t  cur_dumu;
	int8_t   mode_hua;
	int8_t   mode_xian;
	int8_t   mode_zhenban;
	int8_t   res;

	float   k;
	float   kuse;

	int16_t kp_add;

} COMPENSAT_PULS_TYPEDEF;
extern COMPENSAT_PULS_TYPEDEF comp_plus;
//----------------------------------------------
typedef struct {
	uint16_t	min;
	uint16_t 	mid;
	uint16_t 	max;
	uint16_t  	cmd;
	uint16_t    delt;

	int16_t     cur_ndl;
	int16_t  	start_ndl;
	int16_t  	stop_ndl;

} BAI_GAN_TOR_TYPEDEF;

extern BAI_GAN_TOR_TYPEDEF  gan_tor;

extern SELF_POS_CMD_TYPEDEF  self_cmd;

extern YARN_POS_CMD  yarn_pos;
//---------------------------------------------------------
typedef struct {
	int32_t  	rcv_plus;
	int32_t  	can_res;

	int32_t  	rcv_plus_old;
	int32_t  	learn_min;
	int32_t  	learn_max;

	int16_t     plus_delt;
	int16_t     ndl_cur;    //当前针位置

	int8_t     	dir;
	int8_t  	dir_old;

	float     kp2ndl;


} JT_LEARN_TYPEDEF;

extern JT_LEARN_TYPEDEF jt_learn;
void jt_pos_learn_handle(void);
void jt_pos_init(void);

//---------------------------------------------

void gan_tor_init(void);
void gan_tor_cmd_handle(void);


void yarn_pos_init(void);
void yarn_len_cmd_handle(void);
void self_learn_handle(void);
void dia_calibration_handle(void);
void yarn_learn_date_avg_handle(void);
void yarn_learn_date_avg_handle2(void);
void yarn_learn_date_avg_handle3(void);
void yarn_learn_date_avg_handle4(void);
void yarn_learn_date_avg_handle5(void);

void stop_postion_judge_handle(void);

int16_t gan_pos_buchang_handle(void);
int16_t gan_pos_buchang_handle2(void);

#endif
