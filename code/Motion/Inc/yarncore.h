

#ifndef YARNCORE_H_
#define YARNCORE_H_

typedef struct {
	int32_t 	yarn_cmd_mm;
	int32_t 	res;
	int32_t	 	yarn_cmd_mm_old;
	int64_t		plus64;
	int64_t		plus64_old;
	int32_t  	plus_delt;

	int16_t     cmd_delt;
	int16_t     bj_delt;

	int32_t  	cmd_delt_sum;

} CAN_CMD_RCV_TYPEDEF;

typedef struct {
	int    		dlen;
	uint8_t  	data[8];
	// int32_t 	yarn_cmd_mm;
	// int32_t	 	yarn_cmd_mm_old;
} CAN_Data;
typedef struct {
	int16_t  delt_cmd_mm;
	int8_t   dir;
	int8_t   byte3;
	int32_t  byte4567;

	int8_t   dir_old;
	int8_t   clear_flag;
	int32_t  delt_cmd_plus;

	int32_t  cmd_mm_sum;

	int16_t  plus_err;
} CAN_DELT_CMD_TYPEDEF;

extern CAN_DELT_CMD_TYPEDEF    can_delt_cmd;


typedef struct {
	int8_t yarn_len;
	int8_t alarm;
	int8_t cmd;
	int8_t enable;
	int8_t mode;
	uint8_t cs_time_100us;
	int8_t maste_state;
	int8_t onebyone;
	int16_t cnt;
	int16_t cs_time_k;
	int8_t  dly_cnt;
	int8_t  dly_value;
	int8_t  loop_en;
	int32_t cnt32;
	int8_t  cnt8;
	int8_t  self_id;
	int8_t  first_id;
	int8_t  pre_id;

	int8_t  use_id_flag;
	int8_t  test_mode;
	int8_t  machine_mode;  // 0 run ; 1 -stop
	int8_t  mode_old;
	//------------------
	int8_t  pos_son;

} CAN_SEND_FLAG_TYPEDEF;

extern CAN_SEND_FLAG_TYPEDEF  	can_send_flag ;
extern CAN_CMD_RCV_TYPEDEF		can_pos_rcv;
extern CAN_Data    can_rcv;

typedef struct {
	int16_t stop_spd;
	int16_t acc_len_mm;
	int16_t   add_tor;
	int16_t   set_tor;

	int16_t acc_tor;

	int16_t tor_old;
	int8_t  acc_flag;
	int8_t  res;
	int32_t     mt_enc_add;  //
	float       mt_enc_add_mm;


} FUNC_PARA_TYPEDEF;

extern FUNC_PARA_TYPEDEF 	funcPar;


extern  int16_t yc_rundir ;
extern  int32_t yc_runspd ;
extern  int16_t yc_ndlpos ;
extern  int16_t yc_ndlPos_inc;

extern   int8_t  yc_start_flag ;
extern   int8_t  yc_stop_flag ;
extern   int8_t  yc_start_flag_old ;
extern   int8_t  yc_stop_flag_old ;


int YC_DoCmd(CAN_Data *pkg);

void yc_pos_judge_edge(void);

void TOR_ACC_POSITION_COMP(void);

#endif /* YARNCORE_H_ */
