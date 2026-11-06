#ifndef _YARN_FEED_H_
#define _YARN_FEED_H_
#include "math.h"
#include "mcdrv.h"
#include "param.h"
#define RUN_MODE_INIT (0)
#define RUN_MODE_ZERO (1)
#define RUN_MODE_SIDE (2)
#define RUN_MODE_FAULT (3)
#define RUN_MODE_RUN (4)
#define RUN_MODE_SYNC (5)
#define RUN_MODE_TEST (6)
#define RUN_MODE_POS_TO_TORQUE (7)
#define RUN_MODE_TORQUE (8)

#define MOT_MSG_NONE (0)
#define MOT_MSG_ZERO (1)
#define MOT_MSG_DEVID_MODE (2)
#define MOT_MSG_SEND_RANDID (3)
#define MOT_MSG_SELECT_DEV (4)
#define MOT_MSG_SET_DEVID (5)
#define MOT_MSG_FIND_SIDE (6)
#define MOT_MSG_ENABLE (7)
#define MOT_MSG_MOVE_POS (8)
#define MOT_MSG_MOVE_SPD (9)
#define MOT_MSG_MOVE_TORQ (10)
#define MOT_MSG_STOP (11)
#define MOT_SYNC (12)
#define MOT_SYNC_STOP (13)
#define MOT_SYNC_POS (14)
#define MOT_MSG_MOVE_POS_TO (15)
#define MOT_MSG_SCALAR_CTRL (16)
#define MOT_MSG_MOVE_POS_TO_TORQUE (17)
#define MOT_STATE_READY (0x01)
#define MOT_STATE_ZERO_FINISHED (0x02)

enum PID_CLASS{PID_SYNC=0,PID_NOMAL,PID_STOP} ;
#define MAX_POS_BUF 1000

#ifdef MOTORID
#define POS_SPD_SCALE (1)
#define ENCODER_DEMO (1)
#define MOTOR_DIR (1)
#define LIMIT_SPD (M1_N_NOM)
#define MIN_SPD (10 / g_fltM1speedAngularScale) //
#define MAX_SPD (450/g_fltM1speedAngularScale)
#else
#define POS_SPD_SCALE (1)
#define ENCODER_DEMO (32)
#define MOTOR_DIR (1)
#define LIMIT_SPD (M1_N_NOM)
#define MIN_SPD (10 / g_fltM1speedAngularScale) //
#endif

#define EXT_TO_IN_SCALE  0.03354f //65556 / 314 *7.0 = 0.1mm的脉冲数   脉冲数转0.1mm = 7 电机内部角度
#define MM_TO_MOTOR_POS  
#define MAX_GRAP_POS 1000.0f //100
typedef struct 
{
    float target_pos  ;
    int32_t cur_pos  ;
    uint16_t speed_limit ;
    uint8_t ctrl_flag ;
    float   set_pos ;
    float   torque_limit ;
    uint8_t ctrl_period ;
    uint8_t state ; 
    float   ext_to_in_scale ;
    uint8_t test_flag ; 
    float test_left_pos ;
    float test_right_pos ;
    float encoder_diameter_mm ; //编码器直径
    uint32_t encoder_circle_value ; //编码器一圈值
    float motor_encoder_10mm; //10mm 电机外部是多少个编码器
    float move_target_pos ;
    float move_target_cur_pos ;

}Mot_Ext_Pos_Ctrl_ST ;


typedef struct Mot_Move_Time_Param_Tag
{
	float   target_pos;
	float   cur_pos;
	float   begin_pos ;
	float   dis_pos ;
	uint8_t move_flag ;
	uint16_t time ; 
	uint8_t  dis_time ;
} Mot_Move_Time_Param;

typedef struct Mot_Move_Param_Tag
{
  float pos;
  float speed;
  float torque;
  
} Mot_Move_Param;

typedef struct Mot_Run_State_Tag
{
  uint8_t run_mode;
  uint8_t sub_mode;
  uint8_t motRun;
  uint8_t runDir;
  uint8_t zeroDir;
  int8_t encDir;   // dir between motor and outside(yarn feed)
  float zeroPos;   // unit of outside
  float zeroStPos; // unit of outside
  float zeroOffset;
  uint8_t repZero; // auto report after finding zero position
  int16_t nume;
  int16_t deno;
  float leftLmt;
  float rightLmt;
  float repUpPos; // auto report with moving up this position
  float repDnPos; // auto report with moving down this position
  uint32_t repStTime;
  uint32_t lightSpd;
  float maxSyncOff; // max offset in sync.
  int16_t ratioSyncOff;
  uint8_t syncDir;
  float syncPos;
  float syncOfsset;
  float masterPos; // sync dest pos
  float masterSpd;
  float syncErr;
  uint8_t state;
  uint8_t report_type;
  float torqueReq;
  float speedReq;
  float posReq;
  float target_pos_offset;
  uint8_t target_pos_ack_flag;
  uint16_t signal_report;
  uint8_t signal;
  uint8_t can_baund;
  uint32_t move_pos_begin; //
  uint32_t move_pos_dis;   //
  uint8_t zero_mode;       //
  uint8_t sensor_flag;
  float mot_zero_spd;
  uint8_t motor_type; //
  uint8_t dir_protocol;
  int16_t nume2;
  int16_t deno2;
  uint8_t zero_flag;
  uint8_t arr_pos_flag;
  uint8_t change_pid_flag;
  uint32_t change_pid_begin_tick;
  uint8_t torque_check_over_spd_send_flag;
  int32_t pos_target;
  int32_t zero_set_pos;
  uint8_t stop_flag;
  float_t punching_speed;
  float_t io_fun_set;
  uint8_t punching_state;
  uint16_t punching_max_current;
  uint32_t punching_cnt;
  Mot_Move_Time_Param mot_move_time_param ;
  Mot_Move_Param mot_pos_to_torque_st ;
  uint16_t io_light_spd ;//ms
  uint8_t  ack_cmd ;	
  uint32_t motor_dir_rev ;
  float_t  can_baund2 ;
  int32_t ext_pos ;
  uint8_t  fun_test_flag ;
  uint16_t  pos_buf_ptr;
  uint16_t  pos_zero_ptr ;
  uint8_t pos_over_zero_flag ;
  uint8_t pos_over_zero_report_enable ;
  uint16_t pos_buf_temp_ptr; 
  uint32_t pos_buf_time ; //现有的时间
  float ext_move_slow_speed ;//外部移动慢档的速度 
  float ext_move_fast_speed ;//外部移动快档的速度 
  float    pos_buf_clib_scale ; //校准的比例
  uint8_t  ext_ctrl_pos_enable ; //外部控制标记
  uint8_t  ext_ctrl_pos_level ; //外部控制 速度档位 1慢档 2快档
  float    speed_left_pos ;
  float    speed_right_pos ;
} Mot_Run_State;
typedef struct Motor_RUN_Tag
{
  float posAcc;
  float posDec;
  float posMaxJerk;
  float posMinJerk;
  float spdAcc;
  float spdJerk;
  float posPip;
  float inertia;
  float spdBW;
  float spdILmt;
  float IBW;
  float ILmt;
  float VFF;
  float noWait;
  float JerkMulti;
  float hwLmtIo;
  float spdPidP;
  float spdPidI;
  float zero_offset;
  float spdPidP1;
  float spdPidI1;
  float posPip1;
  float over_load_scale;
  float over_load_scale1;
  float ramp_type ; 
} Motor_RUN;

typedef struct Motor_tag
{
  float pp;
  float rs;
  float ld;
  float lq;
  float ke;
  float Iph_nom;
  float Uph_nom;
  float N_nom;
  float I_max;
  float UDCB_max;
  float UDCB_trip;
  float UDCB_under;
  float UDCB_over;
  float N_over;
  float N_min;
  float N_max;
  float E_block;
  float E_block_pre;
  float dir;        // 0  1
  float gear_ratio; // 1
  float torque_over_speed;
  float max_over_load_scale;
  float max_over_load_scale1 ;
  float over_load_time[over_load_groups];       // 
  float over_load_thresshold[over_load_groups]; //
  float over_load_cnt[over_load_groups];        //
  float dis_over_load;                          //
  float speed_loop_rpm01;                       //
} Motor;
typedef struct Mot_Zero_Param_Tag
{
  float speed;
  float torque;

} Mot_Zero_Param;

typedef struct Mot_Sync_Param_Tag
{
  uint8_t dir;
  int32_t offset;
  int32_t syncPos;
} Mot_Sync_Param;


typedef struct Dev_Id_Setting_Tag
{
  uint32_t rand;
  uint32_t motId;
} Dev_Id_Setting;

typedef struct Dev_Select_Tag
{
  uint32_t rand;
  uint8_t sel;
} Dev_Select;

typedef union Mot_Msg_Param_Tag
{
  Mot_Zero_Param motZero;
  Mot_Move_Param motMove;
  Dev_Id_Setting devIdSetting;
  Dev_Select devSelect;
  uint8_t motEnable;
  uint8_t devIdMode;
  Mot_Sync_Param motSync;

} Mot_Msg_Param;
typedef enum
{
	MOTOR_NOMAL_TYPE =0,//正常类型
	MOTOR_MAIN_TYPE =1 ,//主类型
	MOTOR_SLAVE_TYPE =2 ,//从类型
}Motor_Sr_type ;
typedef struct 
{
	Motor_Sr_type sr_type ; //收发类型 0表示正常模式 1表示 主机模式自动发送 2表示接受模式接收主发送ID 
	uint8_t send_enable ; //自动发送使能
	uint16_t send_period_ms ; //自动发送周期
	int32_t  resv_pos ; //接收到的外部位置数据
	uint16_t sr_canid ; //自动收发的CANID
	uint8_t  sensor_port ; //传感器接入的PORT 
	int32_t  offset_pos ; //主从电机偏移量
}Mot_Auto_SR_Pos_T;

extern Motor_RUN motor_run;
extern Motor motor;
extern uint16_t mot_id;
extern Mot_Run_State mot_run_state;

extern Mot_Ext_Pos_Ctrl_ST mot_ext_pos_ctrl_st;

void initMotor(void);
float torqIn2Out(float fTorq);
float torqOut2In(float fTorq);
float spdIn2Out(uint8_t mot_msg, float fSpd);
float spdOut2In(uint8_t mot_msg, float fSpd);
float posIn2Out(float fPos);
float posOut2In(float fPos);
float posRatioIn2Out(float fPos);
float posRatioOut2In(float fPos);
void setCurPos(float fPos);
float getCurPos(void);
float getCurTorq(uint8_t type);
float getCurSpd(uint8_t type);
void motSetMasterPos(float pos);
void motRunSM(uint32_t msg, Mot_Msg_Param *param);
void motAutoRunTestSM(void);

void moveTorq(float fTorq);
void moveSpd(float fSpd, float fTorq);
void movePos(float fPos, float fSpd, float fTorq);
void moveSRampPos(float fPos, float fSpd, float fTorq) ;
void moveStop(uint8_t emerg);
void moveResume(void);

void syncActionSM(void);
void motEncCalibrate(void);
void motSysParamApply(void);

void motEncCalibrateExt(void); //

void motEncClibrateStop(void); //

void motEncClibrateData(uint16_t ext_encode, uint16_t in_encode); //

void motEncClibrateSaveData(void); //

float posRatio2In2Out(float fPos);
void enableMotor(uint8_t enb);
void brakeManual(uint8_t release, uint8_t duty_pct, uint8_t freq_khz);   //手动刹车覆盖(CAN 0x43): release=1松闸/0抱闸, duty_pct=保持占空比%(0=默认50), freq_khz=频率kHz(0=默认10k,1~20), 会取消排队中的启停时序
void brake_open_seq(uint32_t freq_hz, uint8_t duty_pct); //松闸序列(强励磁->保持): 启停/0x43/0x45 统一入口

float posRatio2Out2In(float fPos);
void mot_task_ms_loop(void); //1ms周期

void io_ctrl_set(uint8_t io_idx,uint8_t type,uint16_t lightms) ;

void servo_Auto_SR_Set_Period(uint16_t per) ; //自动收发周期

void servo_Auto_SR_Set_Type(uint8_t type); //自动收发 类型
void servo_Auto_SR_Enable(uint8_t en); //自动收发 使能
void servo_Auto_SR_Set_CanID(uint16_t canid); //自动收发 CANID
void servo_Auto_SR_Set_Sensor_Port(uint8_t port); //自动收发 传感器接入 PORT
void servo_Auto_SR_Set_offset(int32_t offset) ; //自动收发 主从位置偏移
uint16_t servo_Auto_SR_Get_CanID(void) ;
void servo_Auto_SR_Set_Resv_Pos(int32_t pos) ;
Motor_Sr_type servo_Auto_SR_Get_Type(void) ;
void servo_Auto_SR_Slave_ctrl(void) ;// 100us

void motOpenSpd(void);
int moveIsStop(uint32_t period, uint32_t tryTime, uint8_t resh);
uint8_t motTorqueOverSpd(void);
uint8_t motGetMovePosArrFlag(void);
void motSetGearRatio(int16_t demo, int16_t nume); //

void update_target_pos(float fPos);
void motor_update_speed(float fSpd); // 速度模式和位置模式下 实时更新速度
int moveIsLock(uint32_t period, uint32_t tryTime);
int moveIsStop1(uint32_t period, uint32_t tryTime,uint8_t resh);
void motHalfCurrentLock(void);
void motChangePid(uint8_t type);
void initMotorPos(void);
void saveZeroPos(void);
void mot_run_circle(void);
void motUpdateSpeedPidI(float fltIntegral);

int motorIsStop(uint32_t period, uint32_t tryTime, uint8_t resh); // 判断电机是否停止
void Servo_PID_Change(u8 cs) ;

void mot_set_ext_pos(int32_t pos);
int32_t mot_get_ext_pos(void);
void servo_sync_clib_time(uint32_t time) ;//校准同步时间

uint32_t servo_sync_get_clib_time(void);

void servo_ext_pos_ctrl_set(uint16_t speed,int32_t pos);//外部控制设置
void servo_ext_pos_ctrl_loop(void);//外部控制loop
void servo_ext_pos_ctrl_init(void) ;
void servo_set_ext_ctrl_enable(uint8_t en,uint8_t level,uint8_t canid) ;
void servo_ext_update_ext_to_in_scale(void);
void servo_sync_buf_zero(void) ;
uint16_t servo_get_sync_pos_ptr(void);
void servo_ext_open_read_encode(void);//打开外部编码器读取
#endif
