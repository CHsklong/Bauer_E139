/*
 * UserMCSwPort.h
 *
 *  Created on: 2016年4月20日
 *      Author: admin
 */

#ifndef USERMCSWPORT_H_
#define USERMCSWPORT_H_



/*********************************************************************************************/
#define SERVO_MODE_POS 0
#define SERVO_MODE_SPD 1
#define SERVO_MODE_TOR 2
#define SERVO_MODE_P_S 3
#define SERVO_MODE_S_T 4
#define SERVO_MODE_P_T 5
/*********************************************************************************************/

/*********************************************************************************************/
enum{
	ATOR_ID =0,
	ASPD_ID
};
/*********************************************************************************************/

/*********************************************************************************************/
//输入端口
struct MC_SW_INPORT_BIT {
	u32 res1:1;		//						bit 0
	u32 SON:1;		//使能     				bit 1
	u32 INHP:1;		//脉冲禁止					bit 2	
	u32 CCLR:1;		//脉冲清零					bit 3	
	u32 CMDINV:1;	//指令方向取反				bit 4
	u32 ZCLAMP:1;	//零速箝位					bit 5
	u32 GNUM:1;		//齿轮比切换					bit 6
	u32 POSTRG:1;	//位置内部触发				bit 7		
	u32 STOP:1;		//电机停止					bit 8
	u32 EMGS:1;		//急停并报警					bit 9
	u32 POS_LMT:1;	//正限位					bit 10
	u32 NPOS_LMT:1;	//负限位					bit 11
	u32 ARST:1;		//故障清除					bit 12
	u32 S_P:1;		//速度位置模式切换				bit 13
	u32 S_T:1;		//速度扭矩模式切换				bit 14
	u32 T_P:1;		//扭矩位置模式切换				bit 15	
	u32 SPDCH:2;	//内部速度切换				bit 16 17 
	u32 TCMCH:2;	//内部扭矩切换				bit 18 19
	u32 Gorg:1;		//回零点					bit 20
//u32 GAINUP:2;	//增益切换					bit 10 11


//	u16 res2:3;
};
#define INPUT_FUN_NUM 20
/*********************************************************************************************/

/*********************************************************************************************/

union MC_SW_INPORT{
	u32 All;
	struct MC_SW_INPORT_BIT bit;
};

/*********************************************************************************************/

/*********************************************************************************************/
//输出端口
struct MC_SW_OUTPORT_BIT {
	u32 Null:1;		//bit 0//NULL 			
	u32 SERR:1;		//bit 1//故障				
	u32 SBRK:1;		//bit 2//抱闸输出		
	u32 SON:1;		//bit 3//使能				
	u32 SRDY:1;		//bit 4//准备好			
	u32 SINP:1;		//bit 5//定位完成		
	u32 TSPD:1;		//bit 6//速度跟踪完成	
	u32 TAT:1;		//bit 7//转矩到达		
	u32 ZSPDF:1;	//bit 8//零速				
	u32 SPD_OK:1;	//bit 9//速度到达		
	u32 TQL:1;		//bit 10//扭矩限制中	
	u32 BRKR:1;		//bit 11//制动				
	u32 FAN:1;		//bit 12//风扇				
	u32 SZFLG:1;	//bit 13//编码器零点	
	u32 HZFLG:1;	//bit 14//编码器零点	/////-------------
	u32 WARN:1;		//bit 15//警告输出		
	u32 OLW:1;		//bit 16//过负载			

//	u32 res:17;
};
//#define OUTPUT_FUN_HZFLG 16
#define OUTPUT_FUN_NUM 16
/*********************************************************************************************/

/*********************************************************************************************/
union MC_SW_OUTPORT{
	u32 All;
	struct MC_SW_OUTPORT_BIT bit;
};
/*********************************************************************************************/

/*********************************************************************************************/
typedef struct{
	union MC_SW_INPORT In;
	union MC_SW_OUTPORT Out;
	float PosGvn;
	float SpdGvn;
	float TorGvn;
	float uKpp;
	float uKvp;
	float uKvi;
}MC_SWPORT_TYPEDEF;
/*********************************************************************************************/

extern MC_SWPORT_TYPEDEF McSwPort;
extern AdcChannelTypeDef AngTorRef[2];

void MC_ANG_SWPORT_INIT(void);
void McSwPortRead(void);
void McSwPortLoad(void);
void McAngPortRead(void);
void McGainHandle(void);
void SwOutPortLogic(void);
#endif /* USERMCSWPORT_H_ */
