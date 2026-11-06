/*
 * UserMcApi.h
 *
 *  Created on: 2015-2-5
 *      Author: Administrator
 */

#ifndef USERHDPORT_H_
#define USERHDPORT_H_

#include "mcu.h"


#define CPLD_RD 0
#define CPLD_WR	!CPLD_RD



/*******************************************************************/
//
struct MC_HD_INPORT_BIT {
	u32 InPort1:1;	//1
	u32 InPort2:1;	//2
	u32 InPort3:1;	//3
	u32 InPort4:1;	//4
	u32 InPort5:1;	//5
	u32 InPort6:1;	//6
	u32 InPort7:1;	//7
	u32 InPort8:1;	//8
	u32 InPort9:1;	//9
	u32 QepUVW:3;		//UVW
	u32 ErrABZ:3;		//ABZ
	u32 ErrUVW:3;		//UVW	
	u32 IBOV:1;			//ipm
	u32 BrkOV:1;		//
	u32 IFOV:2;			//

	u32 res:10 ;
};
union MC_HD_INPORT{
	u32 All;
	struct MC_HD_INPORT_BIT bit;
};

/*******************************************************************/
//
struct MC_HD_OUTPORT_BIT {
	u32 OutPort1:1;		//1
	u32 OutPort2:1;		//2
	u32 OutPort3:1;		//3
	u32 OutPort4:1;		//4
	u32 OutPort5:1;		//5
	u32 OutPort6:1;		//6
	u32 EncPwrPort:1;	//
	u32 BusEnPort:1;	//245
	u32 BreakPort:1;	//
	u32 StPort:1;			//
	u32 FanPort:1;		//
	u32 Out1Zflag:1;	//Z
	u32 Out2Zflag:1;	//Z
	u32 Out3Zflag:1;	//Z
	u32 Out4Zflag:1;	//Z
	u32 Out5Zflag:1;	//Z
	u32 res:16;
};

union MC_HD_OUTPORT{
	u32 All;
	struct MC_HD_OUTPORT_BIT bit;
};
/*******************************************************************/


/*******************************************************************/
struct MC_HD_CpldInPort_BIT{
	u32 QepUVW:3;//bit0 1 2
	u32 ErrABZ:3;//bit3 4 5
	u32 ExInp1:1;//bit6
	u32 ExInp2:1;//bit7
	u32 ExInp3:1;//bit8
	u32 ExInp4:1;//bit9
	u32 ExInp5:1;//bit10
	u32 ExInp6:1;//bit11
	u32 ExInp7:1;//bit12
	u32 ExPulseReg:10;//bit13 22
	u32 ExInRes:1;//23

};

union MC_HD_CpldInPORT{
	u32 All;
	struct MC_HD_CpldInPort_BIT bit;
};
/*******************************************************************/

/*******************************************************************/
struct MC_HD_CpldOutPort_BIT{
	u32 ExOut1:1;		//bit0
	u32 ExOut2:1;		//bit1
	u32 ExOut3:1;		//bit2
	u32 ExOut4:1;		//bit3
	u32 ExOut5:1;		//bit4
	u32 PulseCmd:2;		//bit5 6
	u32 QepDivPsc:7;	//bit 7 8 9 10 11 12 13
	u32 QepDirCmd:1;	//bit14
	u32 InFilter:3;		//bit15 16 17
	u32 PulsFilter:4;	//bit18 19 20 21
	u32 ExOutRes:2;		//bit22 23
	u32 Out1Zflag:1;	//Z 24
	u32 Out2Zflag:1;	//Z 25
	u32 Out3Zflag:1;	//Z 26
	u32 Out4Zflag:1;	//Z 27
	u32 Out5Zflag:1;	//Z 28
};

union MC_HD_CpldOutPORT{
	u32 All;
	struct MC_HD_CpldOutPort_BIT bit;
};
/*******************************************************************/


typedef struct {
	u16 Index;				//
	s32 EncoderPos;		//
	u16 EncoderQD;		//
	u16 EncoderCapQD;	// 
	s16 ExPulse;		//
	s16 ExPulseBk;		//
	u16 CurU;					//U
	u16 CurV;					//V
	u16 Vbus;					//
	u16 Temp;					//
	u16 zl_pos;					//张力位置
	u16 zl_cur;					//张力电流
        u16 Ref1;
        u16 Ref2;
	u16 DrvInfo;			//
	u16 PwmCmp1;			//svpwm 1
	u16 PwmCmp2;			//svpwm 2
	u16 PwmCmp3;			//svpwm 3
	u16 PwnEn;
	union MC_HD_INPORT 		MCInport;
	union MC_HD_OUTPORT 	MCOutport;
	union MC_HD_CpldInPORT 	McCpldIn;
	union MC_HD_CpldOutPORT McCpldOut;	
}MC_HD_PORT;
/*******************************************************************/


extern MC_HD_PORT MC_HD_Port;
/********************************************************************************************************************/

/********************************************************************************************************************/
//
typedef struct{
	u16 Index;
	s32 QepData;
	s32 Reg;
	s32 RegBk;
	s32 RegInc;

        s32 hdReg;
        s32 hdRegBk;
        s32 hdRegInc;

	
	s32 QepLine;
	s32 QepCircle;

	u16 ZFlag;
	u16 UpFlag;
	u16 DnFlag;
	
	u16 SynFlag;
	s32 ZSynVar;
	u16 ZTrgReg;

}QepRegVarTypeDef; 

extern QepRegVarTypeDef QepVar;

void DrvQepInit(void);
void DrvQepRegSnap(void);
void DrvQepZTrgIsrHandle(void);
/********************************************************************************************************************/

/********************************************************************************************************************/
//

#define ERROR_NC 0
#define NO_TEMP_SEN_ERR 1
#define HIGH_TEMP_ERR 2
#define HIGH_TEMP_WARN 3

typedef struct
{
	Uint16 gRTempAdc_value;
	Uint32 gRTempSum;
	Uint16 gRTemp_index;
	Uint16 gRTemp_resault;
	Uint16 gRTemp_C;
	Uint16 gRTemp_FanOpen_Thd;
	Uint16 gRTemp_FanClose_Thd;
	Uint16 gRTemp_HT_Warning_Thd;
	Uint16 gRTemp_HT_Err_Thd;
	Uint16 gRTemp_sts;
	Uint16 gFan_sts;
}MC_HD_TempSampleTypeDef;
extern MC_HD_TempSampleTypeDef HD_Temp;

void MC_HD_TempSample(MC_HD_TempSampleTypeDef *dp);


#define RADITOR_TEMP_OPENFAN_THD 50
#define RADITOR_TEMP_CLOSEFAN_THD 45
#define RADITOR_TEMP_ERR_THD 75
#define RADITOR_TEMP_WARNING_THD 68

#define MC_HD_RADITORTEMP_INIT {0,\
								0,\
								0,\
								0,\
								0,\
								RADITOR_TEMP_OPENFAN_THD,\
								RADITOR_TEMP_CLOSEFAN_THD,\
								RADITOR_TEMP_WARNING_THD,\
								RADITOR_TEMP_ERR_THD,\
								0,\
								0}

///////////////////////////////////////////////////////////////////////////////////////////////
//
#define SERVO_RADITOR_TEMP_DISGREE_NOSEN 3800
#define SERVO_RADITOR_TEMP_DISGREE_20TO41 2680
#define SERVO_RADITOR_TEMP_DISGREE_42TO49 1769
#define SERVO_RADITOR_TEMP_DISGREE_50TO62 1466
#define SERVO_RADITOR_TEMP_DISGREE_63TO78 1050
#define SERVO_RADITOR_TEMP_DISGREE_79TO90 680
#define SERVO_RADITOR_TEMP_DISGREE_90TO99 492
#define SERVO_RADITOR_TEMP_DISGREE_100    375

#define SERVO_RADITOR_TEMP_SCALE_20TO41 0.02392
#define SERVO_RADITOR_TEMP_SCALE_42TO49 0.02632
#define SERVO_RADITOR_TEMP_SCALE_50TO62 0.03125
#define SERVO_RADITOR_TEMP_SCALE_63TO78 0.04167
#define SERVO_RADITOR_TEMP_SCALE_79TO90 0.06250
#define SERVO_RADITOR_TEMP_SCALE_90TO99 0.08333
///////////////////////////////////////////////////////////////////////////////////////////////
/********************************************************************************************************************/

void MC_HD_PortInit(void);
void MC_HD_Port_Read(void);
void MC_HD_Ang_Port_Read(void);
void MC_HD_Port_Load(void);
void MC_HD_Cpld_Com(void);
void EncUVWLineLearn(void);
#endif /* USERMCAPI_H_ */
