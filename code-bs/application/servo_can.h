#ifndef _EASY_CAN_H_
#define _EASY_CAN_H_

#define YARN_FEED     (1)
#define DEBUG_CMD  1
//little-endian
typedef struct {
	uint32_t id;
	uint8_t dlc;
	uint8_t data[8];
}CAN_FRAME_t;

#define GET_WORD(iN1, iN2)   (((iN1))|((iN2)<<8))    
#define GET_DWORD(iN1, iN2, iN3, iN4)  (((iN1))|((iN2)<<8)|((iN3)<<16)|((iN4)<<24))   
#define SET_WORD(iN, oUT1, oUT2)  do{(oUT1) = ((iN)&0xff); (oUT2) = (((iN)>>8)&0xff);}while(0)     
#define SET_DWORD(iN, oUT1, oUT2, oUT3, oUT4)  do{(oUT1) = ((iN)&0xff); (oUT2) = (((iN)>>8)&0xff); (oUT3) = (((iN)>>16)&0xff); (oUT4) = (((iN)>>24)&0xff);}while(0)   

#define CAN_BANDING_RECV_ID (0x308)
#define CAN_BANDING_SEND_ID (0x608)
//----------CAN command-------------------
#define CMD_SERIAL        (0x60)
#define CMD_OSC           (0x61)

//----------CAN SERVO-----------------
#define CMD_ZERO                (0x01)   // goto zero position
#define CMD_WR_PARAM            (0x02)   // write parameter

#define CMD_SYNC_CW             (0x03)   // synchronize cw
#define CMD_SYNC_CCW            (0x04)   // synchronize ccw
#define CMD_SYNC_STOP           (0X05)   // stop synchronized
#define CMD_MOVE_POS_TO         (0X06)   // move to abslute position
#define CMD_ENABLE              (0x07)   // enable/disable motor
#define CMD_CLR_ALERT           (0x08)   // clear all alerts
#define CMD_CLIB_ZERO           (0x09)   // motor calibration
#define CMD_MOVE_POS            (0x0a)   // move to inc postion in position mode
#define CMD_MOVE_SPD            (0x0b)   // move in speed mode
#define CMD_MOVE_TORQ           (0x0c)   // move in torque mode
#define CMD_MOVE_POS_ACK        (0x8a)   // 
#define CMD_MOVE_TORQ_CHANGE    (0x8C)   // 
#define CDM_MOVE_CIRCLE			(0x8E)
#define CMD_MOVE_POS_TIME		(0x8D)   
#define CMD_MOVE_POS_SPD_TORQ   (0x0d)   // move in position mode with limit speed and torque
#define CMD_MOVE_POS_SPD        (0x0e)   // move in position mode with limit speed.
#define CMD_STOP                (0x11)   // stop move.
#define CMD_WR_PARAM_LIM        (0x12)   // set limit parameter
#define CMD_AUTO_ANSWER         (0x13)   // auto response state and alert
#define CMD_MOVE_POS_ACK_REPORT (0x15)
#define CMD_SERVO_MOVE_SYNC_CW   (0x22)
#define CMD_SERVO_MOVE_SYNC_CCW  (0x27)

#define CMD_MOVE_TO_POS_TO_TQRD (0x2d)

#define CMD_TAKE_DEBUG_PRINT         (0x61)   // 打印变量
#define CMD_TAKE_DEBUG_BEGING   (0x60)   // 打印开始
#define CMD_TAKE_DEBUG_END      (0x62)   //打印结束
#define CMD_EXT_POS_CTRL        (0x6A)
#define CMD_RD_STATUS           (0x71)   // read state
#define CMD_RD_PARAM            (0x72)   // read parameter
#define CMD_RD_TORQ             (0x76)   // read torque
#define CMD_RD_SPD              (0x75)   // read spd
#define CMD_COND_RESP           (0x78)   // response conditions of satisfaction
#define CMD_FIND_DIST           (0x79)   // find side
#define CMD_ZHISHAN_READ_VER    (0x7B)   //兼容芝山的读版本号接口
//set device id
#define CMD_SCALAR_CTRL         (0x8f)   //开环速度控制
#define CMD_ID_MODE             (0x50)   // enter/exit deivce id setting.
#define CMD_ID_RESP             (0x51)   // request rand number
#define CMD_SEL_DEVICE          (0x52)   // select device（500ms）
#define CMD_WR_ID               (0x53)   // set new device id.
// programing 
#define CMD_READ_ALARM_STR (0xD6)
#define CMD_ENCRYPT_KEY         (0x55) //发送加密秘钥
// programing 


#define CMD_PROG_START          (0xE1)   // start program
#define CMD_RPOG_ADDR           (0xE2)   // start address of program
#define CMD_PROG_DATA4          (0xE3)   // 4byte data of program
#define CMD_PROG_DATA2          (0xE4)   // 2byte data of program
#define CMD_PROG_END            (0xE5)   // end of program

#define CMD_RD_DEV_ID           (0xE6)   // get deivce id
#define CMD_RD_DEV_VER          (0xE7)   // get version of program


#define CMD_ALARM_REPORT        (0xF0)   // 
#define CMD_TORQ_OVER_SPD_MSG   (0xF1)   //
#define CAN_SEND_ID             (0x100+mot_id)
#define CAN_DEBUG_ID            (0x105)
#define CAN_RECEIVE_ID          (0x1)
#define CAN_SYNC_ID             (0x100)

#define CAN_SEND_MBOX_NUM       (4)
#define CAN_REC_MBOX_NUM        (15)
#define CAN_SYNC_MBOX_NUM       (10)

#define CMD_READ_ENC_ORIG       (0xD0)  //
#define CMD_READ_ENC_CLIB       (0xD1)   //


#define CMD_CALIB_ENC           	(0xE8)   // 校准编码器
#define CMD_CALIB_ENC_STEPS     	(0xE9)   // 校准走一步记录下位置值
#define CMD_CALIB_ENC_END       	(0xEA)   // 校准结束 
#define CMD_CALIB_ENC_EXTERN_CTRL (0xEB) //作为工装程序用
#define CMD_CALIB_ENC_BEGIN     	(0xEC)   //校准开始 并重启 重新设置6570的编码器值
#define CMD_RESET               	(0xED)   //软件重启
#define CMD_CLEAR_ENC_CLIB_DATA  	(0XEE)  //清除校准数据
//#define CMD_CALIB_READ          	(0XEF)

#define CMD_CLIB_READ_ENCODE      (0xAB) //读原始编码值
#define CMD_CLIB_CTRL             (0xAD) //校准操作
#define CMD_DEBUG_PRINT           (0xFD) //打印数据
#define SERVO_GET_ALERT_INFO     (0xd6)
#define SERVO_DEV_INFO           (0xd2)
#define SERVO_DEV_ID             (0xd4)
#define SERVO_DEV_ASK            (0xef)


#define CMD_ROUND_RUN_SET_CHANGE_DIR_DATA  (0xa1) //设置换向数据
#define CMD_ROUND_RUN_REPORT     (0xF2) //其他事件上报
#define CMD_ROUND_RUN_CHANGE_DIR_REPORT (0xF3) //换向事件上报
int  easyCanPush(uint8_t data[8], uint32_t canid, uint8_t dlc);
int  easyCanPush1(uint8_t data[8], uint32_t canid, uint8_t dlc);
void easyCanProc(void);
void easyCanProc1(void);

#endif
