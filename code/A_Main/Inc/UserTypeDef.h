/*
 * SysTypeDef.h
 *
 *  Created on: 2014-8-1
 *      Author: admin
 */

#ifndef USERTYPEDEF_H_
#define USERTYPEDEF_H_

//-----------------------------------------------------------//
//
#define DSP_SOFT_VERSION 1
//-----------------------------------------------------------//
#define	CUR_UV 	1
#define	CUR_UW	2

#define	CUR_SEL	CUR_UV
//	#define	CUR_SEL	CUR_UW
//-----------------------------------------------------------//
#define	OC_PWM1	1
#define	OC_PWM2 2

#define	OC_PWM_SEL OC_PWM1
//-----------------------------------------------------------//



#define SYSTEM_FREQUENCY 	160 		//mHZ						// Define the system frequency (MHz)
#define  INT_FREQ               20 //16              //KHZ

#define PWM_FREQUENCY           (20000)  //    (16000)
#define PWM_RELOAD                  (motor_clock_hz/PWM_FREQUENCY)

#define POS_LOOP_FRE  		 10  	//8  // 	//KHZ

//-----------------------------------------------------------//

#define	ENCODE_ABZ  0
#define	ENCODE_17	1

#define	ENCODE_SEL	ENCODE_17
//-----------------------------------------------------------//
//380V220V
#define POW_VER_220 0
#define POW_VER_380 1
#define	POW_VER_24V 2
//-----------------------------------------------------------//

//-----------------------------------------------------------//
//CPLD
#define CPLD_CHIP_DISABLE 	0		//CPLD
#define CPLD_CHIP_ENABLE 	1	    //CPLD
//-----------------------------------------------------------//

//-----------------------------------------------------------//
//CPLD
#define POW_VER_STS 		POW_VER_24V
#define CPLD_CHIP_STS  	CPLD_CHIP_DISABLE
#define SW_VERSION  (POW_VER_STS*1000 + CPLD_CHIP_STS*100 + DSP_SOFT_VERSION)
//-----------------------------------------------------------//


//-----------------------------------------------------------//
//
#define MC_ZERO_MODE 11
#define MC_TEST_MODE 22
#define MC_DEBG_MODE 33
//-----------------------------------------------------------//


/************************************************************************************************************/
//


extern unsigned int 	ISR_FREQUENCY; 					// Define the ISR frequency (kHz)
extern unsigned int  	SPEED_LOOP_PSR;					//
extern float  			T_PWM;						//(0.001/ISR_FREQUENCY)    // Samping period (sec), see parameter.h
extern unsigned int  	SVPWM_PWM_PERIOD;				//SYSTEM_FREQUENCY*1000000*T_PWM/2;	//PWM
extern unsigned int 	SVPWM_PWM_DEDTIME;			//PWM
/************************************************************************************************************/


/************************************************************************************************************/
#define ENCODER_CIRCLE 			100.0f								//
/************************************************************************************************************/


/************************************************************************************************************/
enum {
	ENC_1024  = 1,
	ENC_STANDARD = 2,
	ENC_LESS,
	ENC_STANDARD_5K = 5,
	ENC_LESS_5K,
	ENC_ABS_17BIT = 17,
	ENC_ABS_20BIT = 20,
	ENC_ABS_23BIT = 23,
	ENC_NUM
};

enum {
	ENC_SWAP_DIS = 0,
	ENC_SWAP_EN
};
/************************************************************************************************************/


/************************************************************************************************************/
//
#define MECH_FREQ 			50
#define BASE_SPD_RPM   	(MECH_FREQ*60)
/************************************************************************************************************/

/************************************************************************************************************/
#define MAX_ERR_ZERO_POSE 50
/************************************************************************************************************/


void gSysTypeDefVarInit(void);


#endif /* SYSTYPEDEF_H_ */
