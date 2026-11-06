#ifndef _SYSMAIN_H_
#define _SYSMAIN_H_


///////////////////////////////////////////////

#include "mcu.h"
#include "dataTypeDef.h"
extern uint32_t runTimea ;
extern u16 MainTaskTicker1ms;
void MainTask1ms(void);




/******************************************
#include "DrvPwm.h"
#include "DrvAdc.h"
#include "DrvSci.h"
#include "DrvSpiDis.h"
#include "DrvFram.h"
#include "DrvQep.h"
#include "DrvPulse.h"
******************************************/

#include "UserTypeDef.h"
#include "UserAbs.h"

#include "UserMCIsr.h"
#include "UserHdPort.h"
#include "UserAng.h"
#include "UserMCPowUpTask.h"
#include "UserMCSysInitTask.h"

#include "UserMCSwPort.h"
///////////////////////////////////////////////

#include "ServoInOutLogic.h"
#include "ServoMP.h"
#include "ServoTask_Tor.h"
#include "ServoTask_Spd.h"
#include "ServoTask_Pos.h"
#include "UserPulse.h"
#include "Filter.h"
///////////////////////////////////////////////

#include "UserMCServoTask.h"
//#include "UserMCTest.h"
#include "UserMCZero.h"
//#include "UserMCDebug.h"
///////////////////////////////////////////////
/*
#include "UserCom.h"
#include "UserComCmd.h"
#include "UserComModBusRtu.h"
//#include "UserDebugModBusRtu.h"

*/


#include "UserSysPar.h"

#include "UserMotoPar.h"
//#include "UserMotoParHv.h"
#include "UserSysErr.h"

///////////////////////////////////////////////
/*


#include "UserDisTask.h"
#include "UserMonitor.h"
#include "UserHmiApi.h"
#include "UserHmi.h"
*/
///////////////////////////////////////////////


//PMSM
#include "MathCurLoop.h"
#include "MathAngle.h"
#include "MathSpeed.h"
#include "MathPidSpd.h"
#include "MathPidPos.h"
#include "MathBrake.h"
#include "MathRamp.h"
#include "MathFilter.h"
#include "MathBrake.h"
#include "MathOverLoad.h"
#include "MathVoltCal.h"
#include "MathFilterFuzzy.h"
///////////////////////////////////////////////

#include "yarncore.h"
#include "tor.h"
#include "mot1.h"
#include "mot2.h"
#include "mot3.h"
#include "mot4.h"

#endif


