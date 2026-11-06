
#include  "SysMain.h"
#include "mcu.h"
//-------------------------------------------------------------------------------
//void MainTask1ms(void);
//-------------------------------------------------------------------------------
u16	keyread;
//-------------------------------------------------------------------------------
int main(void)
{


	
	filter_K_cal(&gFilter1);

	filter_K_idiq(&gFilterId);
	filter_K_idiq(&gFilterIq);

	filter_K_idiq(&gFilterIU);
	filter_K_idiq(&gFilterIV);

	MonitorVarInit();								//监控数据初始化
	gSysErrVarInit();								//故障寄存器初始化 
	MC_HD_PortInit();								//端口变量初始化 

	//eeprom
	
	SysParHandle();	
	//uart
		
	MC_Task_Ptr = &SysPowUpTask;			//初始化任务-主中断
	Save_Task_Ptr = &SaveTaskSec;			//初始化任务-保存任务
	

	/////////////////////////////////////////////////////////////////////////////
	for(;;)
	{
	
		filter_K_cal(&gFilter1);

		filter_K_idiq(&gFilterId);
		filter_K_idiq(&gFilterIq);

		filter_K_idiq(&gFilterIU);
		filter_K_idiq(&gFilterIV);

		MainTask1ms();			//1msTask
	}

}
//-------------------------------------------------------------------------------
u16 MainTaskTicker1ms = 0;
void MainTask1ms(void)
{
	if(MainTaskTicker1ms < ISR_FREQUENCY) return;
	MainTaskTicker1ms = 0;
	

	//----------------------------------------------------
	//温度采样及风扇控制
	HD_Temp.gRTempAdc_value = MC_HD_Port.Temp;
	HD_Temp.gRTemp_HT_Err_Thd = SysParP7.TMPH;
	MC_HD_TempSample(&HD_Temp);
	MC_HD_Port.MCOutport.bit.FanPort = HD_Temp.gFan_sts;
	if(HD_Temp.gRTemp_sts == HIGH_TEMP_ERR)
	{
		gSysErr.ErrHd.bit.OverTemp = TRUE;
	}
	//----------------------------------------------------

	//----------------------------------------------------
	//监控数据处理
	MonitorDataLoad();
	//----------------------------------------------------

}

//-------------------------------------------------------------------------------





//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------



