/*
 * UserMCApi.c
 *
 *  Created on: 2015-12-2
 *      Author: admin
 */

#include "SysMain.h"

u16	pwmtest = 0;

MC_HD_PORT MC_HD_Port;

/*******************************************************************************************************************************/
void MC_HD_PortInit(void)
{
	memset_16(&MC_HD_Port.Index, 0, sizeof_16(MC_HD_PORT));
}
/*******************************************************************************************************************************/


/*******************************************************************************************************************************/
s16 InterPulseReg = 0;
ATTR_RAMFUNC void MC_HD_Port_Read(void)
{
	//---------------------------------------------------------------------------------------------------
	//
	// DrvQepRegSnap();
	MC_HD_Port.EncoderPos = QepVar.QepData;
	//MC_HD_Port.MCInport.bit.ErrABZ = (ENCODE_ERRA()<<2)|(ENCODE_ERRB()<<1)|ENCODE_ERRZ();
	//MC_HD_Port.MCInport.bit.ErrUVW = (ENCODE_ERRU()<<2)|(ENCODE_ERRV()<<1)|ENCODE_ERRW();
	//MC_HD_Port.MCInport.bit.QepUVW = (ENCODE_U()<<2)|(ENCODE_V()<<1)|ENCODE_W();

//       read_u = gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOB, UPIN_NUM);
//       read_v = gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOB, VPIN_NUM);
//       read_w = gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOB, WPIN_NUM);
//       MC_HD_Port.MCInport.bit.QepUVW = read_u<<2 | read_v<<1 | read_w ;

	//MC_HD_Port.EncoderQD = MC_HD_Port.MCInport.bit.QepUVW;
	//---------------------------------------------------------------------------------------------------




	//---------------------------------------------------------------------------------------------------
	if (gPwmIsrTacker == 1) //
	{

		//----------------------------------------------------
		//
		//InterPulseReg = SERVO_PLUSE_REG;
	
		//----------------------------------------------------

		//----------------------------------------------------
		/*
		MC_HD_Port.MCInport.bit.InPort1 = INPORT_INPUT1();
		MC_HD_Port.MCInport.bit.InPort2 = INPORT_INPUT2();
		MC_HD_Port.MCInport.bit.InPort3 = INPORT_INPUT3();
		MC_HD_Port.MCInport.bit.InPort4 = INPORT_INPUT4();
		MC_HD_Port.MCInport.bit.InPort5 = 0;
		MC_HD_Port.MCInport.bit.InPort6 = 0;
		MC_HD_Port.MCInport.bit.InPort7 = 0;
		        */
		//----------------------------------------------------
	}
	//---------------------------------------------------------------------------------------------------


	//---------------------------------------------------------------------------------------------------
	//
	//MC_HD_Port.MCInport.bit.BrkOV = BRAKE_CUR_OVR();
	//---------------------------------------------------------------------------------------------------

}
/*******************************************************************************************************************************/

/*******************************************************************************************************************************/
//
ATTR_RAMFUNC void MC_HD_Ang_Port_Read(void)
{
	u16 busy = 0;
	u16	i = 0;
	u32   temp32 = 0;
//	OUTPORT_OUT1(FALSE);
	/*
	do{
		busy = ADC_CHANGE_END_FLAG();
	}while(busy==0);
	ADC_SOFT_FLAG_CLEAR();
	// 20adc  30us
	*/

	while (Adc1_Trigger_Times == 0) {};
	Adc1_Trigger_Times = 0;
	/*
		while(Adc3_Trigger_Times == 0){};
		Adc3_Trigger_Times = 0;
	*/
//		OUTPORT_OUT1(TRUE);
#if 1
	temp32 = ADC_IU_RES;
	MC_HD_Port.CurU = temp32 >> 4;
	temp32 = ADC_IV_RES;
	MC_HD_Port.CurV = temp32 >> 4;
	temp32 = ADC_VBUS_RES;
	MC_HD_Port.Vbus = temp32 >> 4;


	temp32 = ZL_POS_ENC_RES ;
	MC_HD_Port.zl_pos = temp32 >> 4;
	temp32 = ZL_CUR_PWM_RES ;
	MC_HD_Port.zl_cur = temp32 >> 4;

	//MC_HD_Port.Temp = ADC_TEMP_RES;
	//MC_HD_Port.Ref1 = ADC_VREF1_RES;
	//MC_HD_Port.Ref2 = ADC_VREF2_RES;
	//MC_HD_Port.DrvInfo = ADC_INFO_RES;

#endif
	//---------------------------------------------------------------------------------------------------
	/*
	for(i = 0; i<16;i++)
	{
		ADC1_RegularConvertedValueTab[i] =0;
		ADC3_RegularConvertedValueTab[i] =0;
	}
	*/
	//----------------------------------------------------
	//
	AngVar_Vbus.iReg = MC_HD_Port.Vbus;
	ADC_LP_MACRO(&AngVar_Vbus);
	//----------------------------------------------------

}

/*******************************************************************************************************************************/
/*******************************************************************************************************************************/
//
ATTR_RAMFUNC void MC_HD_Port_Load(void)
{
	//---------------------------------------------------------------------------------------------------
	//
	MC_HD_Port.McCpldOut.bit.ExOut1 = !MC_HD_Port.MCOutport.bit.OutPort1;
	MC_HD_Port.McCpldOut.bit.ExOut2 = !MC_HD_Port.MCOutport.bit.OutPort2;
	MC_HD_Port.McCpldOut.bit.ExOut3 = !MC_HD_Port.MCOutport.bit.OutPort3;
	MC_HD_Port.McCpldOut.bit.ExOut4 = !MC_HD_Port.MCOutport.bit.OutPort4;
	MC_HD_Port.McCpldOut.bit.ExOut5 = !MC_HD_Port.MCOutport.bit.OutPort5;

	MC_HD_Port.McCpldOut.bit.Out1Zflag = MC_HD_Port.MCOutport.bit.Out1Zflag;
	MC_HD_Port.McCpldOut.bit.Out2Zflag = MC_HD_Port.MCOutport.bit.Out2Zflag;
	MC_HD_Port.McCpldOut.bit.Out3Zflag = MC_HD_Port.MCOutport.bit.Out3Zflag;
	MC_HD_Port.McCpldOut.bit.Out4Zflag = MC_HD_Port.MCOutport.bit.Out4Zflag;
	MC_HD_Port.McCpldOut.bit.Out5Zflag = MC_HD_Port.MCOutport.bit.Out5Zflag;
	//---------------------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------------------
	MC_HD_Port.McCpldOut.bit.QepDirCmd = SysParP3.AOUT;
	MC_HD_Port.McCpldOut.bit.QepDivPsc = SysParP3.GROUT;
	MC_HD_Port.McCpldOut.bit.PulseCmd = SysParP3.PPT;
	MC_HD_Port.McCpldOut.bit.InFilter = SysParP1.DRT;
	MC_HD_Port.McCpldOut.bit.PulsFilter = SysParP3.PFT;

	MC_HD_Port.PwnEn = gSysCtlVar.Ctl.bit.PwmEn;
	//---------------------------------------------------------------------------------------------------



//2020
//	OUTPORT_OUT1(MC_HD_Port.MCOutport.bit.OutPort1);
//	OUTPORT_OUT2(MC_HD_Port.MCOutport.bit.OutPort2);
	//---------------------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------------------
	//TEST
	if (pwmtest) {
		if (MC_HD_Port.MCInport.bit.IBOV == FALSE) { //ipm    set 1
			SVPWMON();
		}
	} else {
		if (MC_HD_Port.PwnEn == TRUE) {
			SVPWMON();
		} else {
			if (init_power == 0) {
				SVPWMOFF();
			}

		}

	}


	//---------------------------------------------------------------------------------------------------
}
/*******************************************************************************************************************************/

/*******************************************************************************************************************************/
//CPLD
u16 CpldErr = 0;
void MC_HD_Cpld_Com(void)
{

}
/***********************************************************************************/



/*******************************************************************************************************************************/

/*******************************************************************************************************************************/



/*******************************************************************************************************************************/
//-------------------------------------------------------------------------------------------
QepRegVarTypeDef QepVar = {0, 0, 0, 0};
ATTR_RAMFUNC void DrvQepRegSnap(void)
{

}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
void DrvQepInit(void)
{

}
//-------------------------------------------------------------------------------------------

/*******************************************************************************************************************************/



/*******************************************************************************************************************************/
MC_HD_TempSampleTypeDef HD_Temp = MC_HD_RADITORTEMP_INIT;

/*******************************************************************************************************************************/


