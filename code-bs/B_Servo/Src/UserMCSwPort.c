/*
 * UserMCSwPort.c
 *
 *  Created on: 2016420
 *      Author: admin
 */


#include "SysMain.h"


MC_SWPORT_TYPEDEF McSwPort = {0,0};
AdcChannelTypeDef AngTorRef[2];

void MC_ANG_SWPORT_INIT(void)
{
	memset_16(&McSwPort.In.All,0,sizeof_16(MC_SWPORT_TYPEDEF));
	McSwPort.uKpp = SysParP0.KPP1;
	McSwPort.uKvp = SysParP0.KVP1;
	McSwPort.uKvi = SysParP0.KVI;
	memset_16(&AngTorRef[ATOR_ID].iReg,0,sizeof_16(AdcChannelTypeDef));
	memset_16(&AngTorRef[ASPD_ID].iReg,0,sizeof_16(AdcChannelTypeDef));
}

ATTR_RAMFUNC void McSwPortRead(void)
{
	u16 temp = 0;
	u16 temp2 = 0;
	u32 SwInReg = 0;

	/*****************************************************************************/
	temp = SysParP1.LOGI1 ? !MC_HD_Port.MCInport.bit.InPort1 : MC_HD_Port.MCInport.bit.InPort1;
	temp2 = SysParP1.DI1 ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));

	temp = (SysParP1.LOGI2) ? !MC_HD_Port.MCInport.bit.InPort2 : MC_HD_Port.MCInport.bit.InPort2;
	temp2 = SysParP1.DI2  ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));

	temp = (SysParP1.LOGI3) ? !MC_HD_Port.MCInport.bit.InPort3 : MC_HD_Port.MCInport.bit.InPort3;
	temp2 = SysParP1.DI3  ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));

	temp = (SysParP1.LOGI4) ? !MC_HD_Port.MCInport.bit.InPort4 : MC_HD_Port.MCInport.bit.InPort4;
	temp2 = SysParP1.DI4  ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));

	temp = (SysParP1.LOGI5) ? !MC_HD_Port.MCInport.bit.InPort5 : MC_HD_Port.MCInport.bit.InPort5;
	temp2 = SysParP1.DI5  ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));

	temp = (SysParP1.LOGI6) ? !MC_HD_Port.MCInport.bit.InPort6 : MC_HD_Port.MCInport.bit.InPort6;
	temp2 = SysParP1.DI6  ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));

	temp = (SysParP1.LOGI7) ? !MC_HD_Port.MCInport.bit.InPort7 : MC_HD_Port.MCInport.bit.InPort7;
	temp2 = SysParP1.DI7  ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));
	
	temp = (SysParP1.LOGI8) ? !MC_HD_Port.MCInport.bit.InPort8 : MC_HD_Port.MCInport.bit.InPort8;
	temp2 = SysParP1.DI8  ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));

	temp = (SysParP1.LOGI9) ? !MC_HD_Port.MCInport.bit.InPort9 : MC_HD_Port.MCInport.bit.InPort9;
	temp2 = SysParP1.DI9  ;
	temp2 = (temp2>INPUT_FUN_NUM)?INPUT_FUN_NUM:temp2;
	if(temp)  SwInReg |= (0x01L<<temp2);
	else SwInReg &= (~(0x01L<<temp2));

	
	McSwPort.In.All = SwInReg;


	/*****************************************************************************/
}




ATTR_RAMFUNC void McAngPortRead(void)
{
	u16 temp = 0;
	u16 tempno = 0;
	float ZeroSpd = 0;
/**************************************1***************************************************************/

	//---------------------------------------------------------------------------------------------//
	//

	//---------------------------------------------------------------------------------------------//


	//---------------------------------------------------------------------------------------------//
	if(SysParP5.TAS == 3)
	{
	#if 0
		switch(temp)
		{
			case 0:
				AngTorRef[ATOR_ID].iReg = MC_HD_Port.Ref1;
				AngTorRef[ATOR_ID].ErrVolt = SysParP1.TCMLPF;
				break;
			case 1:
				AngTorRef[ATOR_ID].iReg = MC_HD_Port.Ref2;
				AngTorRef[ATOR_ID].ErrVolt = SysParP1.VCMLPF;
				break;
			default:
				AngTorRef[ATOR_ID].iReg = AngTorRef[ATOR_ID].Offset;
				AngTorRef[ATOR_ID].ErrVolt = 0;
				break;
		}
	#endif
		AngTorRef[ATOR_ID].iReg = MC_HD_Port.Ref2;
		AngTorRef[ATOR_ID].ErrVolt = SysParP1.TCMLPF;
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		//
		AngTorRef[ATOR_ID].Gain =  SysParP1.TCM * ((1.0/3000) * 3.3f /(EX_REF_ANG_MAX_VOLT * 4096));

		//
		AngTorRef[ATOR_ID].Offset = (s16)((EX_REF_ANG_ZERO_VOLT + SysParP1.TAO*0.00015f)*EX_REF_ANG_ZERO_SCALE);


		//
		if(SysParP1.TFLT == 0)
		{
			AngTorRef[ATOR_ID].Kn_1 = 0;
			AngTorRef[ATOR_ID].Kn = 1.0f;
		}else{
			AngTorRef[ATOR_ID].Kn_1 = 1.0f/(1.0f+((2*PI)/SysParP1.TFLT));
			AngTorRef[ATOR_ID].Kn = 1.0f - 1.0f/(1.0f+((2*PI)/SysParP1.TFLT));
		}
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		ADC_LP_MACRO(&AngTorRef[ATOR_ID]);
		//McSwPort.TorGvn = AngTorRef[ATOR_ID].Out;
		McSwPort.TorGvn = 0;
		ZeroSpd = SysParP1.TST*0.00033333333f;
		if((-ZeroSpd <= McSwPort.TorGvn) && (McSwPort.TorGvn <= ZeroSpd))
		{
			McSwPort.TorGvn = 0;
		}

		//---------------------------------------------------------------------------------------------//
	}else{
		AngTorRef[ATOR_ID].ErrVolt = SysParP1.TCMLPF;
		//---------------------------------------------------------------------------------------------//
		McSwPort.TorGvn = SysParP5.CTOR * 0.00033333333f;	//1
		//---------------------------------------------------------------------------------------------//
	}
/***********************************************************************************************************/


/***********************************************************************************************************/
	//---------------------------------------------------------------------------------------------//
	// 

	//---------------------------------------------------------------------------------------------//

	//---------------------------------------------------------------------------------------------//
	//
	if(SysParP4.SAS == 3)
	{
	#if 0
		switch(temp)
		{
			case 0:
				AngTorRef[ASPD_ID].iReg = MC_HD_Port.Ref2;
				AngTorRef[ASPD_ID].ErrVolt = SysParP1.VCMLPF;
				break;
			case 1:
				AngTorRef[ASPD_ID].iReg = MC_HD_Port.Ref1;
				AngTorRef[ASPD_ID].ErrVolt = SysParP1.TCMLPF;
				break;
			default:
				AngTorRef[ASPD_ID].iReg = AngTorRef[ASPD_ID].Offset;
				AngTorRef[ASPD_ID].ErrVolt = 0;
				break;
		}
	#endif
		AngTorRef[ASPD_ID].iReg = MC_HD_Port.Ref1;
		AngTorRef[ASPD_ID].ErrVolt = SysParP1.VCMLPF;
		//---------------------------------------------------------------------------------------------//


		//---------------------------------------------------------------------------------------------//
		// 
		AngTorRef[ASPD_ID].Gain = SysParP1.VCM * ((1.0/3000) * 3.3f /(EX_REF_ANG_MAX_VOLT * 4096));

		// 
		AngTorRef[ASPD_ID].Offset = (s16)((EX_REF_ANG_ZERO_VOLT + SysParP1.SAO*0.00015f)*EX_REF_ANG_ZERO_SCALE);

		// 
		if(SysParP1.SFLT == 0)
		{
			AngTorRef[ASPD_ID].Kn_1 = 0.0f;
			AngTorRef[ASPD_ID].Kn = 1.0f;
		}else{
			AngTorRef[ASPD_ID].Kn_1 = 1.0f/(1.0f+((2*PI)/SysParP1.SFLT));
			AngTorRef[ASPD_ID].Kn = 1.0f - 1.0f/(1.0f+((2*PI)/SysParP1.SFLT));
		}
		//---------------------------------------------------------------------------------------------//

		//---------------------------------------------------------------------------------------------//
		ADC_LP_MACRO(&AngTorRef[ASPD_ID]);
		//McSwPort.SpdGvn = AngTorRef[ASPD_ID].Out;

		McSwPort.SpdGvn = 0;
		ZeroSpd = SysParP1.ZST*0.00033333333f;
		if((-ZeroSpd <= McSwPort.SpdGvn) && (McSwPort.SpdGvn <= ZeroSpd))
		{
			McSwPort.SpdGvn = 0;
		}
		//---------------------------------------------------------------------------------------------//

	}else{
		AngTorRef[ASPD_ID].ErrVolt = SysParP1.VCMLPF;
		//---------------------------------------------------------------------------------------------//

		McSwPort.SpdGvn = SysParP4.CSPD * 0.00033333333f;	//1

		//---------------------------------------------------------------------------------------------//
	}

	//---------------------------------------------------------------------------------------------//
	//
	if(SysParP1.OSET)
	{
		if(SysParP1.OSET % 10){
			SysParP1.SAO = ((MC_HD_Port.Ref1 / EX_REF_ANG_ZERO_SCALE) - EX_REF_ANG_ZERO_VOLT) * 6666.66666667f;//*1000mv/0.15
			SysParP6.SAVE = TRUE;
		}

		if(SysParP1.OSET / 10){
			SysParP1.TAO = ((MC_HD_Port.Ref2 / EX_REF_ANG_ZERO_SCALE) - EX_REF_ANG_ZERO_VOLT) * 6666.66666667f;//*1000mv/0.15
			SysParP6.SAVE = TRUE;
		}
		SysParP1.OSET = 0;
	}
	//---------------------------------------------------------------------------------------------//

/***********************************************************************************************************/

}


ATTR_RAMFUNC void McSwPortLoad(void)
{
	//6.7us,OK
	u32 temp = 0,temp2 = 0;
	/*****************************************************************************/
	temp = SysParP2.DO1 ;
	temp = (temp>OUTPUT_FUN_NUM)?OUTPUT_FUN_NUM:temp;
	temp2 = (McSwPort.Out.All  >> temp)&0x01L;
	MC_HD_Port.MCOutport.bit.OutPort1 =  (SysParP2.LOGO1)? !temp2 : temp2;

	temp = SysParP2.DO2 ;
	temp = (temp>OUTPUT_FUN_NUM)?OUTPUT_FUN_NUM:temp;
	temp2 = (McSwPort.Out.All  >> temp)&0x01L;
	MC_HD_Port.MCOutport.bit.OutPort2 =  (SysParP2.LOGO2)? !temp2 : temp2;

	temp = SysParP2.DO3 ;
	
	//if(temp == OUTPUT_FUN_HZFLG)
	//{
	//	MC_HD_Port.MCOutport.bit.Out3Zflag = 1;
	//}else{
		MC_HD_Port.MCOutport.bit.Out3Zflag = 0;
		temp = (temp>OUTPUT_FUN_NUM)?OUTPUT_FUN_NUM:temp;
		temp2 = (McSwPort.Out.All  >> temp)&0x01L;
		MC_HD_Port.MCOutport.bit.OutPort3 =  (SysParP2.LOGO3)? !temp2 : temp2;
	//}

	temp = SysParP2.DO4 ;

		MC_HD_Port.MCOutport.bit.Out4Zflag = 0;
		temp = (temp>OUTPUT_FUN_NUM)?OUTPUT_FUN_NUM:temp;
		temp2 = (McSwPort.Out.All  >> temp)&0x01L;
		MC_HD_Port.MCOutport.bit.OutPort4 =  (SysParP2.LOGO4)? !temp2 : temp2;
	

	temp = SysParP2.DO5 ;

		temp = (temp>OUTPUT_FUN_NUM)?OUTPUT_FUN_NUM:temp;
		temp2 = (McSwPort.Out.All  >> temp)&0x01L;
		MC_HD_Port.MCOutport.bit.OutPort5 =  (SysParP2.LOGO5)? !temp2 : temp2;
	
	temp = SysParP2.DO6 ;

		temp = (temp>OUTPUT_FUN_NUM)?OUTPUT_FUN_NUM:temp;
		temp2 = (McSwPort.Out.All  >> temp)&0x01L;
		MC_HD_Port.MCOutport.bit.OutPort5 =  (SysParP2.LOGO6)? !temp2 : temp2;
	

	
	/*****************************************************************************/
	MC_HD_Port.MCOutport.bit.BreakPort = BrakeVar.OutSts;
}

/*****************************************************************************/

ATTR_RAMFUNC void McGainHandle(void)
{
	
	/*
	u16 temp1 = 0;
	u16 Kpp = 0, Kvp = 0, Kvi=0;
	float Kn = 1.0;
	float Kn_1 = 0.0;

	
	temp1 = SysParP2.GCC%10;
	//temp2 = SysParP2.GCC/10;

	switch(temp1){
		case 0:
			Kpp = SysParP0.KPP1;
			Kvp = SysParP0.KVP1;
			Kvi = SysParP0.KVI;
			break;
		case 1:
			if(McSwPort.In.bit.GAINUP == 0){
				Kpp = SysParP0.KPP1;
				Kvp = SysParP0.KVP1;
			}else if(McSwPort.In.bit.GAINUP == 1){
				Kpp = SysParP0.KPP2;
				Kvp = SysParP0.KVP2;
			}
			Kvi = SysParP0.KVI;
			break;
		default:
			Kpp = SysParP0.KPP1;
			Kvp = SysParP0.KVP1;
			Kvi = SysParP0.KVI;
			break;
	}
	if(SysParP2.GUT == 0)
	{
		Kn = 1.0f;
		Kn_1 = 0.0f;
	}else{
		Kn_1 =1.0f/(1.0f+((2*PI*T_PWM*SPEED_LOOP_PSR*1000)/ SysParP2.GUT));
		Kn = 1.0f - Kn_1;
	}

	McSwPort.uKpp = ((float)McSwPort.uKpp * Kn_1 + (float)Kpp * Kn);
	McSwPort.uKvp = ((float)McSwPort.uKvp * Kn_1 + (float)Kvp * Kn) ;
	McSwPort.uKvi = ((float)McSwPort.uKvi * Kn_1 + (float)Kvi * Kn);
*/

	McSwPort.uKpp = SysParP0.KPP1;;
	McSwPort.uKvp = SysParP0.KVP1* gMcCtlPar.Kps_R;;
	McSwPort.uKvi = SysParP0.KVI;


	/********************************************************************************************************/
	//
	if(SysParP0.MSPD > MotoPar.MSPD)
	{
		SysParP0.MSPD = MotoPar.MSPD;
	}

	//
	if(SysParP0.MTOR > (gMcCtlPar.CurMax*1000))
	{
		SysParP0.MTOR = gMcCtlPar.CurMax*1000;
	}
	/********************************************************************************************************/

}
/*****************************************************************************/
