#include "SysMain.h"
/***************************************************************************************/
ABS_TYPE	AbsPar;

#define	ABSEE_TEST	1

void ABS_POWER_WAIT(void)
{

}

void ABS_Tx_start(void)
{

}
/***************************************************************************************/

void ABSInit(void)
{
	memset(&AbsPar.TxdLen, 0, sizeof(ABS_TYPE));

	// ABS_TASK_PTR = &ABS_INIT_READ;

	/*
		//测试写数据和读数据 ABS_INIT_WRITE
		AbsPar.WriteInit[0] = 35;
		AbsPar.WriteInit[1] = 36;
		AbsPar.WriteInit[2] = 37;
		AbsPar.WriteInit[3] = 38;
		ABS_TASK_PTR = &ABS_INIT_WRITE;
		//中科写ee 要增加延迟
	*/
}


/***************************************************************************************/

void ABS_INIT_WRITE(void)
{
	if (AbsPar.sts == 0) {
		AbsPar.TxdBuf[0] = ABS_CF_WRITE_EE;
		AbsPar.TxdBuf[1] = ABS_EE_ADS_0;
		AbsPar.TxdBuf[2] = AbsPar.WriteInit[0];
		AbsPar.TxdBuf[3] = AbsPar.TxdBuf[0] ^ AbsPar.TxdBuf[1] ^ AbsPar.TxdBuf[2];
		AbsPar.TxdLen = 4;
		AbsPar.RecLen = 4;
		ABS_Tx_start();

		AbsPar.sts++;
	} else if (AbsPar.sts <= 10) {
		// delay
		//		DMA_Cmd(NRZ_RX_DMA_STREAM, DISABLE);                      //先关闭DMA,才能设置它 同时关闭rx dma
		//	while (DMA_GetCmdStatus(NRZ_RX_DMA_STREAM) != DISABLE){}	//等待传输结束

		AbsPar.sts++;
	} else if (AbsPar.sts <= 100) {
		AbsPar.sts++;
	} else if (AbsPar.sts <= 101) {
		AbsPar.TxdBuf[0] = ABS_CF_WRITE_EE;
		AbsPar.TxdBuf[1] = ABS_EE_ADS_1;
		AbsPar.TxdBuf[2] = AbsPar.WriteInit[1];
		AbsPar.TxdBuf[3] = AbsPar.TxdBuf[0] ^ AbsPar.TxdBuf[1] ^ AbsPar.TxdBuf[2];
		AbsPar.TxdLen = 4;
		AbsPar.RecLen = 4;

		ABS_Tx_start();  //
		AbsPar.sts++;
	} else if (AbsPar.sts <= 200) {
		AbsPar.sts++;
	} else if (AbsPar.sts == 201) {
		AbsPar.TxdBuf[0] = ABS_CF_WRITE_EE;
		AbsPar.TxdBuf[1] = ABS_EE_ADS_2;
		AbsPar.TxdBuf[2] = AbsPar.WriteInit[2];
		AbsPar.TxdBuf[3] = AbsPar.TxdBuf[0] ^ AbsPar.TxdBuf[1] ^ AbsPar.TxdBuf[2];
		AbsPar.TxdLen = 4;
		AbsPar.RecLen = 4;
		ABS_Tx_start();

		AbsPar.sts++;
	} else if (AbsPar.sts <= 300) {
		AbsPar.sts++;
	} else if (AbsPar.sts == 301) {
		AbsPar.TxdBuf[0] = ABS_CF_WRITE_EE;
		AbsPar.TxdBuf[1] = ABS_EE_ADS_3;
		AbsPar.TxdBuf[2] = AbsPar.WriteInit[3];
		AbsPar.TxdBuf[3] = AbsPar.TxdBuf[0] ^ AbsPar.TxdBuf[1] ^ AbsPar.TxdBuf[2];
		AbsPar.TxdLen = 4;
		AbsPar.RecLen = 4;
		ABS_Tx_start();

		AbsPar.sts++;
	} else if (AbsPar.sts <= 600) {


		AbsPar.sts++;
	} else if (AbsPar.sts == 601) {
		ABS_TASK_PTR = &ABS_INIT_READ;

		AbsPar.sts = 0;
	}

}
/***************************************************************************************/

void ABS_INIT_READ(void)
{
	if (AbsPar.sts == 0) {
		// delay
		//	DMA_Cmd(NRZ_RX_DMA_STREAM, DISABLE);					  //先关闭DMA,才能设置它 同时关闭rx dma
		//	while (DMA_GetCmdStatus(NRZ_RX_DMA_STREAM) != DISABLE){}	//等待传输结束

		AbsPar.sts++;

	} else if (AbsPar.sts == 1) {
		AbsPar.TxdBuf[0] = ABS_CF_READ_EE;
		AbsPar.TxdBuf[1] = ABS_EE_ADS_0;
		AbsPar.TxdBuf[2] = AbsPar.TxdBuf[0] ^ AbsPar.TxdBuf[1] ;
		AbsPar.TxdLen = 3;	//读时，发送3个字节，返回是4个字节
		AbsPar.RecLen = 4;
		ABS_Tx_start();  //
		AbsPar.sts++;



	} else if (AbsPar.sts == 2) {
		AbsPar.TxdBuf[0] = ABS_CF_READ_EE;
		AbsPar.TxdBuf[1] = ABS_EE_ADS_1;
		AbsPar.TxdBuf[2] = AbsPar.TxdBuf[0] ^ AbsPar.TxdBuf[1] ;
		AbsPar.TxdLen = 3;	//读时，发送3个字节，返回是4个字节
		AbsPar.RecLen = 4;
		ABS_Tx_start();  //
		AbsPar.sts++;
	} else if (AbsPar.sts == 3) {
		AbsPar.TxdBuf[0] = ABS_CF_READ_EE;
		AbsPar.TxdBuf[1] = ABS_EE_ADS_2;
		AbsPar.TxdBuf[2] = AbsPar.TxdBuf[0] ^ AbsPar.TxdBuf[1] ;
		AbsPar.TxdLen = 3;	//读时，发送3个字节，返回是4个字节
		AbsPar.RecLen = 4;
		ABS_Tx_start();  //
		AbsPar.sts++;

	} else if (AbsPar.sts == 4) {
		AbsPar.TxdBuf[0] = ABS_CF_READ_EE;
		AbsPar.TxdBuf[1] = ABS_EE_ADS_3;
		AbsPar.TxdBuf[2] = AbsPar.TxdBuf[0] ^ AbsPar.TxdBuf[1] ;
		AbsPar.TxdLen = 3;	//读时，发送3个字节，返回是4个字节
		AbsPar.RecLen = 4;
		ABS_Tx_start();  //
		AbsPar.sts++;

	} else if (AbsPar.sts == 5) {

		AbsPar.CalibratedAngle = -((AbsPar.ReadInit[1] << 8) + AbsPar.ReadInit[0]);

		AbsPar.sts++;

	} else if (AbsPar.sts == 6) {
		AbsPar.TxdBuf[0] = ABS_CF_ID;
		AbsPar.TxdLen = 1;
		AbsPar.RecLen = 4;
		ABS_Tx_start();
		AbsPar.sts++;
	} else if (AbsPar.sts == 7) {
		AbsPar.sts++;
	} else if (AbsPar.sts == 8) {
		AbsPar.TxdBuf[0] = ABS_CF_ABS_ABM2;
		AbsPar.TxdLen = 1;
		AbsPar.RecLen = 9;
		ABS_Tx_start();
		AbsPar.sts++;
	} else if (AbsPar.sts == 9) {

		if (AbsPar.absID > 15) {

			AbsPar.Half = 1 << (AbsPar.absID - 1) ;

		}



		ABS_TASK_PTR = &ABS_READ;

		AbsPar.sts = 0;

	} else if (AbsPar.sts == 10) {

	} else if (AbsPar.sts == 11) {



	}


}
/***************************************************************************************/
#define	ABS_RX_NULL_NUM		8

void ABS_READ(void)
{
	AbsPar.TxdBuf[0] = ABS_CF_ABS;
	AbsPar.TxdLen = 1;
	AbsPar.RecLen = 6;
	ABS_Tx_start();
	if (AbsPar.txdmaCnt > ABS_RX_NULL_NUM ) {
		gSysErr.ErrCtl.bit.EncRx_null = TRUE;
	}
}




/***************************************************************************************/
// dma 接受中断完成中 调用
void ABS_RCV_OK(void)
{
	int i ;
	AbsPar.rxdmaCnt++;
	AbsPar.txdmaCnt = 0 ;
	//数据处理	0.5us
	if (AbsPar.TxdBuf[0] == ABS_CF_ABS) {
		if ((AbsPar.RcvBuf[0] ^ AbsPar.RcvBuf[1] ^ AbsPar.RcvBuf[2] ^ AbsPar.RcvBuf[3] ^ AbsPar.RcvBuf[4])
		        == AbsPar.RcvBuf[5]) {

			AbsPar.abs = AbsPar.RcvBuf[2] + (AbsPar.RcvBuf[3] << 8) + (AbsPar.RcvBuf[4] << 16);
			AbsPar.absInc = AbsPar.abs - AbsPar.absBk;
			AbsPar.absBk = AbsPar.abs;
			//计算 inc
			if (AbsPar.absInc > AbsPar.Half) {
				AbsPar.absInc -= gQepAngle.ReferenceLine;
				AbsPar.ZFlag = TRUE;

			} else if (AbsPar.absInc < -AbsPar.Half) {
				AbsPar.absInc += gQepAngle.ReferenceLine;
				AbsPar.ZFlag = TRUE;

			} else {

				AbsPar.ZFlag = FALSE;
			}
			//---------------------------------平滑
			/**/
			AbsPar.absIncSum = AbsPar.absIncSum + AbsPar.absInc ;
			AbsPar.absIncOut = AbsPar.absIncSum >> SysParP8.ENFT;	//ABS_SH_N ;   //
			AbsPar.absIncSum = AbsPar.absIncSum - AbsPar.absIncOut ;


			//AbsPar.absIncOut = AbsPar.absInc;
			//-----------------------------


			// 累加		pos 环中使用
			//AbsPar.absMlt += AbsPar.absInc ;

			AbsPar.absMlt += AbsPar.absIncOut ;



			if (AbsPar.absMlt < 0) {
				AbsPar.absMlt += gQepAngle.CircleLine ;
			} else if (AbsPar.absMlt > gQepAngle.CircleLine) {
				AbsPar.absMlt -= gQepAngle.CircleLine ;

			}
			AbsPar.absCrcErr = 0;
		} else {
			AbsPar.absCrcErr++;
		}
	} else if (AbsPar.TxdBuf[0] == ABS_CF_ABS_ABM2) {
		if ( (AbsPar.RcvBuf[0] ^ AbsPar.RcvBuf[1] ^ AbsPar.RcvBuf[2] ^ AbsPar.RcvBuf[3] ^ AbsPar.RcvBuf[4]
		        ^AbsPar.RcvBuf[5] ^ AbsPar.RcvBuf[6] ^ AbsPar.RcvBuf[7])
		        == AbsPar.RcvBuf[8]) {

			AbsPar.abs = AbsPar.RcvBuf[2] + (AbsPar.RcvBuf[3] << 8) + (AbsPar.RcvBuf[4] << 16);
			AbsPar.abm = AbsPar.RcvBuf[5] + (AbsPar.RcvBuf[6] << 8);
			AbsPar.absRdErr = AbsPar.RcvBuf[7];
			AbsPar.abs64 = (AbsPar.abm << (AbsPar.absID)) + AbsPar.abs;


			AbsPar.absBk = AbsPar.abs ;
			AbsPar.absInc = 0;

			AbsPar.absCrcErr = 0;
		} else {
			AbsPar.absCrcErr++;
		}

	} else if (AbsPar.TxdBuf[0] == ABS_CF_ABS_ABM3) {

	} else if (AbsPar.TxdBuf[0] == ABS_CF_ID) {
		if ((AbsPar.RcvBuf[0] ^ AbsPar.RcvBuf[1] ^ AbsPar.RcvBuf[2])	== AbsPar.RcvBuf[3]) {
			AbsPar.absID = AbsPar.RcvBuf[2];


			AbsPar.absCrcErr = 0;
		} else {
			AbsPar.absCrcErr++;

		}

	} else if (AbsPar.TxdBuf[0] == ABS_CF_READ_EE) {
		if ((AbsPar.RcvBuf[0] ^ AbsPar.RcvBuf[1] ^ AbsPar.RcvBuf[2])	== AbsPar.RcvBuf[3]) {
			i = AbsPar.RcvBuf[1];
			AbsPar.ReadInit[i - 4] = AbsPar.RcvBuf[2];

			AbsPar.absCrcErr = 0;
		} else {
			AbsPar.absCrcErr++;
		}

	} else if (AbsPar.TxdBuf[0] == ABS_CF_WRITE_EE) {

		if ((AbsPar.RcvBuf[0] ^ AbsPar.RcvBuf[1] ^ AbsPar.RcvBuf[2])	== AbsPar.RcvBuf[3]) {

			AbsPar.absCrcErr = 0;
		} else {
			AbsPar.absCrcErr++;
		}


	}

}

/***************************************************************************************/


