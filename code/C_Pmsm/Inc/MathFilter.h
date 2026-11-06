/*
 * MathFilter.h
 *
 *  Created on: 2016年4月12日
 *      Author: admin
 */

#ifndef MATHFILTER_H_
#define MATHFILTER_H_

#include "Bsp.h"

#define AVG_BUF_LEN_MAX 32

typedef struct{
	u8 	pLen;						//滤波深度
	u8 	Index;						//索引号
	s16 data[AVG_BUF_LEN_MAX];		//缓存
	s32 add;						//和
	s16 In;							//输入数据
	s16 Out;						//输出数据
}DATA_AVG_TYPEDEF;



typedef struct{
	float In;
	float Sum;
	u16   Ticker;
	u16   Len;
	float Scale;
	float Out;
}FITER_AVG_TYPEDEF;



/***************************************************************************************/
#define FiterAvgMacro(dp)		\
{								\
	dp.Ticker++;				\
	dp.Sum += dp.In;			\
	if(dp.Ticker >= dp.Len)		\
	{							\
		dp.Out = dp.Sum*dp.Scale;	\
		dp.Sum = 0;				\
		dp.Ticker = 0;			\
	}							\
}								\
/***************************************************************************************/


/***************************************************************************************/
#define FiterPeakMacro(dp)					\
	dp.Ticker++;							\
	if(dp.In >= 0)							\
	{										\
		if(dp.Sum < dp.In) dp.Sum = dp.In;	\
	}else{									\
		if(dp.Sum > dp.In) dp.Sum = dp.In;	\
	}										\
	if(dp.Ticker >= dp.Len)					\
	{										\
		dp.Out = dp.Sum;					\
		dp.Sum = 0;							\
		dp.Ticker = 0;						\
	}										\


/***************************************************************************************/

/***************************************************************************************/

#define PulseAvgMacro(dp)					\
{											\
	dp.Ticker++;							\
	dp.Sum += dp.In;						\
	if(dp.Ticker >= dp.Len)					\
	{										\
		dp.Out = dp.Sum;					\
		dp.Sum = 0;							\
		dp.Ticker = 0;						\
	}										\
}											\

/***************************************************************************************/




#endif /* MATHFILTER_H_ */
