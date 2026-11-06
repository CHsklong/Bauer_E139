#ifndef __AV_DATA_H__
#define __AV_DATA_H__
 
#include "stdint.h"
#include "dataTypeDef.h"
#define AD_BUF_LEN 10
#ifdef __cplusplus
extern "C" {
#endif

//typedef unsigned short u16 ;

//typedef unsigned int u32 ;

//typedef double f64 ;
//typedef uint8_t u8 ; 
void av_set_data(u8 idx ,u16 value ) ;
u16 av_get_data(u8 idx);
#endif
