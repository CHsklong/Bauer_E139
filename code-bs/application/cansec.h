#ifndef _CANSEC_H_
#define _CANSEC_H_

#include "stdint.h"

#define DEFAULT_KEY 0x4851E102
// 密码更新接口
int CANEncrypt_setkey(unsigned short keytype,unsigned long key) ;
/*
 * CAN数据加解密
 * mode: 0 解密
 *	 1 加密
 */
int CANEncrypt(unsigned char *dat, int len, int mode) ;
#endif // !_CANSEC_H_