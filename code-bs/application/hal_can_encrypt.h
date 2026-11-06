
#ifndef HAL_CAN_ENCRYPT_H
#define HAL_CAN_ENCRYPT_H

// 密码更新接口
int CANEncrypt_setkey(unsigned long key) ;
// CAN数据加解密
int CANEncrypt(unsigned char *dat) ;

#endif 
