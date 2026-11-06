#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cansec.h"
#include "arc4.h"
static unsigned char sec_key[8];
static unsigned long sec_keygen = DEFAULT_KEY;
static int m_nh_encrypt_support = 0;
static unsigned long sec_sel = 1;

#define CE_RAND_MAX	32767

static long int seed1 = 1;
static long int seed2 = 2;
static long int seed3 = 3;

#define CRANK(a,b,c,m,s) 	\
	q = s/a;		\
	s = b*(s-a*q) - c*q;	\
	if(s<0) s+=m;

static long int _Rand()
{
	register long int q;

	CRANK(206, 157, 31, 32363, seed1);
	CRANK(217, 146, 45, 31727, seed2);
	CRANK(222, 142, 133, 31657, seed3);

	return seed1 ^ seed2 ^ seed3;
}

static void _SRand(unsigned long seed)
{
	seed &= CE_RAND_MAX;
	seed1 = seed % 32362 + 1;
	seed2 = seed % 31726 + 1;
	seed3 = seed % 31656 + 1;
}

// 密码更新接口
int CANEncrypt_setkey(unsigned short keytype,unsigned long key)
{
	int i;
	sec_keygen = key;
        sec_sel = keytype;
	_SRand(key);
	for(i = 0; i < 8; i++) {
		sec_key[i] = _Rand() & 0xFF;
	}
}

unsigned long CANEncrypt_getkey()
{
	return sec_keygen;
}

void EncryptM1(unsigned char *dat, int len, unsigned char *key)
{
	int i;
	for(i = 0; i < len; i++) {
		dat[i] ^= key[i];
	}
}

/*
 * CAN数据加解密
 * mode: 0 解密
 *	 1 加密
 */
int CANEncrypt(unsigned char *dat, int len, int mode)
{
	int key_update = 0;
	unsigned long key = 0;
	//if(mode && dat[0] == 0xA408) {
	//	key = dat[2] | (dat[3] << 16);
	//	key_update = 1;
	//}
	if(sec_keygen) {
		if(sec_sel == 1) {
			EncryptM1(dat, len, sec_key);
		}
		else {
			Rc4Encrypt(dat, dat, len, sec_key);
		}
	}

	//if(key_update) {
	//	CANEncrypt_setkey(key);
	//}
	return 1;
}

