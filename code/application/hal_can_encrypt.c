#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arc4.h"
#include "hal_can_encrypt.h"
int CNQ_Queue_push(int level, int type, int dlen, short *data);
int CNQ_UMT_set(int type, void *addr, unsigned long timeout, int mode);

typedef struct
{
	char data[8];
	int dlen;
	int result;
} CNQMsg;

extern arc4_context g_ctx;
static unsigned long enc_seed = 0;
static unsigned char enc_key[8] =
{
	'H',
	'Q',
	0xE4,
	0x41,
	'A',
	'R',
	0x20,
	0x20,
};

void Rc4Encrypt(const unsigned char *pSrc, unsigned char *pDst, int iSize, unsigned char szKeys[8]);

#define CE_RAND_MAX 32767

static long int seed1 = 1;
static long int seed2 = 2;
static long int seed3 = 3;

#define CRANK(a, b, c, m, s)     \
	q = s / a;                   \
	s = b * (s - a * q) - c * q; \
	if (s < 0)                   \
		s += m;

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
int CANEncrypt_setkey(unsigned long key)
{
	int i;
	enc_seed = key;
	_SRand(key);
	for (i = 0; i < 8; i++)
	{
		enc_key[i] = _Rand() & 0xFF;
	}
	if (key != 0)
	{
		arc4_setup(&g_ctx, enc_key, 8);
	}
	return 0;
}

// CAN数据加解密
int CANEncrypt(unsigned char *dat)
{

	if (enc_seed)
	{
		Rc4Encrypt(dat, dat, 8, enc_key);
		return 1;
	}
	return 0;
}
#if 0
static int can_data_send(void *data)
{
	CANEncrypt(data);
	return 0 ;
	//return CNQ_Queue_push(0, 0x2000 + 0x441, 8, data);
}
#endif
