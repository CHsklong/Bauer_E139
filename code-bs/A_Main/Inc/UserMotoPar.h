/*
 * UserMotoPar.h
 *
 *  Created on: 2016824
 *      Author: renbin
 */

#ifndef USERMOTOPAR_H_
#define USERMOTOPAR_H_

#include "mcu.h"
#include "UserTypeDef.h"

#if POW_VER_STS == POW_VER_24V
//---------------------------------------------------------------------------------------//
//
typedef struct{
	u16 	MotoId;		// 1
	u16 	DrvId;		//ID 2
	u32 	ENCL;		// 3
	u16 	ENCT;		// 4
	s16 	EZRO; 		//         5
	u16 	POW;		//		W  6
	u16 	VOLT;		//	V 7
	u16 	NSPD;		//		RPM 8
	u16 	MSPD;		//		RPM 9
	u16 	KvpScale;	//		10
	float 	NCUR;		//		A 11 12
	float 	NTOR;		//		N*M 13 14
	float 	MTOR;		//		N*M 15 16
	u16 	POLES;		// 17
	u16 	VCONSTANT;	//	V/1000RMP 18
	float 	INERTIA;	//		(10^-3)kg*m^2  19 20
	float 	RES;		//		R 21 22
	float 	INDUCT;		//		mH 23 24
	float 	Kpt;
	float 	Kit;
}MC_SYS_PAR_MOTO;
#define MOTO_PAR_LEN sizeof_16(MC_SYS_PAR_MOTO)
//---------------------------------------------------------------------------------------//

extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_5A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_10A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_15A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_20A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_30A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_50A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_70A[];

extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_M_5A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_M_10A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_M_15A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_M_20A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_M_30A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_M_50A[];
extern const MC_SYS_PAR_MOTO MOTOPAR_INIT_M_70A[];

extern MC_SYS_PAR_MOTO MotoPar;

void gMotoParSec(void);

#endif


#endif /* USERMOTOPAR_H_ */
