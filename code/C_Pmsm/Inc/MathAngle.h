/*
 * MathAngleNew.h
 *
 *  Created on: 2016413
 *      Author: admin
 */

#ifndef MATHANGLE_H_
#define MATHANGLE_H_

#include "Bsp.h"

typedef struct {float  ElecTheta;        // Output: Motor Electrical angle (Q24)
				float  ElecThetaN;
				float  ElecThetaN1;

                int32  MechTheta;        // Output: Motor Mechanical Angle (Q24)
                Uint16 DirectionQep;    // Output: Motor rotation direction (Q0)
                Uint16 QepPeriod;       // Output: Capture period of QEP signal in number of EQEP capture timer (QCTMR) period  (Q0)
                 int32 QepCountIndex;   // Variable: Encoder counter index (Q0)
                 int32 RawTheta;        // Variable: Raw angle from EQEP Postiion counter (Q0)
                 int32 RawThetanew;     // Variable: Raw angle from EQEP Postiion counter (Q0)
                float  MechScaler;      // Parameter: 0.9999/total count (Q30)
                Uint32 LineEncoder;     //  Number of line encoder (Q0)
                Uint16 PolePairs;       // Parameter: Number of pole pairs (Q0)
                 int32 CalibratedAngle; // Parameter: Raw angular offset between encoder index and phase a (Q0)
                Uint32 ReferenceLine;	// *4
                Uint32 CircleLine;		// *4 * ENCODER_CIRCLE
                Uint16 FirstIndexFlag;		//Parameter
                 int16 FirstIndexCnt;		//
                 int32 FirstIndexMechTheta;
                Uint16 ErrFlag;
                Uint16 IndexSyncFlag;
				Uint16 QepType;
                }QEP_F;

extern QEP_F gQepAngle;


void ANGLE_INIT_ALL(void);
void ANGLE_MACRO_ALL(u32 AbsQep,s32 AbsQepInc);

#endif /* MATHANGLENEW_H_ */
