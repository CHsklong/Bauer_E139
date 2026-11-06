/*
 * svgen_f.h
 *
 *  Created on: 2013-9-29
 *      Author: Administrator
 */

#ifndef SVGEN_F_H_
#define SVGEN_F_H_


typedef struct 	{ float  Ualpha; 			// Input: reference alpha-axis phase voltage
				  float  Ubeta;			// Input: reference beta-axis phase voltage
				  float  Ta;				// Output: reference phase-a switching function
				  float  Tb;				// Output: reference phase-b switching function
				  float  Tc;				// Output: reference phase-c switching function
				  float  tmp1;			// Variable: temp variable
				  float  tmp2;			// Variable: temp variable
				  float  tmp3;			// Variable: temp variable
				  float VecSector;		// Space vector sector
				} SVGEN_CLA;




/*------------------------------------------------------------------------------
	Space Vector  Generator (SVGEN) Macro Definition
------------------------------------------------------------------------------*/

#if 1
#define SVGEN_CLA_MACRO(v)														\
	v.tmp1= v.Ubeta;															\
	v.tmp2= v.Ubeta*0.5f + ((0.866f)*v.Ualpha);									\
    v.tmp3= v.tmp2 - v.tmp1;													\
																				\
	v.VecSector=3;																\
	if(v.tmp2> 0)v.VecSector--;													\
	if(v.tmp3> 0)v.VecSector--;													\
	if(v.tmp1< 0)v.VecSector=7-v.VecSector;										\
	/*v.VecSector=(v.tmp2> 0)?( v.VecSector-1):v.VecSector;*/					\
	/*v.VecSector=(v.tmp3> 0)?( v.VecSector-1):v.VecSector;*/					\
	/*v.VecSector=(v.tmp1< 0)?(7-v.VecSector) :v.VecSector;*/					\
																				\
	if     (v.VecSector==1 || v.VecSector==4)                                   \
      {     v.Ta= v.tmp2; 														\
      		v.Tb= v.tmp1-v.tmp3; 												\
      		v.Tc=-v.tmp2;														\
      }								    										\
   																				\
    else if(v.VecSector==2 || v.VecSector==5)                                   \
      {     v.Ta= v.tmp3+v.tmp2; 												\
      		v.Tb= v.tmp1; 														\
      		v.Tc=-v.tmp1;														\
      }																	   		\
   																				\
    else                                                                        \
      {     v.Ta= v.tmp3; 														\
      		v.Tb=-v.tmp3; 														\
      		v.Tc=-(v.tmp1+v.tmp2);												\
      }																	   		\

#endif


#define SVGEN_CLA_INIT_MACRO(v)	\
	  v.  Ualpha=0; 		\
	  v.  Ubeta=0;			\
	  v.  Ta=0;				\
	  v.  Tb=0;				\
	  v.  Tc=0;				\
	  v.  tmp1=0;			\
	  v.  tmp2=0;			\
	  v.  tmp3=0;			\
	  v. VecSector=0;		\


#endif /* SVGEN_F_H_ */
