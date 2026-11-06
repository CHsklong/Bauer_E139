#ifndef _LADRC_H
#define _LADRC_H

typedef struct LADRC
{
	  float P;//feedback KP value
    float h;//time step size          
    float z1;//observation value of system output
		float z2;//disturbance observation value     
    float wo;//observer bandwidth
		float b0;//control gain
	  float limit; 
		float u; //controller output  
}LADRC_Obj;

typedef struct TD
{
		float h;//time step size
		float r;//transition parameter
	  float x1;//transition value
	  float x2;//derivative of the transition value
	  float limit;
}TD_Obj;

typedef struct PI_struct
{
	  float kp;
    float ki;        
    float integration;
		float limit;   
		float u; //controller output  
}PI_Obj;
extern TD_Obj td_set_spd;
extern TD_Obj fact_spd;
void LADRC_Init(LADRC_Obj *adrc,float h,float Kp,float b0,float wo,float limit);
void  LADRC_Reset(LADRC_Obj *adrc);
float LADRC_Controller(LADRC_Obj *adrc,float setpoint,float feedback);
void  TD_Init(TD_Obj *td,float h,float r,float limit);
void  TD(TD_Obj *td,float x);
void TD_Reset(TD_Obj *td);
float  PI_Control(PI_Obj *pi,float setpoint,float feedback);
void PI_Init(PI_Obj *pi,float kp,float ki,float limit);
void NLTD(TD_Obj *td,float x);
#endif
