#include "LADRC.h"
#include <math.h>
LADRC_Obj spd;
TD_Obj td_set_spd;
TD_Obj fact_spd;
PI_Obj PI_spd;
TD_Obj set_NLTD;

// Tracking differentiator initialization
void TD_Init(TD_Obj *td, float h, float r, float limit)
{
	td->h = h;
	td->r = r;
	td->limit = limit;
}

void TD_Reset(TD_Obj *td)
{
	td->x1 = 0;
	td->x2 = 0;
}
// The function of the tracking differentiator is to arrange the transition process
// x is the target value.
// transition time is 7/r
void TD(TD_Obj *td, float x)
{
	float fh;
	fh = -td->r * td->r * (td->x1 - x) - 2 * td->r * td->x2;
	td->x1 = td->x1 + td->h * td->x2;
	td->x2 = td->x2 + td->h * fh;
	if (td->x2 > td->limit)
		td->x2 = td->limit;
	else if (td->x2 < -td->limit)
		td->x2 = -td->limit;
}

void LADRC_Init(LADRC_Obj *adrc, float h, float Kp, float b0, float wo, float limit)
{
	adrc->h = h;
	adrc->P = Kp;
	adrc->b0 = b0;
	adrc->wo = wo;
	adrc->z1 = 0;
	adrc->z2 = 0;
	adrc->u = 0;
	adrc->limit = limit;
}

void LADRC_Reset(LADRC_Obj *adrc)
{
	adrc->z1 = 0;
	adrc->z2 = 0;
	adrc->u = 0;
}

float LADRC_Controller(LADRC_Obj *adrc, float setpoint, float feedback)
{
	float beta01, beta02;
	float error = setpoint - feedback;
	float error1 = adrc->z1 - feedback;
	beta01 = 2 * adrc->wo;
	beta02 = (adrc->wo) * (adrc->wo);
	adrc->z1 = adrc->z1 + adrc->h * (adrc->z2 - beta01 * error1 + adrc->b0 * adrc->u);
	//	if(((fact_spd.x2<10*error)&&(error1>0))||((fact_spd.x2>10*error)&&(error1<0)))
	adrc->z2 = adrc->z2 + adrc->h * (-beta02 * error1);
	//	if((setpoint*adrc->z2)>0)
	//		adrc->z2=0;
	adrc->u = adrc->P * error - adrc->z2 / adrc->b0;
	if (adrc->u > adrc->limit)
		adrc->u = adrc->limit;
	else if (adrc->u < -adrc->limit)
		adrc->u = -adrc->limit;
	return (adrc->u);
}

float PI_Control(PI_Obj *pi, float setpoint, float feedback)
{
	float max_integration;
	float error = setpoint - feedback;
	if (pi->ki == 0)
		pi->integration = 0;
	else
	{
		if (((fact_spd.x2 < 10 * error) && (error > 0)) || ((fact_spd.x2 > 10 * error) && (error < 0)))
			pi->integration += error;
		max_integration = pi->limit / pi->ki;
		if (pi->integration > max_integration)
			pi->integration = max_integration;
		else if (pi->integration < -max_integration)
			pi->integration = -max_integration;
	}
	//		 pi->integration=0;
	pi->u = pi->ki * pi->integration + pi->kp * error;
	if (pi->u > pi->limit)
		pi->u = pi->limit;
	else if (pi->u < -pi->limit)
		pi->u = -pi->limit;
	return (pi->u);
}

void PI_Init(PI_Obj *pi, float kp, float ki, float limit)
{
	pi->kp = kp;
	pi->ki = ki;
	pi->limit = limit;
}

static float sign(float x)
{
	if (x > 0)
		return (1);
	else if (x < 0)
		return (-1);
	else
		return (0);
}

static float fsg(float x, float d)
{
	float u;
	u = (sign(x + d) - sign(x - d)) / 2;
	return (u);
}

static float fhan(float e, float v, float r, float h)
{
	float d, a0, y, a1, a2, a, u;
	d = r * h * h;
	a0 = h * v;
	y = e + a0;
	a1 = sqrt(d * (d + 8 * fabs(y)));
	a2 = a0 + sign(y) * (a1 - d) / 2;
	a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
	u = -r * (a / d) * fsg(a, d) - r * sign(a) * (1 - fsg(a, d));
	return (u);
}
/*
static float fhan1(float e, float v, float r, float h)
{
	float d, a0, y, a, u, z, sy, sz;
	d = r * h * h;
	a0 = h * v;
	y = e + a0;
	z = y + a0;
	a = y + a0 * (fabs(a0) / d - 1) / 2;
	sy = (sign(y - d) - sign(y + d)) / 2;
	sz = (sign(z - d) - sign(z + d)) / 2;
	u = -r * ((z - sign(z) - sign(a)) * sy * sz + sign(z) + sign(a));
	return (u);
}*/

void NLTD(TD_Obj *td, float x)
{
	float fh;
	fh = fhan(td->x1 - x, td->x2, td->r, td->h);
	td->x1 = td->x1 + td->h * td->x2;
	td->x2 = td->x2 + td->h * fh;
	if (td->x2 > td->limit)
		td->x2 = td->limit;
	else if (td->x2 < -td->limit)
		td->x2 = -td->limit;
}