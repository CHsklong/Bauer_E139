

#ifndef FILTER_H_
#define FILTER_H_


typedef struct {
	float	Xn ;
	float	Xn_1;
	float	Yn;
	float	Yn_1;
	float	Kn;
	float 	Kn_1;
	float	Km_1;


}FILTER_TYPE_1;

extern FILTER_TYPE_1 gFilter1;
extern FILTER_TYPE_1 gFilterId,gFilterIq, gFilterIU,gFilterIV;




void filter_K_cal(FILTER_TYPE_1 *dp);
void filter_step1(FILTER_TYPE_1 *ptr);


void filter_K_idiq(FILTER_TYPE_1 *dp);
void filter_K_zl(FILTER_TYPE_1 *dp);
void filter_K_gan(FILTER_TYPE_1 *dp);

#endif /* FILTER_H_ */

