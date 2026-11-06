#include "av_data.h"
 


static u16 val[AD_BUF_LEN];
static u16 val_ptr=0;
static u16 val_num=0;
static u16 val_x[AD_BUF_LEN];
static u16 val_x_ptr=0;
static u16 val_x_num=0;
 
static u16 val_y[AD_BUF_LEN];
static u16 val_y_ptr=0;
static u16 val_y_num=0;
u16 av_buf[AD_BUF_LEN];
void av_array_sort(u16 *src, u16* dst, u8 len, u8 cmp)
{
	u8 i,j,min,max;
	u16 tmp;
	for(i=0;i<len;i++){
		dst[i]=src[i];
	}	
	if(cmp>(len/2)) cmp=len/2;
	for (i=0;i<cmp;i++) {  
		min=i; 
		max=i;
		for (j= i+1; j< len-i; j++) {
			if (dst[j] > dst[max]) { 
				max = j ; 
				continue ; 
			}  
			if (dst[j]< dst[min]) { 
				min = j ; 
			}   
	  }
		if(max==min)
				return;
	  tmp = dst[i];
		dst[i] = dst[min];
		dst[min] = tmp;
	  tmp = dst[len-i-1];
		dst[len-i-1] = dst[max];
		dst[max] = tmp; 
	} 
}

void av_set_data(u8 idx ,u16 value )
{
	 switch (idx)
	 {
		 case 1 :
			 val[val_ptr]=value;
			 val_ptr++;
			 if(val_ptr>=AD_BUF_LEN) val_ptr=0;
			 if(val_num<AD_BUF_LEN){
					val_num++;	
			 }	
			 break;
		 case 2:
			 val_x[val_x_ptr]=value;
			 val_x_ptr++;
			 if(val_x_ptr>=AD_BUF_LEN) val_x_ptr=0;
			 if(val_x_num<AD_BUF_LEN){
					val_x_num++;	
			 }	
			 break;
		 case 3:
			 val_y[val_y_ptr]=value;
			 val_y_ptr++;
			 if(val_y_ptr>=AD_BUF_LEN) val_y_ptr=0;
			 if(val_y_num<AD_BUF_LEN){
					val_y_num++;	
			 }	
			 break;
	 }
}
u16 av_get_data(u8 idx)
{
	u8 i;
	u8 cmp=2; 
	u32 sum=0;	


	 switch (idx)
	 {
		 case 1 :
					if(val_num<AD_BUF_LEN){
					return 0;
					}
					av_array_sort(val, av_buf, AD_BUF_LEN, cmp);
					for(i=cmp;i<(AD_BUF_LEN-cmp);i++)
					{
						sum+=av_buf[i];		
					}
					return sum/(AD_BUF_LEN-2*cmp); 
			break;
		 case 2:
			    if(val_x_num<AD_BUF_LEN){
					return 0;
					}
					av_array_sort(val_x, av_buf, AD_BUF_LEN, cmp);
					for(i=cmp;i<(AD_BUF_LEN-cmp);i++)
					{
						sum+=av_buf[i];		
					}
					return sum/(AD_BUF_LEN-2*cmp); 
			break;
		 case 3:
			    if(val_y_num<AD_BUF_LEN){
					return 0;
					}
					av_array_sort(val_y, av_buf, AD_BUF_LEN, cmp);
					for(i=cmp;i<(AD_BUF_LEN-cmp);i++)
					{
						sum+=av_buf[i];		
					}
					return sum/(AD_BUF_LEN-2*cmp);
			break;
	 }
	 return 0 ;
}
 
