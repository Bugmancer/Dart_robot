#include "switch.h"
#include "main.h"

#define guangdian_gpio  GPIOI
#define guangdian_num   GPIO_PIN_0

int count_1=0;//计数
int test=0;
//光电初始化
void guangdianint(switch_flag* flag)
{
     flag->guangdian_flag=0;
     flag->count=0;				
}

//光电判断
int guangdian(switch_flag* flag)
{   
	
   if(HAL_GPIO_ReadPin(guangdian_gpio, guangdian_num )==1)
     { 
			 flag->guangdian_flag=1;
		 
             if(test==0)
			 {
				  flag->count++;
			 }
	 } 
	else
     {
		 flag->guangdian_flag=0;		         
	  }
		 test=flag->guangdian_flag;
		 
		 return flag->count;

}






