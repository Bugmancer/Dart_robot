#ifndef SWITCH_H
#define SWITCH_H
#include "struct_typedef.h"


typedef struct
{
   int guangdian_flag;//光电标志
   int grigger_flag;
	 int step1;
	 int step2;
	 int step3;
	 int step4;
	 int count;//遮挡次数
}switch_flag;



void guangdianint(switch_flag* flag);
int guangdian(switch_flag *flag);
void touch(void);
const switch_flag *flag_rtos(switch_flag* flag);
#endif

