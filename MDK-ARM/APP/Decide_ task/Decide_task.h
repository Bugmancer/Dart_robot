#ifndef _DECIDE_TASK_
#define _DECIDE_TASK_

#include "stm32f4xx_hal.h"
#include "remote_Control.h"

typedef enum 
{
	DECIDE_RELAX,
	DECIDE_RC,
	DECIDE_ALONE,
}decide_mode_e;//决策模式

typedef enum  
{
	
}action_mode_e;//

typedef struct
{
	const RC_ctrl_t *decide_RC;
	
	decide_mode_e decide_mode;
	decide_mode_e decide_last_mode;
	
	action_mode_e action_mode;
	action_mode_e action_last_mode;
}decide_move_t;

#endif
