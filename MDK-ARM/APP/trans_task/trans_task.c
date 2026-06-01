/**
  * @file   trans_task.c/.h   
  * @brief  飞镖传送任务
  * @note   舵机
  * @history
  *  Version    Date            Author          Modification
  *  
   */
	 
#include "trans_task.h"
#include "chassis_task.h"
#include "tim.h"

trans_move_t trans_move;
extern chassis_move_t chassis_move;


//平移初始化,获取平移电机数据地址，pid初始化等
void trans_task_init(void);
static void trans_update_data(trans_move_t *trans_move_update);
static void trans_mode_set(trans_move_t  *trans_move_transit);
static void trans_control_set(trans_move_t *trans_move_control_set);
static void trans_control_loop(trans_move_t *trans_move_mode_set);
static void trans_mode_change_transit(trans_move_t *trans_move_control_loop);

void TRANS_RC_Analyse(void);
void TRANS_AUTO_Analyse(void);

void trans_task(void)
{
	
	trans_update_data(&trans_move);
	trans_mode_set(&trans_move);
	trans_mode_change_transit(&trans_move);
	trans_control_set(&trans_move);        
	trans_control_loop(&trans_move);       	
	if(trans_move.trans_mode == TRANS_RELAX)
	{
		CAN_cmd_trans(0,0,0,0);
	}
	else
	{
		CAN_cmd_trans(0,0,trans_move.trans_motor.give_current,0);
	}
}

/****************************************
*              初始化
*****************************************/
void trans_task_init(void)
{
	trans_move.trans_mode = TRANS_RELAX;

   const static float trans_motor_speed_pid[3] = {TRANS_MOTOR_SPEED_PID_KP, TRANS_MOTOR_SPEED_PID_KI, TRANS_MOTOR_SPEED_PID_KD};
	//获取遥控器控制信息
	trans_move.trans_RC = get_remote_control_point();
  trans_move.trans_motor.Trans_Receive_data = Back_Trans_Data();	
	//速度环PID初始化
	PID_Init(&trans_move.trans_motor.trans_motor_speed_pid,PID_POSITION,trans_motor_speed_pid,TRANS_MOTOR_SPEED_PID_MAX_OUT,TRANS_MOTOR_SPEED_PID_MAX_IOUT);	
	trans_move.trans_last_mode = trans_move.trans_mode; 
}

/****************************************
*              数据更新
*****************************************/
static void trans_update_data(trans_move_t *trans_move_update)
{
	
}

/****************************************
*              平移模式设定
*****************************************/
static void trans_mode_set(trans_move_t *trans_move_mode_set)
{
	if(switch_is_up(trans_move_mode_set->trans_RC->rc.s[0]) )
	{
		trans_move_mode_set->trans_mode = TRANS_AUTO; 
	}			
	else if(switch_is_mid(trans_move_mode_set->trans_RC->rc.s[0]) ) 
	{
		trans_move_mode_set->trans_mode = TRANS_RC; 
	}			
	else if(switch_is_down(trans_move_mode_set->trans_RC->rc.s[0]) )
	{
		trans_move_mode_set->trans_mode = TRANS_RELAX; 
	}		
}


/****************************************
*             平移模式改变设定
*****************************************/
static void trans_mode_change_transit(trans_move_t *trans_move_transit)
{
	if(trans_move_transit->trans_mode == trans_move_transit->trans_last_mode)
	{
		return ; 
	}
		
	trans_move_transit->trans_last_mode = trans_move_transit->trans_mode ;
	trans_move_transit->trans_state = TRANS_TASK_NOT;

	PID_clear(&trans_move_transit->trans_motor.trans_motor_speed_pid);
}


/****************************************
*             控制量计算
*****************************************/
static void trans_control_set(trans_move_t *trans_move_control_set)
{ 
	if(trans_move_control_set->trans_mode == TRANS_RELAX)
	{
		
	}
	else if(trans_move_control_set->trans_mode == TRANS_RC)
	{
		TRANS_RC_Analyse();
	}
	else if(trans_move_control_set->trans_mode == TRANS_AUTO)
	{
		TRANS_AUTO_Analyse();
	}
	
}

/*------------------------------------------------------*/	
/*                 	平移状态判断

note:需要修改状态切换时需要保存的参数
*/
/*------------------------------------------------------*/	
static void trans_control_loop(trans_move_t *trans_move_contorl_loop)
{
	
	trans_move_contorl_loop->trans_motor.give_current = PID_Calc(&trans_move_contorl_loop->trans_motor.trans_motor_speed_pid,
																																trans_move_contorl_loop->trans_motor.Trans_Receive_data->rpm,
																																trans_move_contorl_loop->rpm_set);
}


void TRANS_RC_Analyse(void)
{
	
	trans_move.rpm_set  -= trans_move.trans_RC->rc.ch[RIGHT_MOTE_UPANDDOWN] * RC_VAL_TO_RPM_SET;
	if(trans_move.rpm_set  > TRANS_MAX_RPM)
	{
		trans_move.rpm_set  = TRANS_MAX_RPM;
	}
	if(trans_move.rpm_set  < TRANS_MIN_RPM)
	{
		trans_move.rpm_set  = TRANS_MIN_RPM;
	}
	
	if(trans_move.trans_RC->rc.ch[RIGHT_MOTE_LEFTANDRIGHT] >330)
	{
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,2000);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,1050);
	}
	
}

void TRANS_AUTO_Analyse(void)
{
	
}


/****************************************	
*          平移任务状态返回	
*****************************************/	
uint8_t get_trans_model_state(void)
{
    if(trans_move.trans_state == TRANS_TASK_OK)
			return 1;
		else if(trans_move.trans_state == TRANS_TASK_NOT)
			return 0;
		
		return 0;			
}


