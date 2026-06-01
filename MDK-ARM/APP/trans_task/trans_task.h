#ifndef _TRANS_TASK_
#define _TRANS_TASK_


#include "remote_Control.h"
#include "user_lib.h"
#include "pid.h"
#include "Can_receive.h"


//平移电机速度环PID  &&&&!&&&&
#define TRANS_MOTOR_SPEED_PID_KP       8.0f
#define TRANS_MOTOR_SPEED_PID_KI       1.0f
#define TRANS_MOTOR_SPEED_PID_KD       0.0f
#define TRANS_MOTOR_SPEED_PID_MAX_OUT  16000.0f
#define TRANS_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//平移最大速度
#define TRANS_MAX_RPM									 0
#define TRANS_MIN_RPM 								 -9000

#define RC_VAL_TO_RPM_SET 						 0.005f
typedef enum
{
	TRANS_RELAX,
	TRANS_RC,
	TRANS_AUTO,
	TRANS_STOP,	
}trans_mode_e;

typedef enum
{
	TRANS_TASK_OK,
	TRANS_TASK_NOT,
}trans_state_e;

typedef struct
{
	const Revice_Motro_Data *Trans_Receive_data;	 //关联接收数据指针
	
	PidTypeDef trans_motor_position_pid;      //平移电机位置环pid     
	PidTypeDef trans_motor_speed_pid;         //平移电机速度环pid
	int64_t  ecd_sum;
	int16_t  last_ecd;
	int16_t  give_current;  
  
} Trans_motor_t; //电机数据结构体


typedef struct 
{
	const RC_ctrl_t *trans_RC;                   //平移使用的遥控器指针
	
	
	Trans_motor_t trans_motor;
	
	
	trans_mode_e 			trans_mode;
	trans_mode_e 			trans_last_mode;
	trans_state_e 		trans_state;
	
	int16_t		 rpm_set;
} trans_move_t;

void trans_task(void);
void trans_task_init(void);
void trans_task_position_set(float height);
uint8_t get_trans_model_state(void);



#endif 
