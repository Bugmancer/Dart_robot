#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H


#include "stm32f4xx_hal.h"
#include "pid.h"
#include "Can_receive.h"
#include "chassis_motor_data.h"
#include "remote_control.h"
#include "usart.h"
#include "shoot_task.h"
#include "gpio.h"

/****
@用法 1:进入pid调试状态 0:进入正常模式
****/
#define TEST_MODE 0 //测试模式宏定义

/*------------------------------------------------------------ M3508
#define ECOD_TO_YAW_ANGLE 0.00539568f //编码值转化yaw角度
#define ECOD_TO_PITCH_ANGLE 0.00004884f//编码值转化pitch角度

------------------------------------------------------------*/
//360motor_angle = 4mm
#define POSITION_TO_PLUSE 400.0f
#define MOTOR_ANGLE_TO_PULSE 4.44444445f  //1motor_angle == 4.4444445pluse
#define PULSE_TO_MOTOR_ANGLE 0.225f     	//1pluse == 0.225motorangle

#define REMOTEVAL_TO_PITCH_POSITIONSET 0.0001f //遥控器值转位置设定值
#define REMOTEVAL_TO_YAW_POSITIONSET 0.0001f
#define MAX_YAW_POSITION 850000		//YAW最大位置限定
#define MIN_YAW_POSITION -100000	//YAW最小位置限定

#define MAX_PITCH_POSITION  45.0f		//PITCH最大位置限定
#define MIN_PITCH_POSITION  0//PITCH最小位置限定

#define MAX_ADD_TEMP 4.0f //遥控器模式最大加定位置

#define DEADLINE_OF_CH 10 //遥控器死区限制

#define CONTROL_YAW_DIR(STATE) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,STATE)
#define CONTROL_PITCH_DIR(STATE) HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,STATE)



typedef enum
{
	CHASSIS_AT,
	CHASSIS_NOT,
}chassis_compete_flag;

typedef struct
{
	chassis_compete_flag yaw_compete_flag;
	chassis_compete_flag pitch_compete_flag;	
}chassis_com_flag_t; //角度到位标志 

typedef enum
{
    CHASSIS_RELAX,		   		
    CHASSIS_RC,
		CHASSIS_AUTO,
	  CHASSIS_STOP,		
} chassis_mode_e; //底盘任务枚举

typedef struct
{
	float    position_set;     //期望角度     
	float    last_position_set;//上次期望角度  
	float  	 ref_position;			//当前角度
	uint16_t pluse_cnt;		
	uint16_t pluse_need;
}chassis_motor_t;


typedef  struct
{  
	const RC_ctrl_t* Chassis_remote_realte;
	
	chassis_mode_e    last_chassis_mode;  //上次底盘任务
	chassis_mode_e    chassis_mode; 			//底盘任务
	chassis_com_flag_t 	chassis_compete_flag; //角度到位标志

	chassis_motor_t chassis_YAW_motor;
	chassis_motor_t chassis_PITCH_motor;
	
	float stand_yaw;
	uint8_t stand_flag;
} chassis_move_t;


void chassis_task_Init(void);
void chassis_task(void);

#endif
