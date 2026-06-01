#ifndef _SHOOT_TSAK_
#define _SHOOT_TSAK_

#include "stm32f4xx_hal.h"
#include "remote_control.h"
#include "Can_receive.h"
#include "pid.h"

#define OUTSIDE_MOTOR_SPEED_PID_KP          15.0f
#define OUTSIDE_MOTOR_SPEED_PID_KI          0.0f
#define OUTSIDE_MOTOR_SPEED_PID_KD          0.0f
#define OUTSIDE_MOTOR_SPEED_PID_MAX_OUT     16000.0f
#define OUTSIDE_MOTOR_SPEED_PID_MAX_IOUT    6000.0f 

#define PID1_KP          30.0f
#define PID1_KI          0.0f
#define PID1_KD          0.0f


#define PID2_KP         10.0f
#define PID2_KI          0.0f
#define PID2_KD          0.0f



#define STAND_VOLAT_VAL 0
#define MAX_RMP_3508 9000
//extern uint16_t pulse_cnt;

//****************************************结构体模式/状态
typedef enum
{
	SHOOT_RELAX, //放松
	SHOOT_RC,
	SHOOT_AUTO,  //自动
}Shoot_mode_e; //发射任务模式枚举

typedef enum
{
	SHOOT_AUTO_PREPARE,
	SHOOT_AUTO_READY,
	SHOOT_AUTO_SHOOT,
}Shoot_auto_mode_e;

typedef enum
{
	AUTO_TASK_NOT,
	AUTO_TASK_OK,
}Shoot_auto_state_e;



//***************************************电机数据接收结构体
typedef struct
{
	const Revice_Motro_Data *Shoot_Receive_data;
	PidTypeDef Shoot_Motor_Pid;
  int16_t speed_set;
	int16_t give_current;
}Shoot_Motor_Data_t; //摩擦轮电机数据

typedef struct 
{
	//0前哨站 1基地
	uint16_t first_lv_speed[2]; 
	uint16_t second_lv_speed[2];
	int16_t speed_set[2];	
  float  	yaw_add;
}Shoot_Dart_Data_t;

typedef struct 
{
	const RC_ctrl_t* Shoot_remote_realte;
		
	Shoot_Motor_Data_t Outside_Shoot_Motor_Data[5]; //外侧摩擦轮电机数据接收结构体
	Shoot_Dart_Data_t  Shoot_Dart_Data[6];
	
	uint16_t first_lv_speed;
	uint16_t second_lv_speed;
	int length;
	
	Shoot_mode_e			 shoot_mode;									//发射任务模式
	Shoot_mode_e			 last_shoot_mode;
	Shoot_auto_mode_e  shoot_auto_mode;
	Shoot_auto_mode_e	 last_shoot_auto_mode;
	Shoot_auto_state_e Shoot_auto_state;
	
	
	
	
	int8_t Dart_cnt;
	uint8_t adc_flag;
	uint8_t ammo_flag;
	int shoot_flag; //
	float stand_volat;   //基准电压
	float K_proportion;//比例系数 
	
}Shoot_move_t;

void shoot_task(void);
void shoot_task_Init(void);
int read_flag(void);


#endif 
