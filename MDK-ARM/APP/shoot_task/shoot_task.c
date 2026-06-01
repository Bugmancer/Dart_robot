#include "Shoot_task.h"
#include "stdio.h"
#include "tim.h"
#include "usart.h" //重定向用
#include "gpio.h"
#include "chassis_task.h"
//#include "adc.h"
#include "cmsis_os.h"
/**
  * @file  	shoot_task.c/.h
  * @brief  完成发射架拨弹轮任务    
  * @note   ID1外侧左电机 ID2外侧右电机
  * @history TIM4 CH3(PD14)/CH2(PD13)/CH1(PD12) 上中下供弹舵机 CH4 PD15
  *  Version    Date            Author          Modification
  *  
*/
#define pos 2200
Shoot_move_t Shoot_move;
int16_t ecd_error=0;
	int k=0;
	int t=0;
	int z=0;
	int shoot=0;
		int32_t setrmp;
static void shoot_update_date(void);
static void shoot_mode_set(void);
static void shoot_mode_change_transit(void);
static void shoot_contorl_set(void);
static void shoot_control_loop(void);

static void SHOOT_RC_Analsye(void);
static void SHOOT_RELAX_Analsye(void);
void ammo_change(void);

void Adc_get(void);
void ammo_in(void);
extern int chassis_out;
extern chassis_move_t chassis_move;
#define VOLATE_ARRAY_LEN 10
#define VOLATE_DELET_NUM 2
extern int allow_flag;
#define Len 4

float volate_array[VOLATE_ARRAY_LEN];
float volate_temp;
int shoot_flag=0;
int count=0;
extern int roll_flag;

void shoot_task_Init(void)
{
//	
//	/***************************************************************************************/
//	Shoot_move.stand_volat = 2.26f; //基准电压
//	Shoot_move.K_proportion = 3.0f / 2.26f;
//	//行表示对应4个镖体 列表示对应镖体设定速度 第一列为前哨站 第二列为基地
//	uint16_t first_lv_array[Len][2]     = {{1600,1800},{1600,1800},{1600,1800},{1600,1800}}; //一级摩擦轮
//	uint16_t second_lv_array[Len][2]    = {{1600,1800},{1600,1800},{1600,1800},{1600,1800}}; //二级摩擦轮
//	int16_t  outsid_shoot_array[Len][2] = {{6250,5000},{4800,5000},{6250,5000},{6250,5000}}; //三级摩擦轮
//	float 	 yaw_add_array[Len] = {2,-6,-6,-6}; //偏移量
//  /***************************************************************************************/
//	//镖体控制量初始化
//	for(uint8_t i = 0;i < Len;i++)
//	{
//		for(uint8_t j = 0;j < 2;j++)
//		{
//			Shoot_move.Shoot_Dart_Data[i].first_lv_speed[j]  = first_lv_array[i][j];
//			Shoot_move.Shoot_Dart_Data[i].second_lv_speed[j] = second_lv_array[i][j];
//			Shoot_move.Shoot_Dart_Data[i].speed_set[j]       = outsid_shoot_array[i][j]; 	
//			Shoot_move.Shoot_Dart_Data[i].yaw_add 					 = yaw_add_array[i];
//		}
//	}
//	
//	
//	float Outside_motor_Pid_array[3] = {OUTSIDE_MOTOR_SPEED_PID_KP,OUTSIDE_MOTOR_SPEED_PID_KI,OUTSIDE_MOTOR_SPEED_PID_KD};
//	PID_Init(&Shoot_move.Outside_Shoot_Motor_Data[0].Shoot_Motor_Pid,PID_POSITION,Outside_motor_Pid_array,OUTSIDE_MOTOR_SPEED_PID_MAX_OUT,OUTSIDE_MOTOR_SPEED_PID_MAX_IOUT);
//	PID_Init(&Shoot_move.Outside_Shoot_Motor_Data[1].Shoot_Motor_Pid,PID_POSITION,Outside_motor_Pid_array,OUTSIDE_MOTOR_SPEED_PID_MAX_OUT,OUTSIDE_MOTOR_SPEED_PID_MAX_IOUT);
// 
	//拱弹6020双环pid   pid3--位置环    pid4--速度环
  float PID3[3] = {PID1_KP,PID1_KI,PID1_KD};
float PID4[3]= {PID2_KP,PID2_KI,PID2_KD};
	PID_Init(&Shoot_move.Outside_Shoot_Motor_Data[3].Shoot_Motor_Pid,PID_POSITION,	PID3,	8000,5000);
  PID_Init(&Shoot_move.Outside_Shoot_Motor_Data[4].Shoot_Motor_Pid,PID_POSITION,	PID4,	8000,5000);

	
	
	
	
	Shoot_move.Shoot_remote_realte = get_remote_control_point(); //遥控器指针关联
	
	Shoot_move.Outside_Shoot_Motor_Data[0].Shoot_Receive_data = Back_OutsideShoot_Data(0);
	Shoot_move.Outside_Shoot_Motor_Data[1].Shoot_Receive_data = Back_OutsideShoot_Data(1);
	Shoot_move.shoot_mode = SHOOT_RELAX;
	
	Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data = Back_Chassis_pitch_Data();
	
	
	
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,Shoot_move.first_lv_speed);
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,Shoot_move.first_lv_speed);
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,Shoot_move.second_lv_speed);
		
}


void shoot_task(void)
{	
	
	
	shoot_update_date();
	shoot_mode_set();
	shoot_mode_change_transit();
	shoot_contorl_set();
	shoot_control_loop();
	ammo_change();
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	if(chassis_move.chassis_mode == CHASSIS_RELAX)
	{		
		//CAN_cmd_shoot(Shoot_move.Outside_Shoot_Motor_Data[0].give_current,Shoot_move.Outside_Shoot_Motor_Data[1].give_current,0,0);
		CAN_cmd_trans(0,0,0,0);
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,0);
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,0);
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,0);
		
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,0);
	
	}
	else
	{
		//CAN_cmd_shoot(Shoot_move.Outside_Shoot_Motor_Data[0].give_current,Shoot_move.Outside_Shoot_Motor_Data[1].give_current,0,0);
     CAN_cmd_trans(Shoot_move.Outside_Shoot_Motor_Data[4].give_current/2,chassis_out/5,0,0);		
	
//		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,Shoot_move.first_lv_speed);
//		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,Shoot_move.first_lv_speed);
//		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,Shoot_move.second_lv_speed);
//		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,Shoot_move.second_lv_speed);	
		
		//__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,1000);

	}
	
//		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,Shoot_move.first_lv_speed);
//		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,Shoot_move.first_lv_speed);
//		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,Shoot_move.second_lv_speed);
//		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,Shoot_move.second_lv_speed);	



}



static void shoot_update_date(void)
{
	if(Shoot_move.last_shoot_mode != Shoot_move.shoot_mode)
	{
		//Adc_get();
	}
//	if(Shoot_move.current_volat > Shoot_move.stand_volat)
//		Shoot_move.current_volat = Shoot_move.stand_volat;
	Shoot_move.last_shoot_mode = Shoot_move.shoot_mode;
}

static void shoot_mode_set(void)
{
	if(Shoot_move.Shoot_remote_realte-> rc.s[0] == RC_SW_UP)
	{
		Shoot_move.shoot_mode = SHOOT_AUTO;
	}
	else if(Shoot_move.Shoot_remote_realte-> rc.s[0] == RC_SW_MID)
	{
		Shoot_move.shoot_mode = SHOOT_RC;
	}
	else if(Shoot_move.Shoot_remote_realte-> rc.s[0] == RC_SW_DOWN)
	{
		Shoot_move.shoot_mode = SHOOT_RELAX;
	}	
}

static void shoot_mode_change_transit(void)
{
	if(Shoot_move.last_shoot_mode == Shoot_move.shoot_mode)
	{
		return;
	}	
	for(uint8_t i = 0;i < 4;i++)
	{
		PID_clear(&Shoot_move.Outside_Shoot_Motor_Data[i].Shoot_Motor_Pid);
	}
	
}	

static void shoot_contorl_set(void)
{
	if(Shoot_move.shoot_mode == SHOOT_RC)
	{
		SHOOT_RC_Analsye();
	}
	else if(Shoot_move.shoot_mode == SHOOT_RELAX)
	{
		SHOOT_RELAX_Analsye();
	}
	else if(Shoot_move.shoot_mode == SHOOT_AUTO)
	{
		static uint8_t rpm_temp;
		
			if(Shoot_move.Shoot_remote_realte->rc.ch[LEFT_MOTE_UPANDDOWN] > 200)
			{
				if(!rpm_temp)
				{
					Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].speed_set[0] += 50;
					rpm_temp = 1;
				}
			}
			else if(Shoot_move.Shoot_remote_realte->rc.ch[LEFT_MOTE_UPANDDOWN] < -200)
			{
				if(!rpm_temp)
				{
					Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].speed_set[0] -= 50;
					rpm_temp = 1;
				}
			}
			else
			{
				rpm_temp = 0;
			}
			
		
	}
	
	
}

static void shoot_control_loop(void)
{
//	Shoot_move.Outside_Shoot_Motor_Data[0].give_current = 																							
//	PID_Calc(&Shoot_move.Outside_Shoot_Motor_Data[0].Shoot_Motor_Pid,
//	Shoot_move.Outside_Shoot_Motor_Data[0].Shoot_Receive_data->rpm, Shoot_move.Outside_Shoot_Motor_Data[0].speed_set);//外侧左电机电流值赋予
//	
//	Shoot_move.Outside_Shoot_Motor_Data[1].give_current = 
//	PID_Calc(&Shoot_move.Outside_Shoot_Motor_Data[1].Shoot_Motor_Pid,																	
//	Shoot_move.Outside_Shoot_Motor_Data[1].Shoot_Receive_data->rpm, Shoot_move.Outside_Shoot_Motor_Data[1].speed_set);//外侧右电机电流值赋予	
//	
	if(Shoot_move.ammo_flag)
	{
	//	ammo_in();
	//	Shoot_move.Dart_cnt++;
	}
	
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_All,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG,0x0002 << Shoot_move.Dart_cnt,GPIO_PIN_RESET);
}

/*******************模式解析******************************/
static void SHOOT_RC_Analsye(void)
{
	if(Shoot_move.Shoot_remote_realte->rc.s[1] == RC_SW_UP)
	{
//		Shoot_move.first_lv_speed = Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].first_lv_speed[1];
//		Shoot_move.second_lv_speed = Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].second_lv_speed[1];
//		Shoot_move.Outside_Shoot_Motor_Data[0].speed_set = -Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].speed_set[1]
//																											* Shoot_move.K_proportion * Shoot_move.stand_volat / Shoot_move.current_volat;
//		Shoot_move.Outside_Shoot_Motor_Data[1].speed_set = Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].speed_set[1]
//																											* Shoot_move.K_proportion * Shoot_move.stand_volat / Shoot_move.current_volat;
	}
	else if(Shoot_move.Shoot_remote_realte->rc.s[1] == RC_SW_MID)
	{
//			Shoot_move.first_lv_speed = Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].first_lv_speed[0];
//		Shoot_move.second_lv_speed = Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].second_lv_speed[0];
//		Shoot_move.Outside_Shoot_Motor_Data[0].speed_set = -Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].speed_set[0]
//		 - Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].speed_set[0] * Shoot_move.K_proportion * ( 1 -  Shoot_move.current_volat / Shoot_move.stand_volat );
//		Shoot_move.Outside_Shoot_Motor_Data[1].speed_set = Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].speed_set[0] 
//		 + Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].speed_set[0] * Shoot_move.K_proportion * ( 1 -  Shoot_move.current_volat/ Shoot_move.stand_volat  );
	}
	else if(Shoot_move.Shoot_remote_realte->rc.s[1] == RC_SW_DOWN)
	{
//		Shoot_move.first_lv_speed = 0;
//		Shoot_move.second_lv_speed = 0;
//		Shoot_move.Outside_Shoot_Motor_Data[0].speed_set = 0;
//		Shoot_move.Outside_Shoot_Motor_Data[1].speed_set = 0;
	}
	
//	static uint8_t temp; //通过摇杆确定飞镖号
//	if(Shoot_move.Shoot_remote_realte->rc.ch[RIGHT_MOTE_UPANDDOWN] > 330) //摇杆上移飞镖号加一
//	{
//		if(temp)
//		{
//			Shoot_move.Dart_cnt++;
//			
//			count++;
//			temp = 0;
//		}
//	}
//	else if(Shoot_move.Shoot_remote_realte->rc.ch[RIGHT_MOTE_UPANDDOWN] < -330) //摇杆下移飞镖号减一
//	{
//		if(temp)
//		{
//		  Shoot_move.Dart_cnt--;
//			count--;
//			temp = 0;
//		}
//	}
//	else  
//	{
//		temp = 1;
//		if(Shoot_move.Dart_cnt > Len - 1) //飞镖号规整
//			Shoot_move.Dart_cnt = 0;
//		else if(Shoot_move.Dart_cnt < 0)
//			Shoot_move.Dart_cnt = Len - 1;
//	}
	
	if(Shoot_move.Shoot_remote_realte->rc.ch[4] > 200&&roll_flag==0) 
	{
		Shoot_move.ammo_flag = 1;
	}
	if(Shoot_move.Shoot_remote_realte->rc.ch[4] > 200&&roll_flag==1) 
	{
		//if(temp1)
		//{
		//	Shoot_move.Dart_cnt = 0;
		//}
		//temp1 = 1;
		if(Shoot_move.ammo_flag ==1){
		Shoot_move.ammo_flag =0;
		Shoot_move.Dart_cnt++;
			count++;
		
		}
		
		
		
	}
	else
	{
		//temp1 = 0;
	}
	
	

	
	if(Shoot_move.Shoot_remote_realte->rc.ch[RIGHT_MOTE_LEFTANDRIGHT] > 200)
	{
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,2200);
	}
	else
	{
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,800);
	}
	
	
	Shoot_move.adc_flag = 1;
	
}


static void SHOOT_RELAX_Analsye(void)
{
	if(Shoot_move.Shoot_remote_realte->rc.ch[RIGHT_MOTE_UPANDDOWN] > 20)
	{
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,500); 
	}
	else
	{
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,2000);
	}
	if(Shoot_move.Shoot_remote_realte->rc.ch[RIGHT_MOTE_LEFTANDRIGHT] < -20)
	{
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,500);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,1800);
	}
//	if(Shoot_move.Shoot_remote_realte->rc.ch[RIGHT_MOTE_UPANDDOWN] <  -20)
//	{
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);
//	}
//	else
//	{
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,2000);
//	}
}


/***************************AUTO模式解析******************************/


void ammo_in(void)
{
	
	
	
//	if(Shoot_move.Dart_cnt == 0){
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);
//	vTaskDelay(700);
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,2000);
//	vTaskDelay(700);
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,500);
//	vTaskDelay(700);
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,1800);
//	vTaskDelay(700);
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,500);
//	vTaskDelay(700);
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,2000);
//	vTaskDelay(700);
//  Shoot_move.ammo_flag = 0;
//	}
//	else if(Shoot_move.Dart_cnt == 1)
//	{
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);
//	vTaskDelay(700);
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,2000);
//	vTaskDelay(700);
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,500);
//	vTaskDelay(700);
//	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,2000);
//	vTaskDelay(700);
//		Shoot_move.ammo_flag = 0;
//	}
//	else if(Shoot_move.Dart_cnt == 2)
//	{
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);
//		vTaskDelay(700);
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,2000);
//		vTaskDelay(700);
//		Shoot_move.ammo_flag = 0;
//	}
//	else
		return;
}

void ammo_change(void)
{
	

	int16_t Target_speed;
	Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data = Back_Chassis_pitch_Data();
   ecd_error = Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->ecd-k;
       k=Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->last_ecd;
//	  ecd_error = ecd_error >   ?   ecd_error - 8192: ecd_error;
//    ecd_error = ecd_error < -4096 ?   ecd_error + 8192 : ecd_error;
//	    if(ecd_error>8180){k=1;}
//			if(ecd_error>4150&&k==1){k=0;}
//			if(ecd_error<4150&&k==1){t++;k=0;}
//	   if(ecd_error<20){z=1;}
//			if(ecd_error>4150&&z==1){z=0;}
//			if(ecd_error<4150&&z==1){t--;z=0;}
	
	if(ecd_error<4150&&ecd_error>-4150){z=1;}
	if(ecd_error<-4150&&z==1){
	
	t--;z=0;}
	if(ecd_error>4150&&z==1){
	
	t++;z=0;}
	
		Shoot_move.length = -Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->ecd+t*8192;
	
	
	
	
	
    PID_clear(&Shoot_move.Outside_Shoot_Motor_Data[3].Shoot_Motor_Pid);
	   PID_clear(&Shoot_move.Outside_Shoot_Motor_Data[4].Shoot_Motor_Pid);
	if(allow_flag==1)
	{setrmp=count*2048+pos-1854+1024*Shoot_move.ammo_flag;}
//	setrmp=-setrmp-60;
//     double speed=shoot->speed_rpm*0.000081799f;
//   L=pos_cul(shoot);;
	
 Target_speed=PID_Calc(&Shoot_move.Outside_Shoot_Motor_Data[3].Shoot_Motor_Pid,Shoot_move.length,setrmp);
    if((Shoot_move.length-setrmp)*(Shoot_move.length-setrmp)<10)
	{	
		Shoot_move.Outside_Shoot_Motor_Data[4].give_current=PID_Calc(&Shoot_move.Outside_Shoot_Motor_Data[4].Shoot_Motor_Pid,Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->rpm,0);
//	if(Shoot_move.ammo_flag==1) 
//     {shoot_flag=1;}
//		else {shoot_flag=0;}
		if(Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->rpm*Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->rpm<9&&Shoot_move.ammo_flag==1)
	{Shoot_move.shoot_flag=1;}
else
{Shoot_move.shoot_flag=0;}
	}
	else
	{
	  Shoot_move.Outside_Shoot_Motor_Data[4].give_current=PID_Calc(&Shoot_move.Outside_Shoot_Motor_Data[4].Shoot_Motor_Pid,Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->rpm,-Target_speed);

		Shoot_move.shoot_flag=3;
		
  }
 if((Shoot_move.length-setrmp)*(Shoot_move.length-setrmp)<1500)
{	
//		Shoot_move.Outside_Shoot_Motor_Data[4].give_current=PID_Calc(&Shoot_move.Outside_Shoot_Motor_Data[4].Shoot_Motor_Pid,Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->rpm,0);
//	if(Shoot_move.ammo_flag==1) 
//     {shoot_flag=1;}
//		else {shoot_flag=0;}
		if(Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->rpm*Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->rpm<400&&Shoot_move.ammo_flag==1)
	Shoot_move.shoot_flag=1;
else
{Shoot_move.shoot_flag=0;}
	}
	else
	{
//	  Shoot_move.Outside_Shoot_Motor_Data[4].give_current=PID_Calc(&Shoot_move.Outside_Shoot_Motor_Data[4].Shoot_Motor_Pid,Shoot_move.Outside_Shoot_Motor_Data[2].Shoot_Receive_data->rpm,-Target_speed);

		Shoot_move.shoot_flag=3;
		
  }
	
	
	
	
}
int read_flag(){
	
	return Shoot_move.shoot_flag;
}
