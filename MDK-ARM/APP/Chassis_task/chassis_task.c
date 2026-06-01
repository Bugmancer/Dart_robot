#include "chassis_task.h"
#include "stdio.h"
#include "user_lib.h"
#include "trans_task.h"
#include "tim.h"
#include "stdlib.h"
#include "math.h"
#include "cmsis_os.h"

/**
  * @file     chassis_task.c/.h
  * @brief    完成发射架YAW轴,PITCH轴,电机转动
  * @note     PITCH PI5-PWM信号(W) PI7-方向(Y)
							YAW 	PA3-PWM信号(V) PI6-方向(X)
  * @history
  *  Version    Date            Author          Modification
  *  
  */
fp32 Pid2[3]={11,0,0};
fp32 Pid1[3]={1000.0f,0,0};//9000 5 0
chassis_move_t chassis_move;
Shoot_move_t Shoot_move_c;
extern Shoot_move_t Shoot_move;
PidTypeDef pid1;
PidTypeDef pid2;
	int k2=0;
	int t1=0;
	int z1=0;
  int setrmp1;
int ctrl_out;
int LENGTH;
int finish;
int finish1;
int finish2;
int last;
int once;
const Revice_Motro_Data* Shoot;
int flag_once=0;
int chassis_out;
int test3;
int pos[4]={1000000,1000000,1000000,1000000};
int16_t Target_speed1;
int firstflag=0;
int saveflag;
#define distance1 2000000
#define distance2 0
#define transform 0.000059911f
#define distance 1000000
//遥控器死区限制
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

static void chassis_update(void);
static void chassis_mode_set(void);
static void chassis_contorl_set(void);
static void chassis_control_loop(void);
		
static void CHASSIS_RC_Analyse(void); //遥控器模式数据解析
static void CHASSIS_AUTO_Analyse(void);
		
/*底盘任务初始化*/
void chassis_task_Init(void)
{
	chassis_move.Chassis_remote_realte = get_remote_control_point();			 //遥控器数据关联
	
	chassis_move.chassis_compete_flag.yaw_compete_flag = CHASSIS_NOT; //完成标志初始化
	chassis_move.chassis_compete_flag.pitch_compete_flag = CHASSIS_NOT;
	

	chassis_move.chassis_YAW_motor.position_set   = 0;//31.8013954 22.3097839f;// /**YAW初始化**/
	chassis_move.chassis_PITCH_motor.position_set = 0;//15.4415026 23.8211021f; /**PITCH初始化**/
	
	chassis_move.chassis_mode = CHASSIS_RELAX; //底盘任务模式设定
	PID_clear(&pid1);
	   PID_clear(&pid2);
	PID_Init(&pid1,PID_POSITION,	Pid1,	10000,5000);
	PID_Init(&pid2,PID_POSITION,	Pid2,	16000,5000);
	
}
int chassis_2006(){


	
	int ecd_error;
	Shoot= Back_Chassis_yaw_Data();
   ecd_error = Shoot->ecd-last;
      last=Shoot->last_ecd;
//	  ecd_error = ecd_error >   ?   ecd_error - 8192: ecd_error;
//    ecd_error = ecd_error < -4096 ?   ecd_error + 8192 : ecd_error;
//	    if(ecd_error>8180){k=1;}
//			if(ecd_error>4150&&k==1){k=0;}
//			if(ecd_error<4150&&k==1){t++;k=0;}
//	   if(ecd_error<20){z=1;}
//			if(ecd_error>4150&&z==1){z=0;}
//			if(ecd_error<4150&&z==1){t--;z=0;}
	
	if(ecd_error<4150&&ecd_error>-4150){z1=1;}
	if(ecd_error<-4150&&z1==1){
	t1--;z1=0;}
	if(ecd_error>4150&&z1==1){
	t1++;z1=0;}
//		if(once==0){t1=0;}
		LENGTH = -Shoot->ecd+t1*8192;
	
	
	
//	
//	if(once==0){

//	
//	once=-Shoot->ecd;
//	}
//	
	LENGTH=LENGTH-once;
	
    PID_clear(&pid1);
	   PID_clear(&pid2);
	setrmp1=chassis_move.chassis_YAW_motor.position_set/transform;
	

	
//	
//// Target_speed1=PID_Calc(&pid2,LENGTH,setrmp1)/5;
////    if((LENGTH-setrmp1)*(LENGTH-setrmp1)<4000000)
////	{	
//////		ctrl_out=PID_Calc(&pid1,Shoot->rpm,0);
////ctrl_out=0;
////		finish=1;
////	}
////	else
////	{
//////	  ctrl_out=PID_Calc(&pid1,Shoot->rpm,-Target_speed1);

//////		finish=0;
////	
// Target_speed1=PID_Calc(&pid2,LENGTH,setrmp1);
//    if((LENGTH-setrmp1)*(LENGTH-setrmp1)<4000000)
//	{	
////		ctrl_out=PID_Calc(&pid1,Shoot->rpm,0);
//ctrl_out=0;
//		finish=1;
//	}
//if(Target_speed1>0&&finish==0){ctrl_out=PID_Calc(&pid1,Shoot->rpm,-4000);}
//if(Target_speed1<0&&finish==0){ctrl_out=PID_Calc(&pid1,Shoot->rpm,4000);}
if(finish==0){
if(finish1==0){ctrl_out=PID_Calc(&pid1,Shoot->rpm,2500);}
if(finish1==1&&HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)==0&&finish2==0){
if(LENGTH<distance)ctrl_out=PID_Calc(&pid1,Shoot->rpm,-2500);else{ctrl_out=PID_Calc(&pid1,Shoot->rpm,0);finish=1;}}
if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)==1&&firstflag==0){firstflag=1; 
saveflag=uwTick;}
if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)==1&&uwTick-saveflag>500){finish1=1;	ctrl_out=PID_Calc(&pid1,Shoot->rpm,-2500);once=-Shoot->ecd;t1=0;LENGTH=0;}
test3=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12);
if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)==0){firstflag=0;saveflag=uwTick; }

}
	else{finish2=1;}
  if(finish2==1&&chassis_move.Chassis_remote_realte->rc.s[1] == RC_SW_MID){
if(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_LEFTANDRIGHT]>200&&LENGTH<distance1){ctrl_out=PID_Calc(&pid1,Shoot->rpm,-2500);}

else if(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_LEFTANDRIGHT]<-200&&LENGTH>distance2){ctrl_out=PID_Calc(&pid1,Shoot->rpm,2500);}
else{ctrl_out=PID_Calc(&pid1,Shoot->rpm,0);}
}
// if(finish2==1&&chassis_move.Chassis_remote_realte-> rc.s[1] == RC_SW_UP){
//	
// if(LENGTH<pos[k-1]-10){ctrl_out=PID_Calc(&pid1,Shoot->rpm,-2500);}
// else if(LENGTH>pos[k-1]+10){ctrl_out=PID_Calc(&pid1,Shoot->rpm,2500);}
//else{ctrl_out=PID_Calc(&pid1,Shoot->rpm,0);}
// 
// }





return ctrl_out;

}




/*底盘总任务*/
void chassis_task(void)
{
	chassis_update();  
	chassis_mode_set();
	chassis_contorl_set();
	chassis_control_loop();
	chassis_out=chassis_2006();
	
   
}





/*底盘任务数据更新*/
static void chassis_update(void)
{		
	
	//方向管理
	if(chassis_move.chassis_YAW_motor.position_set - chassis_move.chassis_YAW_motor.ref_position > 0)
	{
		CONTROL_YAW_DIR(GPIO_PIN_RESET);			
		
	}
	else if(chassis_move.chassis_YAW_motor.position_set - chassis_move.chassis_YAW_motor.ref_position < 0)
	{
		CONTROL_YAW_DIR(GPIO_PIN_SET);
	}
	else;
	
	if(chassis_move.chassis_PITCH_motor.position_set - chassis_move.chassis_PITCH_motor.ref_position > 0)
	{
			CONTROL_PITCH_DIR(GPIO_PIN_SET);		
	}
	else if(chassis_move.chassis_PITCH_motor.position_set - chassis_move.chassis_PITCH_motor.ref_position < 0)
	{
			CONTROL_PITCH_DIR(GPIO_PIN_RESET);	
	}
	else;
	//若设定值有改变 则启动电机
	if(chassis_move.chassis_YAW_motor.last_position_set != chassis_move.chassis_YAW_motor.position_set) 
	{
		chassis_move.chassis_compete_flag.yaw_compete_flag = CHASSIS_NOT;	
    finish=0;		
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	}
	//若设定值有改变 则启动电机
	if(chassis_move.chassis_PITCH_motor.last_position_set != chassis_move.chassis_PITCH_motor.position_set) 
	{
		chassis_move.chassis_compete_flag.pitch_compete_flag = CHASSIS_NOT;
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	}
	
	//所需转动角度转脉冲
	chassis_move.chassis_YAW_motor.pluse_need = 
	fabs((chassis_move.chassis_YAW_motor.position_set - chassis_move.chassis_YAW_motor.ref_position) * POSITION_TO_PLUSE);
	
	chassis_move.chassis_PITCH_motor.pluse_need = 
	fabs((chassis_move.chassis_PITCH_motor.position_set - chassis_move.chassis_PITCH_motor.ref_position) * POSITION_TO_PLUSE);
		
}

/*底盘任务模式设定*/
static void chassis_mode_set(void)
{
	if(chassis_move.Chassis_remote_realte-> rc.s[1] == RC_SW_UP)
	{
		chassis_move.chassis_mode = CHASSIS_AUTO;
	}
	else if(chassis_move.Chassis_remote_realte->rc.s[1] == RC_SW_MID)
	{
		chassis_move.chassis_mode = CHASSIS_RC;
	}
	else if(chassis_move.Chassis_remote_realte-> rc.s[1] == RC_SW_DOWN)
	{
		chassis_move.chassis_mode = CHASSIS_RELAX;
	}	
}



/*底盘任务控制量设定*/
static void chassis_contorl_set(void)
{
	chassis_move.chassis_YAW_motor.last_position_set = chassis_move.chassis_YAW_motor.position_set; //上一次角度赋值
	chassis_move.chassis_PITCH_motor.last_position_set = chassis_move.chassis_PITCH_motor.position_set; //上一次角度赋值
	if(chassis_move.chassis_mode == CHASSIS_RC)
	{		
		CHASSIS_RC_Analyse();
	}
	else if(chassis_move.chassis_mode == CHASSIS_AUTO)
	{
		CHASSIS_AUTO_Analyse();
	}
	else if(chassis_move.chassis_mode == CHASSIS_RELAX)
	{
//		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
//		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
	//		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
//		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
		
		static int temp,pos;
	if(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_UPANDDOWN]> 330) //摇杆上移飞镖号加一
	{
		if(temp)
		{
			pos++;
			
	temp = 0;
		}
	}
	else if(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_UPANDDOWN] < -330) //摇杆下移飞镖号减一
	{
		if(temp)
		{
		 pos--;
			temp = 0;
		}
	}
	else  
	{
		temp = 1;
	}
	
	if(pos>=2){pos=0;}
	if(pos<0){pos=0;}
	if(pos==0)
		{chassis_move.chassis_PITCH_motor.position_set=0;
		chassis_move.chassis_YAW_motor.position_set=0;}
	else if(pos==1)
   {chassis_move.chassis_PITCH_motor.position_set=2;
		chassis_move.chassis_YAW_motor.position_set=2;}
//	else if(pos==2)
//   {chassis_move.chassis_PITCH_motor.position_set=4;
//		chassis_move.chassis_YAW_motor.position_set=4;}
		
		
	}
}

/*底盘任务闭环控制*/
static void chassis_control_loop(void)
{	
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,249);	
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,249);
}


/***********************/
/*      数据解析       */
/***********************/

static void CHASSIS_RC_Analyse(void) //遥控器模式数据解析
{
	//获取遥控器速度控制数据	
	
	int16_t yaw_remote_val;
	int16_t pitch_remote_val;
	static float yaw_temp;
	static float pitch_temp;
	rc_deadline_limit(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_LEFTANDRIGHT], yaw_remote_val, DEADLINE_OF_CH); 
	rc_deadline_limit(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_UPANDDOWN], pitch_remote_val, DEADLINE_OF_CH); 	
	
	yaw_temp += yaw_remote_val * REMOTEVAL_TO_YAW_POSITIONSET;
	pitch_temp += pitch_remote_val * REMOTEVAL_TO_PITCH_POSITIONSET; 
	
	//防止角度设定超限
  if(yaw_temp > MAX_ADD_TEMP )  yaw_temp = MAX_ADD_TEMP;
  else if(yaw_temp < -MAX_ADD_TEMP )  yaw_temp =  -MAX_ADD_TEMP;
	
	if(pitch_temp > MAX_ADD_TEMP )  pitch_temp  = MAX_ADD_TEMP;
  else if(pitch_temp < -MAX_ADD_TEMP )  pitch_temp = -MAX_ADD_TEMP; 
	
	
	//将temp数值加到设定值
	if(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_LEFTANDRIGHT] == 0)
	{
		static char i = 0;
		i++;
		if(i == 5)
		{
			i = 0;
			chassis_move.chassis_YAW_motor.position_set += yaw_temp;
			yaw_temp = 0;
		}
	}
	if(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_UPANDDOWN] == 0)
	{
		static char j = 0;
		j++;
		if(j == 5)
		{
			j = 0;
			chassis_move.chassis_PITCH_motor.position_set += pitch_temp;
			pitch_temp = 0;
		}
	}
	if(chassis_move.stand_flag == 1)
	{
		chassis_move.chassis_YAW_motor.position_set = chassis_move.stand_yaw + Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].yaw_add;
	}
	
	if(chassis_move.chassis_YAW_motor.position_set > MAX_YAW_POSITION) chassis_move.chassis_YAW_motor.position_set = MAX_YAW_POSITION;
	else if(chassis_move.chassis_YAW_motor.position_set < MIN_YAW_POSITION) chassis_move.chassis_YAW_motor.position_set = MIN_YAW_POSITION;
	
	if(chassis_move.chassis_PITCH_motor.position_set > MAX_PITCH_POSITION) chassis_move.chassis_PITCH_motor.position_set= MAX_PITCH_POSITION;
	else if(chassis_move.chassis_PITCH_motor.position_set < MIN_PITCH_POSITION) chassis_move.chassis_PITCH_motor.position_set = MIN_PITCH_POSITION;
	
	
}

static void CHASSIS_AUTO_Analyse(void) 
{
	if(!chassis_move.stand_flag)
	{
		chassis_move.stand_yaw   = chassis_move.chassis_YAW_motor.ref_position;
		chassis_move.stand_flag++;
	}
	
	static uint8_t yaw_temp;
		
			if(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_LEFTANDRIGHT] > 200)
			{
				if(!yaw_temp)
				{
					Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].yaw_add += 1;
					yaw_temp = 1;
				}
			}
			else if(chassis_move.Chassis_remote_realte->rc.ch[LEFT_MOTE_LEFTANDRIGHT] < -200)
			{
				if(!yaw_temp)
				{
					Shoot_move.Shoot_Dart_Data[Shoot_move.Dart_cnt].yaw_add -= 1;
					yaw_temp = 1;
				}
			}
			else
			{
				yaw_temp = 0;
			}
	
}

void TIM2_IRQHandler(void)
{
  chassis_move.chassis_YAW_motor.pluse_cnt++;
	if(chassis_move.chassis_YAW_motor.pluse_cnt >= chassis_move.chassis_YAW_motor.pluse_need)
	{		
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
		chassis_move.chassis_YAW_motor.ref_position = chassis_move.chassis_YAW_motor.position_set;
		chassis_move.chassis_YAW_motor.pluse_cnt = 0;				
		chassis_move.chassis_compete_flag.yaw_compete_flag = CHASSIS_AT;
	}		
  HAL_TIM_IRQHandler(&htim2);
}	
	
void TIM8_CC_IRQHandler(void)
{
  chassis_move.chassis_PITCH_motor.pluse_cnt++;
	if(chassis_move.chassis_PITCH_motor.pluse_cnt >= chassis_move.chassis_PITCH_motor.pluse_need)
	{		
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
		chassis_move.chassis_PITCH_motor.ref_position = chassis_move.chassis_PITCH_motor.position_set;
		chassis_move.chassis_PITCH_motor.pluse_cnt = 0;				
		chassis_move.chassis_compete_flag.pitch_compete_flag= CHASSIS_AT;
	}		
  HAL_TIM_IRQHandler(&htim8);
 
}
