#include "detect_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*------------------------------------------------------*/	
/*                     定义监控设备结构体               */	
/*------------------------------------------------------*/	
static error_t errorList[errorListLength + 1];


/*------------------------------------------------------*/	
/*                        静态函数声明                  */	
/*------------------------------------------------------*/	

static void DetectInit(uint32_t time);   //初始化错误列表
//static void DetectDisplay(void);         //显示错误信息
static void detect_display(void);//在线状态显示

static GPIO_TypeDef* led_gpio_port[10]={D1_GPIO_Port,D2_GPIO_Port,D3_GPIO_Port,D4_GPIO_Port,D5_GPIO_Port,D6_GPIO_Port,D7_GPIO_Port,D8_GPIO_Port,D9_GPIO_Port,D10_GPIO_Port};//led引脚分类 D1~D10
static uint16_t led_gpio_pin[10]={D1_Pin,D2_Pin,D3_Pin,D4_Pin,D5_Pin,D6_Pin,D7_Pin,D8_Pin,D9_Pin,D10_Pin};//led引脚号 D1~D10


/*------------------------------------------------------*/	
/*                        掉线判断任务                  */	
/*------------------------------------------------------*/	

void detect_task(void const * argument)
{
    static uint32_t systemTime;
    systemTime = xTaskGetTickCount();
	
    //初始化
    DetectInit(systemTime);
	
    //空闲一段时间
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
       systemTime = xTaskGetTickCount();

       for (int i = 0; i < errorListLength; i++)
       {
            //未使能，跳过设备监测
            if (errorList[i].enable == 0)
            {
                continue;
            }

			//刚上线排除
            else if (systemTime - errorList[i].worktime < errorList[i].setOnlineTime)
            {
                errorList[i].isLost = 0;
            }
						
            //超时掉线
            else if (systemTime - errorList[i].newTime > errorList[i].setOfflineTime)
            {
                if (errorList[i].isLost == 0)
                {
                    //记录错误以及掉线时间
                    errorList[i].isLost = 1;
                    errorList[i].Losttime = systemTime;
                }
            }			
			//正常工作
            else
            {
                errorList[i].isLost = 0;		
                //计算频率
                if (errorList[i].newTime > errorList[i].lastTime)
                {
                    errorList[i].frequency = configTICK_RATE_HZ / (fp32)(errorList[i].newTime - errorList[i].lastTime);
                }
            }
       }

	    //离线效果展示
//        DetectDisplay();

			//在线状态显示
				detect_display();
	   
	    //任务延时
        vTaskDelay(DETECT_CONTROL_TIME);

    }
}



/*------------------------------------------------------*/	
/*                   设备接收数据钩子函数               */	
/*------------------------------------------------------*/	

void DetectHook(uint8_t toe)
{
    errorList[toe].lastTime = errorList[toe].newTime;
    errorList[toe].newTime  = xTaskGetTickCount();
	
    //更新丢失情况
    if (errorList[toe].isLost)
    {
        errorList[toe].isLost   = 0;
        errorList[toe].worktime = errorList[toe].newTime;
    }
}



/*------------------------------------------------------*/	
/*                   返回对应的设备是否丢失             */	
/*------------------------------------------------------*/	

bool_t toe_is_error(uint8_t err)
{
	//如果有设备离线，isLost为1，函数返回1；没有设备离线，函数返回0
    return (errorList[err].isLost == 1);
}

/*------------------------------------------------------*/	
/*                     在线状态灯效展示                 */	
/*------------------------------------------------------*/	

static void detect_display(void)
{
	uint8_t i;
	//D1~D6
	for(i=0;i<DETECT_LED_NUM-2;i++)
	{
		if((errorList[i].isLost==0)&&(errorList[i].enable==1))
		{
			HAL_GPIO_WritePin(led_gpio_port[i],led_gpio_pin[i],GPIO_PIN_RESET);
		}
		else 
		{
			HAL_GPIO_WritePin(led_gpio_port[i],led_gpio_pin[i],GPIO_PIN_SET);		
		}
	}
	//D7
	if((errorList[FricLeftMotorTOE].isLost==0)&&(errorList[FricRightMotorTOE].isLost==0)&&(errorList[FricLeftMotorTOE].enable==1)&&(errorList[FricRightMotorTOE].enable==1))
	{
		HAL_GPIO_WritePin(D7_GPIO_Port,D7_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(D7_GPIO_Port,D7_Pin,GPIO_PIN_SET);		
	}
	//D8
	if((errorList[ChassisMotor1TOE].isLost==0)&&(errorList[ChassisMotor2TOE].isLost==0)&&(errorList[ChassisMotor3TOE].isLost==0)&&(errorList[ChassisMotor4TOE].isLost==0)&&(errorList[ChassisMotor1TOE].enable==1)&&(errorList[ChassisMotor2TOE].enable==1)&&(errorList[ChassisMotor3TOE].enable==1)&&(errorList[ChassisMotor4TOE].enable==1))
	{
		HAL_GPIO_WritePin(D8_GPIO_Port,D8_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(D8_GPIO_Port,D8_Pin,GPIO_PIN_SET);		
	}
}

/*------------------------------------------------------*/	
/*                         离线灯效展示                 */	
/*------------------------------------------------------*/	

//static void DetectDisplay(void)
//{
//	uint8_t error_flag =0;
//	if(errorList[2].isLost && (errorList[2].enable == 1) )
//	{
//		OLED_P8x16Str(30,6,(uint8_t *)"super");
//		error_flag =1;
//	}

//	if(errorList[3].isLost && (errorList[3].enable == 1) )
//	{
//		OLED_P8x16Str(30,0,(uint8_t *)"yaw");
//		error_flag =1;
//	}
// 
//	if(errorList[4].isLost && (errorList[4].enable == 1) )
//	{
//		OLED_P8x16Str(30,2,(uint8_t *)"pitch");
//		error_flag =1;
//	}

//	if(errorList[5].isLost && (errorList[5].enable == 1) )
//	{
//		OLED_P8x16Str(30,4,(uint8_t *)"trig");
//		error_flag =1;
//	}

//	if(errorList[6].isLost && (errorList[6].enable == 1) )
//	{
//		OLED_P8x16Str(85,2,(uint8_t *)"frc_L");
//		error_flag =1;
//	}


//	if(errorList[7].isLost && (errorList[7].enable == 1) )
//	{
//		OLED_P8x16Str(85,0,(uint8_t *)"frc_R");
//		error_flag =1;
//	}

//	if(errorList[8].isLost && (errorList[8].enable == 1) )
//	{
//		OLED_P8x16Str(0,0,(uint8_t *)"DP1");
//		error_flag =1;
//	}

//	if(errorList[9].isLost && (errorList[9].enable == 1) )
//	{
//		OLED_P8x16Str(0,2,(uint8_t *)"DP2");
//		error_flag =1;
//	}

//	if(errorList[10].isLost && (errorList[10].enable == 1) )
//	{
//		OLED_P8x16Str(0,4,(uint8_t *)"DP3");
//		error_flag =1;
//	}

//	if(errorList[11].isLost && (errorList[11].enable == 1) )
//	{
//		OLED_P8x16Str(0,6,(uint8_t *)"DP4");
//		error_flag =1;
//	}
//	
//	if(error_flag == 0)
//	{
//		OLED_P8x16Str(35,3,(uint8_t *)"OK!FIGHT!");
//		buzzer_off();
//	}
//		delay_ms(700);
//		OLED_Fill(0);
//}


/*------------------------------------------------------*/	
/*                       离线检测初始化                 */	
/*------------------------------------------------------*/	

static void DetectInit(uint32_t time)
{
    //离线时间阈值 ，上线时间阈值，优先级，使能状态
    uint16_t setItem[errorListLength][4] =
        {
            {30, 40,  15, 1},   //DBUS
            {20,100,  14, 1},   //INS
            {10, 10,  13, 1},   //YAW
            {10, 10,  12, 1},   //PITCH
            {10, 10,  11, 1},   //TRIGGER42
            {10, 10,  10, 1},   //TRIGGER17
            {10, 10,   9, 1},   //FRICLEFT
            {10, 10,   8, 1},   //FRICRIGHT
            {10, 10,   7, 1},   //CHASSIS1
            {10, 10,   6, 1},   //CHASSIS2
            {10, 10,   5, 1},   //CHASSIS3
            {10, 10,   4, 1},   //CHASSIS4
			
        };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
        errorList[i].setOfflineTime = setItem[i][0];
        errorList[i].setOnlineTime =  setItem[i][1];
        errorList[i].Priority = setItem[i][2];
        errorList[i].enable = setItem[i][3];
			
        errorList[i].isLost = 1;
        errorList[i].frequency = 0.0f;
        errorList[i].newTime =  time;
        errorList[i].lastTime = time;
        errorList[i].Losttime = time;
        errorList[i].worktime = time;
    }

}

