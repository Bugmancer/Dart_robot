/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_task.h"
#include "shoot_task.h"
#include "switch.h"
#include "CAN_receive.h"
#include "shoot.h"
#include "remote_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
switch_flag swflag;
switch_flag flag;
int k1 = 0;
const Revice_Motro_Data *motor_data_rtos;
float vofa[6];
UART_HandleTypeDef huart7;
double Len;
int out;
const RC_ctrl_t *rc_ctrl_rtos;
int z_f;
int BF;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Chassis_TASKHandle;
osThreadId Shoot_TASKHandle;
osThreadId Decide_TASKHandle;
osThreadId guangdianHandle;
osThreadId shoot_partHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Chassis_Task(void const * argument);
void Shoot_Task(void const * argument);
void Decide_Task(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Chassis_TASK */
  osThreadDef(Chassis_TASK, Chassis_Task, osPriorityNormal, 0, 256);
  Chassis_TASKHandle = osThreadCreate(osThread(Chassis_TASK), NULL);

  /* definition and creation of Shoot_TASK */
  osThreadDef(Shoot_TASK, Shoot_Task, osPriorityNormal, 0, 256);
  Shoot_TASKHandle = osThreadCreate(osThread(Shoot_TASK), NULL);

  /* definition and creation of Decide_TASK */
  osThreadDef(Decide_TASK, Decide_Task, osPriorityIdle, 0, 256);
  Decide_TASKHandle = osThreadCreate(osThread(Decide_TASK), NULL);

  /* definition and creation of guangdian */
  osThreadDef(guangdian, StartTask04, osPriorityIdle, 0, 128);
  guangdianHandle = osThreadCreate(osThread(guangdian), NULL);

  /* definition and creation of shoot_part */
  osThreadDef(shoot_part, StartTask05, osPriorityIdle, 0, 128);
  shoot_partHandle = osThreadCreate(osThread(shoot_part), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Chassis_Task */
/**
  * @brief  Function implementing the Chassis_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
	chassis_task_Init();
  /* Infinite loop */
  for(;;)
  {
		chassis_task();
    osDelay(2);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
	shoot_task_Init();
  /* Infinite loop */
  for(;;)
  {
		shoot_task(); 
		osDelay(2);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_Decide_Task */
/**
* @brief Function implementing the Decide_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Decide_Task */
void Decide_Task(void const * argument)
{
  /* USER CODE BEGIN Decide_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(2);
  }
  /* USER CODE END Decide_Task */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the guangdian thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
	(void)argument;

	/* 拉绳电机控制器使用第一个外侧发射电机的反馈数据。 */
	motor_data_rtos = Back_OutsideShoot_Data(0);
	shootint(motor_data_rtos);

  /* Infinite loop */
  for(;;)
  {
		/* 保留原有 VOFA 调试输出，同时执行 2 ms 控制周期。 */
		z_f = read_flag();
		rc_ctrl_rtos = get_remote_control_point();

		vofa[0] = z_f;
		vofa[1] = 0;
		vofa[2] = 0;
		vofa[3] = 0;
		vofa[4] = 0;
		vofa[5] = 0;
		vofa_justfloat(&huart7, vofa, 6);

		out = shoottask(motor_data_rtos, 800);

		/* 发送 CAN 电流前先判断遥控急停，急停优先级最高。 */
		if (rc_ctrl_rtos->rc.s[0] == 2)
		{
			CAN_cmd_shoot(0, 0, 0, 0);
		}
		else
		{
			CAN_cmd_shoot(out, 0, 0, 0);
		}

    osDelay(2);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the shoot_part thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
	(void)argument;

  /* Infinite loop */
  for(;;)
  {
    osDelay(2);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
