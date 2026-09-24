/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "bsp_can.h"
#include "pid.h"
#include "ROBOT.h"
#include "navigation_task.h"
#include "chassis.h"
#include "NRF24L01.h"
#include "remote_control.h"
#include "air_operated.h"
#include "math_algorithm.h"
#include "smc.h"
#include "algorithm.h"
#include "motor_control.h"
#include "dunk.h"
#include "unitree_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
/* Definitions for LWIP_TASK */
osThreadId_t LWIP_TASKHandle;
const osThreadAttr_t LWIP_TASK_attributes = {
  .name = "LWIP_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SYSTEM */
osThreadId_t SYSTEMHandle;
const osThreadAttr_t SYSTEM_attributes = {
  .name = "SYSTEM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LOCATION */
osThreadId_t LOCATIONHandle;
const osThreadAttr_t LOCATION_attributes = {
  .name = "LOCATION",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for NAVIGATION */
osThreadId_t NAVIGATIONHandle;
const osThreadAttr_t NAVIGATION_attributes = {
  .name = "NAVIGATION",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for MOTOR_CONTROL */
osThreadId_t MOTOR_CONTROLHandle;
const osThreadAttr_t MOTOR_CONTROL_attributes = {
  .name = "MOTOR_CONTROL",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for REMOTE_CONTROL */
osThreadId_t REMOTE_CONTROLHandle;
const osThreadAttr_t REMOTE_CONTROL_attributes = {
  .name = "REMOTE_CONTROL",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FUN_TASK */
osThreadId_t FUN_TASKHandle;
const osThreadAttr_t FUN_TASK_attributes = {
  .name = "FUN_TASK",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for COMMUNICATION */
osThreadId_t COMMUNICATIONHandle;
const osThreadAttr_t COMMUNICATION_attributes = {
  .name = "COMMUNICATION",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Lwip_Task(void *argument);
void System_Monitor(void *argument);
void Location_Task(void *argument);
void Navigation_Task(void *argument);
void Motor_Control(void *argument);
void Remote_Control(void *argument);
void Fun_Task(void *argument);
void Communication_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of LWIP_TASK */
  LWIP_TASKHandle = osThreadNew(Lwip_Task, NULL, &LWIP_TASK_attributes);

  /* creation of SYSTEM */
  SYSTEMHandle = osThreadNew(System_Monitor, NULL, &SYSTEM_attributes);

  /* creation of LOCATION */
  LOCATIONHandle = osThreadNew(Location_Task, NULL, &LOCATION_attributes);

  /* creation of NAVIGATION */
  NAVIGATIONHandle = osThreadNew(Navigation_Task, NULL, &NAVIGATION_attributes);

  /* creation of MOTOR_CONTROL */
  MOTOR_CONTROLHandle = osThreadNew(Motor_Control, NULL, &MOTOR_CONTROL_attributes);

  /* creation of REMOTE_CONTROL */
  REMOTE_CONTROLHandle = osThreadNew(Remote_Control, NULL, &REMOTE_CONTROL_attributes);

  /* creation of FUN_TASK */
  FUN_TASKHandle = osThreadNew(Fun_Task, NULL, &FUN_TASK_attributes);

  /* creation of COMMUNICATION */
  COMMUNICATIONHandle = osThreadNew(Communication_Task, NULL, &COMMUNICATION_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Lwip_Task */
/**
  * @brief  Function implementing the LWIP_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Lwip_Task */
void Lwip_Task(void *argument)
{
  /* USER CODE BEGIN Lwip_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Lwip_Task */
}

/* USER CODE BEGIN Header_System_Monitor */
/**
* @brief Function implementing the SYSTEM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_System_Monitor */
void System_Monitor(void *argument)
{
  /* USER CODE BEGIN System_Monitor */
  /* Infinite loop */
  for(;;)
  {
    for(int i=0;i<2;i++)
		{
			system_monitor.can_rec_fps[i]=system_monitor.can_rec_cnt[i];
			system_monitor.can_rec_cnt[i]=0;
		}
		system_monitor.can_send_fps=system_monitor.can_send_cnt;
		system_monitor.can_send_cnt=0;
		
		system_monitor.motor_left_4219_fps=system_monitor.motor_left_4219_cnt;
		system_monitor.motor_left_4219_cnt=0;
		
		system_monitor.motor_right_4219_fps=system_monitor.motor_right_4219_cnt;
		system_monitor.motor_right_4219_cnt=0;
		
		system_monitor.motor_left_xc5000_fps=system_monitor.motor_left_xc5000_cnt;
		system_monitor.motor_left_xc5000_cnt=0;
		
		system_monitor.motor_right_xc5000_fps=system_monitor.motor_right_xc5000_cnt;
		system_monitor.motor_right_xc5000_cnt=0;
		
		system_monitor.motor_A1_fps=system_monitor.motor_A1_cnt;
		system_monitor.motor_A1_cnt=0;
		
		system_monitor.uart6_fps=system_monitor.uart6_cnt;
		system_monitor.uart6_cnt=0;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END System_Monitor */
}

/* USER CODE BEGIN Header_Location_Task */
/**
* @brief Function implementing the LOCATION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Location_Task */
void Location_Task(void *argument)
{
  /* USER CODE BEGIN Location_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Location_Task */
}

/* USER CODE BEGIN Header_Navigation_Task */
/**
* @brief Function implementing the NAVIGATION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Navigation_Task */
void Navigation_Task(void *argument)
{
  /* USER CODE BEGIN Navigation_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Navigation_Task */
}

/* USER CODE BEGIN Header_Motor_Control */
/**
* @brief Function implementing the MOTOR_CONTROL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Control */
void Motor_Control(void *argument)
{
  /* USER CODE BEGIN Motor_Control */
  /* Infinite loop */
  for(;;)
  {
		if(nav.nav_state!=NAV_OFF)
		{
			PID_Calc_NEW(&chassis_run.rightdown);
			PID_Calc_NEW(&chassis_run.rightup);
			PID_Calc_NEW(&chassis_run.leftdown);
			PID_Calc_NEW(&chassis_run.leftup);
			
			chassis_run.leftup.fpU += friction_compensation_current.leftup + feed_forward_current.leftup;
			chassis_run.rightup.fpU += friction_compensation_current.rightup + feed_forward_current.rightup;
			chassis_run.rightdown.fpU += friction_compensation_current.rightdown + feed_forward_current.rightdown;
			chassis_run.leftdown.fpU += friction_compensation_current.leftdown + feed_forward_current.leftdown;
//			LESO_Order1(&leso_leftup,chassis_run.leftup.fpFB,chassis_run.leftup.fpU);
//			LESO_Order1(&leso_rightup,chassis_run.rightup.fpFB,chassis_run.rightup.fpU);
//			LESO_Order1(&leso_rightdown,chassis_run.rightdown.fpFB,chassis_run.rightdown.fpU);
//			LESO_Order1(&leso_leftdown,chassis_run.leftdown.fpFB,chassis_run.leftdown.fpU);
//			
//			compensation_leftup = leso_leftup.U - leso_leftup.U0;
//			compensation_rightup = leso_rightup.U - leso_rightup.U0;
//			compensation_rightdown = leso_rightdown.U - leso_rightdown.U0;
//			compensation_leftdown = leso_leftdown.U - leso_leftdown.U0;
//			
//			chassis_run.leftup.fpU += compensation_leftup + feed_forward_current.leftup;
//			chassis_run.rightup.fpU += compensation_rightup + feed_forward_current.rightup;
//			chassis_run.rightdown.fpU += compensation_rightdown + feed_forward_current.rightdown;;
//			chassis_run.leftdown.fpU += compensation_leftdown + feed_forward_current.leftdown;
		}
//		CAN_SendCurrent(&hcan1,0x200,chassis_run.leftup.fpU,chassis_run.rightup.fpU,chassis_run.rightdown.fpU,chassis_run.leftdown.fpU);
		jump_motor_pos_ctrl();
		jump_motor_control(&left_xc5000_motor);
		jump_motor_control(&right_xc5000_motor);
		jump_motor_control(&left_4219_motor);
		jump_motor_control(&right_4219_motor);
		CAN_SendCurrent(&hcan2,0x200,left_xc5000_motor.Motor_Data.motor_current,right_xc5000_motor.Motor_Data.motor_current
																,left_4219_motor.Motor_Data.motor_current,right_4219_motor.Motor_Data.motor_current);
		
		arm_motor_control();
		switch(Suction_State)
		{
			case ALL_CLOSE:
				CLOSE_VALVE(1);
				CLOSE_VALVE(2);
				break;
			case CATCH_BALL:
				OPEN_VALVE(1);
				CLOSE_VALVE(2);
				break;
			case LOOSE_BALL:
				CLOSE_VALVE(1);
				OPEN_VALVE(2);
				break;
			default:
				break;
		}
		SendAirMsgByCan1(&g_uiAirValve);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Motor_Control */
}

/* USER CODE BEGIN Header_Remote_Control */
/**
* @brief Function implementing the REMOTE_CONTROL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Control */
void Remote_Control(void *argument)
{
  /* USER CODE BEGIN Remote_Control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Control */
}

/* USER CODE BEGIN Header_Fun_Task */
/**
* @brief Function implementing the FUN_TASK thread.
* @param argument: Not used
* @retval None
*/
u8 flag;
/* USER CODE END Header_Fun_Task */
void Fun_Task(void *argument)
{
  /* USER CODE BEGIN Fun_Task */
  /* Infinite loop */
  for(;;)
  {
		if(dunk_flag == 1)
		{
			Dunk();
			if(Dunk_State == DUNK_END)
			{
				dunk_flag = 0;
				Dunk_State = DUNK_INIT;
			}
		}
		if(pos_mode_flag == 1)
		{
			if(jump_motor_pos_mode())
				pos_mode_flag = 0;
		}
		if(damp_mode_flag == 1)
		{
			jump_motor_damp_mode();
			damp_mode_flag = 0;
		}
    osDelay(1);
  }
  /* USER CODE END Fun_Task */
}

/* USER CODE BEGIN Header_Communication_Task */
/**
* @brief Function implementing the COMMUNICATION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communication_Task */
void Communication_Task(void *argument)
{
  /* USER CODE BEGIN Communication_Task */
  /* Infinite loop */
  for(;;)
  {
		motor_A1_cmd(&Motor_A1);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Communication_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

