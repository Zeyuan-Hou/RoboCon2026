/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Type.h"
#include "CAN_BSP.h"
#include "PID.h"
#include "GO1_MOTOR.h"
#include "chassis.h"
#include "communication.h"
#include "path.h"
#include "up_down_step.h"
#include "vofa.h"
#include "navigation.h"
#include "ramp_task.h"
#include "JSValue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	uint8_t test_vision_flag = 0;
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
/* Definitions for vofa_task */
osThreadId_t vofa_taskHandle;
const osThreadAttr_t vofa_task_attributes = {
  .name = "vofa_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for all_logic_task */
osThreadId_t all_logic_taskHandle;
const osThreadAttr_t all_logic_task_attributes = {
  .name = "all_logic_task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for run_led_task */
osThreadId_t run_led_taskHandle;
const osThreadAttr_t run_led_task_attributes = {
  .name = "run_led_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pos_yaw_task */
osThreadId_t pos_yaw_taskHandle;
const osThreadAttr_t pos_yaw_task_attributes = {
  .name = "pos_yaw_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void vofa_task_(void *argument);
void all_logic_task_(void *argument);
void run_led_task_(void *argument);
void pos_yaw_task_(void *argument);

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
  /* creation of vofa_task */
  vofa_taskHandle = osThreadNew(vofa_task_, NULL, &vofa_task_attributes);

  /* creation of all_logic_task */
  all_logic_taskHandle = osThreadNew(all_logic_task_, NULL, &all_logic_task_attributes);

  /* creation of run_led_task */
  run_led_taskHandle = osThreadNew(run_led_task_, NULL, &run_led_task_attributes);

  /* creation of pos_yaw_task */
  pos_yaw_taskHandle = osThreadNew(pos_yaw_task_, NULL, &pos_yaw_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_vofa_task_ */
/**
  * @brief  Function implementing the vofa_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vofa_task_ */
void vofa_task_(void *argument)
{
  /* USER CODE BEGIN vofa_task_ */

  /* Infinite loop */
  for(;;)
  {
		vofa[12]=chassis_run.wheel_1.fpDes;
		vofa[13]=chassis_run.wheel_1.fpFB;

		vofa[14]=chassis_run.wheel_2.fpDes;
		vofa[15]=chassis_run.wheel_2.fpFB;
		
		vofa[16]=chassis_run.wheel_3.fpDes;
		vofa[17]=chassis_run.wheel_3.fpFB;

		vofa[18]=chassis_run.wheel_4.fpDes;
		vofa[19]=chassis_run.wheel_4.fpFB;	
		
//		vofa[0]=chassis_run.wheel_1.friction;
//		vofa[1]=chassis_run.wheel_2.friction;
//		vofa[2]=chassis_run.wheel_3.friction;
//		vofa[3]=chassis_run.wheel_4.friction;

//		vofa[20]=chassis_run.wheel_1.fpU;
//		vofa[21]=chassis_run.wheel_2.fpU;	
//		vofa[22]=chassis_run.wheel_3.fpU;
//		vofa[23]=chassis_run.wheel_4.fpU;	
//		
//		vofa[5] = chassis_run.wheel_1.fpUp;
//		vofa[6] =	chassis_run.wheel_2.fpUp;
//		vofa[8] = chassis_run.wheel_3.fpUp;
//		vofa[9] = chassis_run.wheel_4.fpUp;
//		
		
//		vofa[8]=chassis_run.wheel_1.fpUi;
//		vofa[9]=chassis_run.wheel_2.fpUi;
//		vofa[10]=chassis_run.wheel_3.fpUi;
//		vofa[11]=chassis_run.wheel_4.fpUi;			
//		
//		vofa[4]=chassis_run.wheel_1.fpUff;
//		vofa[5]=chassis_run.wheel_2.fpUff;
//		vofa[6]=chassis_run.wheel_3.fpUff;
//		vofa[7]=chassis_run.wheel_4.fpUff;			

//		vofa[0]=point_only.pid_x.fpDes;
//		vofa[1]=point_only.pid_x.fpFB;
//		vofa[2]=point_only.pid_x.fpUi;
//		vofa[3]=point_only.pid_x.fpU;
//		
//		vofa[4]=point_only.pid_y.fpDes;
//		vofa[5]=point_only.pid_y.fpFB;
//		vofa[6]=point_only.pid_y.fpUi;
//		vofa[7]=point_only.pid_y.fpU;
//		
//		vofa[8]=point_only.pid_w.fpDes;
//		vofa[9]=point_only.pid_w.fpFB;
//		vofa[10]=point_only.pid_w.fpUi;
//		vofa[11]=point_only.pid_w.fpU;	
//		
//		vofa[10]=robot_pos.fpPosX;			
//		vofa[11]=robot_pos.fpPosY;	

//		vofa[14]=chassis_run.wheel_1.fpDes;			
//		vofa[15]=chassis_run.wheel_1.fpFB;
//		
		vofa[0]=nav.expect_robot_global_velt.fpX;
		vofa[1]=nav.expect_robot_global_velt.fpY;
		vofa[2]=nav.expect_robot_global_velt.fpW;
		vofa[3]=fabs(nav.auto_path.pos_pid.pid_x.fpE);
		
		vofa[4]=nav.auto_path.pos_pid.pid_x.fpDes;
		vofa[5]=nav.auto_path.pos_pid.pid_y.fpDes;
		vofa[6]=nav.auto_path.pos_pid.pid_w.fpDes;
		
		vofa[7]=nav.auto_path.pos_pid.pid_x.fpFB;
		vofa[8]=nav.auto_path.pos_pid.pid_y.fpFB;
		vofa[9]=nav.auto_path.pos_pid.pid_w.fpFB;
//		
//		vofa[10]=nav.nav_state;
//		vofa[11]=ramp_state;
//		vofa[12]=V_W;


		if(monitor.rate_fps.vision>=200&&fabs(robot_pos.fpPosX)<=20000&&fabs(robot_pos.fpPosY)<=20000&&robot_pos.fpPosY!=0&&robot_pos.fpPosX!=0){key_send[1] = 1;}else{key_send[1] = 2;}
			
		if(vision_data_recieve.KFS_number!=0&&vision_data_recieve.path_number!=0){key_send[2] = 1;}else{key_send[2] = 2;}			
		
		if(!gyro_or_radar){key_send[3] = 1;}else {key_send[3] = 2;}
		
		start_key();
		usart_all_ctrl_send();
		
		remote_send_();
		
		vision_send_task();
		
		VOFA_transmit_data(vofa, 25);		
    osDelay(1);
  }
  /* USER CODE END vofa_task_ */
}

/* USER CODE BEGIN Header_all_logic_task_ */
/**
* @brief Function implementing the all_logic_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_all_logic_task_ */
void all_logic_task_(void *argument)
{
  /* USER CODE BEGIN all_logic_task_ */
  /* Infinite loop */
  for(;;)
  {
		all_path_logic();

    if(ramp_test_flag ==1)
    {
      ramp_task();
    }
		Navigate_Task();
		up_down_logic();		
		choose_pid_yaw();
		
		
	  usart_inner_send();
		SpeedDistribute_Four_OmnidriectionalWhile(&nav.expect_robot_global_velt);
		
		PID_Calc_NEW_wheel(&chassis_run.wheel_1);
		PID_Calc_NEW_wheel(&chassis_run.wheel_2);		
		PID_Calc_NEW_wheel(&chassis_run.wheel_3);		
		PID_Calc_NEW_wheel(&chassis_run.wheel_4);		
		PID_Calc_NEW(&dji_run.DJI_1);		
		PID_Calc_NEW(&dji_run.DJI_2);
		
		if(ctrl_flag.chassis_flag == 0)	{CAN_SendCurrent(&hfdcan2,0x200,1,1,1,1);}
		else if(ctrl_flag.chassis_flag == 1)
		{CAN_SendCurrent(&hfdcan2,0x200,chassis_run.wheel_1.fpU,chassis_run.wheel_2.fpU,chassis_run.wheel_3.fpU,chassis_run.wheel_4.fpU);}
		
		
		if(ctrl_flag.dji_flag == 0)
		{CAN_SendCurrent(&hfdcan1,0x200,0,0,0,0);}else if(ctrl_flag.dji_flag == 1)
		{CAN_SendCurrent(&hfdcan1,0x200,dji_run.DJI_1.fpU,dji_run.DJI_2.fpU,0,0);}
		
		airCtrl[0] = bin_array_to_u8(AirOperaterCtrlBuf);		
		CAN_SendStdData(&hfdcan1,0X300,airCtrl,1);
		
    osDelay(1);
  }
  /* USER CODE END all_logic_task_ */
}

/* USER CODE BEGIN Header_run_led_task_ */
/**
* @brief Function implementing the run_led_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_run_led_task_ */
void run_led_task_(void *argument)
{
  /* USER CODE BEGIN run_led_task_ */
	static uint16_t delay_500 = 0;
	static uint8_t  delay_20 = 0;
	static uint8_t red,blue,green = 0;
  /* Infinite loop */
  for(;;)
  {
		delay_500++;
		delay_20++;
		if(delay_500>=500)
		{		
			if(vision_data_recieve.KFS_number!=0&&monitor.rate_fps.vision>=200){HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);} 
			else {HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);}
			delay_500=0;
		}
		
		
		if(delay_20>=20)
		{
       WS2812_AllSet(red,blue,green);
       WS2812_Update();		
			 delay_20 = 0;
		}
		
		if(vision_data_recieve.path_number == 1){red=1;blue=0;green=0;}
		else if(vision_data_recieve.path_number == 2){red=0;blue=0;green=1;}
		else if(vision_data_recieve.path_number == 3){red=0;blue=1;green=0;}
		else {red=0;blue=0;green=0;}
			
		
    osDelay(1);
  }
  /* USER CODE END run_led_task_ */
}

/* USER CODE BEGIN Header_pos_yaw_task_ */
/**
* @brief Function implementing the pos_yaw_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pos_yaw_task_ */
void pos_yaw_task_(void *argument)
{
  /* USER CODE BEGIN pos_yaw_task_ */
	static uint8_t delay_6_timer = 0;
  /* Infinite loop */
	
	
  for(;;)
  {
		delay_6_timer++;
		if(delay_6_timer>=6)
		{
			robot_pos.fpPosQ = GET_YAW(yaw_gyro_rad,vision_data_recieve.radar_yaw,gyro_w);
			delay_6_timer = 0;
		}
    osDelay(1);
  }
  /* USER CODE END pos_yaw_task_ */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

