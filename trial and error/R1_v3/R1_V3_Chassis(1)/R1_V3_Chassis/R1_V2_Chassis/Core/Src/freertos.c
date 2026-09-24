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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Navigation_Task.h"
#include "Location_Task.h"
#include "Can_Bsp.h"
#include "usart.h"
#include "vofa.h"
#include "Vision.h"
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
/* Definitions for _Remote_Control */
osThreadId_t _Remote_ControlHandle;
const osThreadAttr_t _Remote_Control_attributes = {
  .name = "_Remote_Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _Navigation */
osThreadId_t _NavigationHandle;
const osThreadAttr_t _Navigation_attributes = {
  .name = "_Navigation",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for _Location */
osThreadId_t _LocationHandle;
const osThreadAttr_t _Location_attributes = {
  .name = "_Location",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _Local_Control */
osThreadId_t _Local_ControlHandle;
const osThreadAttr_t _Local_Control_attributes = {
  .name = "_Local_Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Remote_Control(void *argument);
void Navigation(void *argument);
void Location(void *argument);
void Local_Control(void *argument);

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
  /* creation of _Remote_Control */
  _Remote_ControlHandle = osThreadNew(Remote_Control, NULL, &_Remote_Control_attributes);

  /* creation of _Navigation */
  _NavigationHandle = osThreadNew(Navigation, NULL, &_Navigation_attributes);

  /* creation of _Location */
  _LocationHandle = osThreadNew(Location, NULL, &_Location_attributes);

  /* creation of _Local_Control */
  _Local_ControlHandle = osThreadNew(Local_Control, NULL, &_Local_Control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Remote_Control */
/**
  * @brief  Function implementing the _Remote_Control thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Remote_Control */
void Remote_Control(void *argument)
{
  /* USER CODE BEGIN Remote_Control */
  /* Infinite loop */
  for(;;)
  {
		//遥控器部分（3ms一次）
		if(nRF24L01_Tick==3)
		{
		parseDataPacket(nRF24L01_RxBuf, &Js_Value);//解析遥控发来的数�?
		pack_data(&nRF24L01_ack_pay.Ack_Buf[0]);//打包发回遥控器的数据
		HAL_UART_Transmit_DMA(&huart4, nRF24L01_ack_pay.Ack_Buf, 5);//发回遥控�?
		nRF24L01_Tick=0;	
		}
		
		//上下板通信部分
		HAL_UART_Transmit_DMA(&huart1, uart1_tx_buffer, 26);
		
		//发给上位机VOFA的部�?
		//（vofa数组赋值）	
		vofa[0]=nav.auto_path.pos_pid.x.fpDes;
		vofa[1]=nav.auto_path.pos_pid.x.fpFB;
		vofa[2]=nav.auto_path.pos_pid.y.fpDes;
		vofa[3]=nav.auto_path.pos_pid.y.fpFB;	
		vofa[4]=nav.auto_path.pos_pid.w.fpDes;
		vofa[5]=nav.auto_path.pos_pid.w.fpFB;
		
		vofa[6]=chassis_run.leftdown.fpDes;
		vofa[7]=chassis_run.leftdown.fpFB;
		
		vofa[8]=chassis_run.leftup.fpDes;
		vofa[9]=chassis_run.leftup.fpFB;
		
		vofa[10]=chassis_run.rightup.fpDes;
		vofa[11]=chassis_run.rightup.fpFB;
		
		vofa[12]=chassis_run.rightdown.fpDes;
		vofa[13]=chassis_run.rightdown.fpFB;
//    vofa[13]=system_monitor.flw_fps;
		
//		vofa[14]=degreeA;
//    vofa[15]=degreeB;
//		vofa[16]=stRobot.stPos.fpPosX;
//		vofa[17]=stRobot.stPos.fpPosY;
//		vofa[18]=stRobot.stPos.fpPosQ;
    vofa[16]=Vision_Data.x2;
		vofa[17]=system_monitor.Vision_Receive_fps;
		vofa[18]=system_monitor.communicate_tx_fps;
		
		VOFA_transmit_data(vofa, 19);//vofa发�?
		
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Remote_Control */
}

/* USER CODE BEGIN Header_Navigation */
/**
* @brief Function implementing the _Navigation thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Navigation */
void Navigation(void *argument)
{
  /* USER CODE BEGIN Navigation */
  /* Infinite loop */
  for(;;)
  {
		
		
		navigation();
		system_monitor.navigation_cnt++;
		
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  /* USER CODE END Navigation */
}

/* USER CODE BEGIN Header_Location */
/**
* @brief Function implementing the _Location thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Location */
void Location(void *argument)
{
  /* USER CODE BEGIN Location */
  /* Infinite loop */
  for(;;)
  { 
		
//	if(gyro_tick>=5)//   200HZ处理陀螺仪数据
//	{
//		process_data();
//		 gyro_tick=0;
//	}
		
		 


		 
//	 Vision_Location();
    all_locate();//混合定位
//  	Follower_Wheel_Location(&stRobot,&stFollowerWheel);//纯随动轮定位
    UpdatePositionFeedback(&nav,&stRobot);//更新位置环pid
		PositionToVelt();//计算自身速度
		system_monitor.location_cnt++;
		
		
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Location */
}

/* USER CODE BEGIN Header_Local_Control */
/**
* @brief Function implementing the _Local_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Local_Control */
void Local_Control(void *argument)
{
  /* USER CODE BEGIN Local_Control */
  /* Infinite loop */
    for(;;)
  { 
//		if(gyro_reset_flag)
//		{
		HAL_UART_Transmit_DMA(&huart2, g_Usart2_Tx_buf, 1);
//		gyro_reset_flag=0;
//		g_Usart2_Tx_buf[0]=0;
//		}
		
		
		
		
    QD_CANx_SendstdData(&hcan1,0x300,AirCtrl_1,1);//向气动板发送从上层接收到的一位数组
		QD_CANx_SendstdData(&hcan1,0x400,AirCtrl_2,1);//向气动板发送从上层接收到的一位数组
    Chassis_Motor_Ctrl();
    Crane_3508_Ctrl();		
		Vision_Transmit();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Local_Control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

