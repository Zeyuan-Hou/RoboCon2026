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
/* Definitions for location_task */
osThreadId_t location_taskHandle;
const osThreadAttr_t location_task_attributes = {
  .name = "location_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for nav_task */
osThreadId_t nav_taskHandle;
const osThreadAttr_t nav_task_attributes = {
  .name = "nav_task",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassis_task */
osThreadId_t chassis_taskHandle;
const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for send_task */
osThreadId_t send_taskHandle;
const osThreadAttr_t send_task_attributes = {
  .name = "send_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for action_task */
osThreadId_t action_taskHandle;
const osThreadAttr_t action_task_attributes = {
  .name = "action_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gyro_Queue */
osMessageQueueId_t Gyro_QueueHandle;
const osMessageQueueAttr_t Gyro_Queue_attributes = {
  .name = "Gyro_Queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LocationTask(void *argument);
void NavTask(void *argument);
void ChassisTask(void *argument);
void SendTask(void *argument);
void ActionTask(void *argument);

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

  /* Create the queue(s) */
  /* creation of Gyro_Queue */
  Gyro_QueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &Gyro_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of location_task */
  location_taskHandle = osThreadNew(LocationTask, NULL, &location_task_attributes);

  /* creation of nav_task */
  nav_taskHandle = osThreadNew(NavTask, NULL, &nav_task_attributes);

  /* creation of chassis_task */
  chassis_taskHandle = osThreadNew(ChassisTask, NULL, &chassis_task_attributes);

  /* creation of send_task */
  send_taskHandle = osThreadNew(SendTask, NULL, &send_task_attributes);

  /* creation of action_task */
  action_taskHandle = osThreadNew(ActionTask, NULL, &action_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LocationTask */
/**
  * @brief  Function implementing the location_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LocationTask */
void LocationTask(void *argument)
{
  /* USER CODE BEGIN LocationTask */
  /* Infinite loop */
  for(;;)
  {
    Location_dt35_vision();
		sys_mnt.cnt.location_task++;
    osDelay(1);
  }
  /* USER CODE END LocationTask */
}

/* USER CODE BEGIN Header_NavTask */
/**
* @brief Function implementing the nav_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NavTask */
void NavTask(void *argument)
{
  /* USER CODE BEGIN NavTask */
  /* Infinite loop */
  for(;;)
  {
    Nav_Run();
		sys_mnt.cnt.navigation_task++;
    osDelay(1);
  }
  /* USER CODE END NavTask */
}

/* USER CODE BEGIN Header_ChassisTask */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask */
void ChassisTask(void *argument)
{
  /* USER CODE BEGIN ChassisTask */
  /* Infinite loop */
  for(;;)
  {
    Chassis_Run();
		sys_mnt.cnt.chassis_task++;
    osDelay(1);
  }
  /* USER CODE END ChassisTask */
}

/* USER CODE BEGIN Header_SendTask */
/**
* @brief Function implementing the send_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendTask */
void SendTask(void *argument)
{
  /* USER CODE BEGIN SendTask */
  /* Infinite loop */
  for(;;)
  {
    if(V6_SendData(&hfdcan1,0x200, Chassis.wheels.rightup.driver_output, Chassis.wheels.rightdown.driver_output, Chassis.wheels.rightup.steer_output, Chassis.wheels.rightdown.steer_output) == HAL_OK){
      sys_mnt.cnt.can1_v6_tx++;
    }
    if(V6_SendData(&hfdcan2,0x200, Chassis.wheels.leftup.driver_output, Chassis.wheels.leftdown.driver_output, Chassis.wheels.leftup.steer_output, Chassis.wheels.leftdown.steer_output) == HAL_OK){
      sys_mnt.cnt.can2_v6_tx++;
    }
		sys_mnt.cnt.send_task++;
    osDelay(1);
  }
  /* USER CODE END SendTask */
}

/* USER CODE BEGIN Header_ActionTask */
/**
* @brief Function implementing the action_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ActionTask */
void ActionTask(void *argument)
{
  /* USER CODE BEGIN ActionTask */
	uint16_t trt_cnt = 0;
  /* Infinite loop */
  for(;;)
  {
    Send_to_Upper();
		Send_to_Vofa();
		if(Vision_Data.num == To_Vision.num)
			To_Vision.num++;
		if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&To_Vision, sizeof(TO_VISION)) == HAL_OK){
			sys_mnt.cnt.vision_tx ++;
		};
		
		trt_cnt++;
		if(trt_cnt >= 250){
			
			TRT_data.data[0] = data_from_upper.QR_byte;
			TRT_data.data[1] = data_from_upper.QR_byte;
			TRT_data.data[2] = data_from_upper.QR_byte;
			HAL_UART_Transmit_DMA (&huart2, (uint8_t *)&TRT_data, sizeof(TRT_DATA));
			sys_mnt.cnt.trt_tx++; 
			trt_cnt = 0;
		}		
		
		
		sys_mnt.cnt.action_task++;
		Process_from_upper();
    osDelay(1);
  }
  /* USER CODE END ActionTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

