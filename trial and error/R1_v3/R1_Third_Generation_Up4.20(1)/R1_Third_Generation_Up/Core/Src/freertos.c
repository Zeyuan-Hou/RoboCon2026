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
#include "Logic.h"
#include "Trajectory.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Motor_Ctrl */
osThreadId_t Motor_CtrlHandle;
const osThreadAttr_t Motor_Ctrl_attributes = {
  .name = "Motor_Ctrl",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Access_Block */
osThreadId_t Access_BlockHandle;
const osThreadAttr_t Access_Block_attributes = {
  .name = "Access_Block",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Access_Pole */
osThreadId_t Access_PoleHandle;
const osThreadAttr_t Access_Pole_attributes = {
  .name = "Access_Pole",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Communicate */
osThreadId_t CommunicateHandle;
const osThreadAttr_t Communicate_attributes = {
  .name = "Communicate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Trajectory */
osThreadId_t TrajectoryHandle;
const osThreadAttr_t Trajectory_attributes = {
  .name = "Trajectory",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Start_Motor_Ctrl(void *argument);
void Start_Access_Block(void *argument);
void Start_Access_Pole(void *argument);
void Start_Communicate(void *argument);
void Start_Trajectory(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Motor_Ctrl */
  Motor_CtrlHandle = osThreadNew(Start_Motor_Ctrl, NULL, &Motor_Ctrl_attributes);

  /* creation of Access_Block */
  Access_BlockHandle = osThreadNew(Start_Access_Block, NULL, &Access_Block_attributes);

  /* creation of Access_Pole */
  Access_PoleHandle = osThreadNew(Start_Access_Pole, NULL, &Access_Pole_attributes);

  /* creation of Communicate */
  CommunicateHandle = osThreadNew(Start_Communicate, NULL, &Communicate_attributes);

  /* creation of Trajectory */
  TrajectoryHandle = osThreadNew(Start_Trajectory, NULL, &Trajectory_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_Motor_Ctrl */
/**
* @brief Function implementing the Motor_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Motor_Ctrl */
void Start_Motor_Ctrl(void *argument)
{
  /* USER CODE BEGIN Start_Motor_Ctrl */
  /* Infinite loop */
  for(;;)
  {
    Motor_CAN_Send();
    test[1]++;	  
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Start_Motor_Ctrl */
}

/* USER CODE BEGIN Header_Start_Access_Block */
/**
* @brief Function implementing the Access_Block thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Access_Block */
void Start_Access_Block(void *argument)
{
  /* USER CODE BEGIN Start_Access_Block */
  /* Infinite loop */
  for(;;)
  {
    BlockArm_Logic();
    test[2]++;	  
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Start_Access_Block */
}

/* USER CODE BEGIN Header_Start_Access_Pole */
/**
* @brief Function implementing the Access_Pole thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Access_Pole */
void Start_Access_Pole(void *argument)
{
  /* USER CODE BEGIN Start_Access_Pole */
  /* Infinite loop */
  for(;;)
  {
    PoleArm_Logic();
//	PoleArm_Lower_Solenoid_Valve_State=1;
    PoleArm_Adjustment();
    test[3]++;	  
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Start_Access_Pole */
}

/* USER CODE BEGIN Header_Start_Communicate */
/**
* @brief Function implementing the Communicate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Communicate */
void Start_Communicate(void *argument)
{
  /* USER CODE BEGIN Start_Communicate */
  /* Infinite loop */
  for(;;)
  {
	Communicate_Task();
	test[4]++;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Start_Communicate */
}

/* USER CODE BEGIN Header_Start_Trajectory */
/**
* @brief Function implementing the Trajectory thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Trajectory */
void Start_Trajectory(void *argument)
{
  /* USER CODE BEGIN Start_Trajectory */
  /* Infinite loop */
  for(;;)
  {
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    test[5]++;
//	if(Traj_Flag==1)
//	{
//		Traj_Angle_Init(&Traj_Gimbal,BlockArm_Gimbal_J60Data.position_,360.f,90.f,5.f);
//		Traj_Arc_Init(&Traj_Arm,Arm_BackSolving.x+100.f,Arm_BackSolving.y,100.f,180.f,-180.f,100.f,100.f);
//		Traj_Flag=0;
//	}
//	if(Traj_Flag==2)
//	{
//		Traj_Angle_Init(&Traj_Gimbal,BlockArm_Gimbal_J60Data.position_,0.f,90.f,5.f);
//		Traj_Line_Init(&Traj_Arm,Arm_BackSolving.x,Arm_BackSolving.y,400.f,200.f,200,100.f);
//		Traj_Flag=0;
//	}
	  
	  
	  
	Traj_Angle_Update(&Traj_BlockArm_Joint3,&MotorInput.BlockArm_Joint3_3508,&BlockArm_Joint3_3508.target_anglev,&BlockArm_Joint3_3508.target_acc);
	Traj_Angle_Update(&Traj_Gimbal,&MotorInput.BlockArm_Gimbal_J60,&YSC_Vel[2],&acc);
	Traj_Line_Update(&Traj_Arm,&Arm_BackSolving);
	Traj_Arc_Update(&Traj_Arm,&Arm_BackSolving);
  }
  /* USER CODE END Start_Trajectory */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

