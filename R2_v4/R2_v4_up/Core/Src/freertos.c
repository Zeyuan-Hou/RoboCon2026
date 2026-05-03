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
#include "fdcan.h"
#include "usart.h"
#include "GravityCompensation.h"
#include "ROBOT.h"
#include "CAN_Bsp.h"
#include "dm_motor.h"
#include "DJI_motor.h"
#include "J60_motor.h"
#include "StateMachine.h"
#include "MathAlgorithm.h"
#include "unitree_motor.h"
#include "withinBoardCommunication.h"
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
float g_current = 0.f;
/* USER CODE END Variables */
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
    .name = "Task1",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
    .name = "Task2",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
    .name = "Task3",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
    .name = "Task4",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task5 */
osThreadId_t Task5Handle;
const osThreadAttr_t Task5_attributes = {
    .name = "Task5",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task6 */
osThreadId_t Task6Handle;
const osThreadAttr_t Task6_attributes = {
    .name = "Task6",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TASK1(void *argument);
void TASK2(void *argument);
void TASK3(void *argument);
void TASK4(void *argument);
void TASK5(void *argument);
void TASK6(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  /* creation of Task1 */
  Task1Handle = osThreadNew(TASK1, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(TASK2, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(TASK3, NULL, &Task3_attributes);

  /* creation of Task4 */
  Task4Handle = osThreadNew(TASK4, NULL, &Task4_attributes);

  /* creation of Task5 */
  Task5Handle = osThreadNew(TASK5, NULL, &Task5_attributes);

  /* creation of Task6 */
  Task6Handle = osThreadNew(TASK6, NULL, &Task6_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_TASK1 */
/**
 * @brief  Function implementing the Task1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_TASK1 */
void TASK1(void *argument)
{
  /* USER CODE BEGIN TASK1 */
  /* Infinite loop */
  for (;;)
  {
    // myStateMachine();
    system_monitor.cntMonitor.StateMachineTask++;
    osDelay(1);
  }
  /* USER CODE END TASK1 */
}

/* USER CODE BEGIN Header_TASK2 */
/**
 * @brief Function implementing the Task2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TASK2 */
void TASK2(void *argument)
{
  /* USER CODE BEGIN TASK2 */
  /* Infinite loop */
  for (;;)
  {
    DJIMotorControl(&stretch_2006);
    DJIMotorControl(&DJI_3508);
    airOperator.airOperatorTx = bin_array_to_u8(airOperator.airOperatorTxBuf);
    system_monitor.cntMonitor.PIDCalcTask++;
    osDelay(1);
  }
  /* USER CODE END TASK2 */
}

/* USER CODE BEGIN Header_TASK3 */
/**
 * @brief Function implementing the Task3 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TASK3 */
void TASK3(void *argument)
{
  /* USER CODE BEGIN TASK3 */
  /* Infinite loop */
  for (;;)
  {
    //    gravityCompensation();
    system_monitor.cntMonitor.GravityCompensationTask++;
    osDelay(1);
  }
  /* USER CODE END TASK3 */
}

/* USER CODE BEGIN Header_TASK4 */
/**
 * @brief Function implementing the Task4 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TASK4 */
void TASK4(void *argument)
{
  /* USER CODE BEGIN TASK4 */
  // Gimbal_J60_CMD.kp_ = 230;
  // Gimbal_J60_CMD.kd_ = 10;
  Gimbal_J60_CMD.kp_ = 0;
  Gimbal_J60_CMD.kd_ = 0;
  Gimbal_J60_CMD.torque_ = 0;
  Gimbal_J60_CMD.position_ = -0.22f;
  Gimbal_J60_CMD.motor_id_ = 1;

  // stretch_DM.ctrl.kp_set = 30;
  // stretch_DM.ctrl.kd_set = 1.2f;
  stretch_DM.ctrl.kp_set = 0;
  stretch_DM.ctrl.kd_set = 0;
  stretch_DM.ctrl.pos_set = 2.26f; //-1.97f;

  airOperator.airOperatorTxBuf[0] = 0;
  airOperator.airOperatorTxBuf[1] = 0;
  airOperator.airOperatorTxBuf[2] = 0;
  airOperator.airOperatorTxBuf[3] = 0;
  airOperator.airOperatorTxBuf[4] = 0;
  /* Infinite loop */
  for (;;)
  {
    SetMotionCMD(&Gimbal_J60_CMD, Gimbal_J60.motor_id_, 4, Gimbal_J60_CMD.position_, Gimbal_J60_CMD.velocity_, Gimbal_J60_CMD.torque_, Gimbal_J60_CMD.kp_, Gimbal_J60_CMD.kd_);
    Gimbal_J60_CMD.cmd_ = 4;
    // CAN_Send_DeepMsg(&Gimbal_J60_CMD);

    // mit_ctrl(&hfdcan1, &stretch_DM, stretch_DM.id, stretch_DM.ctrl.pos_set, stretch_DM.ctrl.vel_set, stretch_DM.ctrl.kp_set, stretch_DM.ctrl.kd_set, gTorqueDM);

    // CAN_Sendcurrent(&hfdcan1, 0x200, 0, stretch_2006.motor_current + g_current, 0, 0);

    // CAN_Sendcurrent(&hfdcan2, 0x200, DJI_3508.motor_current, 0, 0, 0);

    // CANx_SendstdData(&hfdcan2, 0x300, &airOperator.airOperatorTx, sizeof(uint8_t));
    system_monitor.cntMonitor.MotorControlTask++;
    osDelay(1);
  }
  /* USER CODE END TASK4 */
}

/* USER CODE BEGIN Header_TASK5 */
/**
 * @brief Function implementing the Task5 thread.
 * @param argument: Not used
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
 
}
/* USER CODE END Header_TASK5 */
void TASK5(void *argument)
{
  /* USER CODE BEGIN TASK5 */
  Motor_A1.Ctrl_Data.ID = 0x01;
  Motor_A1.Ctrl_Data.mode = 0x0A;
  // Motor_A1.Ctrl_Data.K_P = 1.2f;  // 0.16f;
  // Motor_A1.Ctrl_Data.K_W = 12.0f; // 0.5f;
  Motor_A1.Ctrl_Data.K_P = 0.f;  // 0.16f;
  Motor_A1.Ctrl_Data.K_W = 0.0f; // 0.5f;
  Motor_A1.Ctrl_Data.T = 0;
  Motor_A1.Rec_Data.ID = 1;
  Motor_A1.TargetPos = 0.05f;
  /* Infinite loop */
  for (;;)
  {
    PackDataToZGT();
    
      HAL_UART_Transmit_DMA(&huart4, TxBufToZGT, 6);
    
    // Motor_A1.Ctrl_Data.T = gTorqueA1;
    Motor_A1.Ctrl_Data.T = 0;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
    motor_A1_cmd(&Motor_A1);
    system_monitor.cntMonitor.WithinBoardCommunicateTask++;
    osDelay(2);
  }
  /* USER CODE END TASK5 */
}

/* USER CODE BEGIN Header_TASK6 */
/**
 * @brief Function implementing the Task6 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TASK6 */
void TASK6(void *argument)
{
  /* USER CODE BEGIN TASK6 */
  /* Infinite loop */
  for (;;)
  {

    osDelay(1);
  }
  /* USER CODE END TASK6 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
