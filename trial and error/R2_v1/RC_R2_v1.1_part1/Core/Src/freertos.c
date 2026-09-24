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
#include "global_declare.h"
#include "motor_dji_sdk.h"
#include "j60_sdk.h"
#include "go1_sdk.h"
#include "NRF24L01.h"
#include "remote_control.h"
#include "navigate.h"
#include "chassis.h"
#include "foot.h"
#include "board_communicate.h"
#include "vofa.h"
#include "vision.h"
#include "action.h"
#include "route_plan.h"
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
extern UART_HandleTypeDef huart1, huart4;
/* USER CODE END Variables */
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
    .name = "Task1",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
    .name = "Task2",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
    .name = "Task3",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
    .name = "Task4",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task5 */
osThreadId_t Task5Handle;
const osThreadAttr_t Task5_attributes = {
    .name = "Task5",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Task6 */
osThreadId_t Task6Handle;
const osThreadAttr_t Task6_attributes = {
    .name = "Task6",
    .stack_size = 256 * 4,
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
    monitor.rate_cnt.task1++;
    Foot_WorkLoop();

    J60_LOOP_CONTROL(&hcan2, &j60_motor_cmd_down, foot.down_aim_angle, 0,cyt_vel + foot_g_feedforward.down, j60_motor_cmd_down.kp_, j60_motor_cmd_down.kd_, &j60_motor_data_down);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    GO1_Tx(&go1_send_left, &huart4, foot.leftup_aim_angle, 0, foot_g_feedforward.leftup, go1_send_left.K_P, go1_send_left.K_W);
    osDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    GO1_Tx(&go1_send_right, &huart4, foot.rightup_aim_angle, 0, foot_g_feedforward.rightup, go1_send_right.K_P, go1_send_right.K_W);
		CAN_SendCurrent(&hcan1, 0x1FF,backfoot_current,0,0,0);
    osDelay(pdMS_TO_TICKS(1));
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
    monitor.rate_cnt.task2++;
    Navigate_Task();
		backfoot_motor.Input_v=backfoot_vel;
		DJI_ControlLoop(&backfoot_motor);
		backfoot_current=ClipFloat(backfoot_motor.motor_current+backfoot_vel*K_backfoot_vel,-12300,12300);
    osDelay(pdMS_TO_TICKS(1));
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
  // leftup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
  // rightup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
  // leftdown_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
  // rightdown_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
  /* Infinite loop */
  for (;;)
  {
    monitor.rate_cnt.task3++;
    CS_Run();

    Drive_Chassis();

    // leftup_turn_motor.Input = steer_pos.leftup * TURN_GEAR_RATIO;
    // leftdown_turn_motor.Input = steer_pos.leftdown * TURN_GEAR_RATIO;
    // rightdown_turn_motor.Input = steer_pos.rightdown * TURN_GEAR_RATIO;
    // rightup_turn_motor.Input = steer_pos.rightup * TURN_GEAR_RATIO;

    // DJI_ControlLoop(&leftup_turn_motor);
    // DJI_ControlLoop(&rightdown_turn_motor);
    // DJI_ControlLoop(&rightup_turn_motor);
    // DJI_ControlLoop(&leftdown_turn_motor);

    // CAN_SendCurrent(&hcan2, 0x200, leftup_turn_motor.motor_current, 0, rightup_turn_motor.motor_current, 0);
    // CAN_SendCurrent(&hcan1, 0X200, leftdown_turn_motor.motor_current, 0, rightdown_turn_motor.motor_current, 0);

    osDelay(pdMS_TO_TICKS(1));
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
  /* Infinite loop */
  for (;;)
  {
    monitor.rate_cnt.task4++;
    Vision_Transmit();
    Route_Update();
    Upper_Lower_Communication();

    VOFA_pack_data();
    VOFA_transmit_data(vofa, 20);

    osDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END TASK4 */
}

/* USER CODE BEGIN Header_TASK5 */
/**
 * @brief Function implementing the Task5 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TASK5 */
void TASK5(void *argument)
{
  /* USER CODE BEGIN TASK5 */
  /* Infinite loop */
  for (;;)
  {
    monitor.rate_cnt.task5++;

//    if (!NRF24L01_Check() && !FLAG_NRF)
//    {
//      NRF24L01_RX_Mode();
//      FLAG_NRF = 1;
//    }

//    if (FLAG_NRF == 1)
//    {
//      nRF24L01_ack_pay.Ack_Channel = 0;
//      nRF24L01_ack_pay.Ack_Status = 3;
//      nRF24L01_ack_pay.Ack_Len = ACK_PLOAD_WIDTH;

//      parseDataPacket(nRF24L01_RxBuf, &Js_Value);
//      Deal_Key_State(&Js_Value);

//      NRF24L01_RxPacket(nRF24L01_RxBuf);
//      Send_To_RemoteControl();
//      NRF24L01_Rx_AckPayload(nRF24L01_ack_pay);

//      time_cnt = tim_ms;
//      if (time_cnt - time_cnt_pre >= 1000)
//      {
//        NRF24L01_RX_Mode();
//        time_cnt_pre = time_cnt;
//      }
//    }
Send_To_RemoteControl();//·¢ËÍ
    osDelay(pdMS_TO_TICKS(1));
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
    monitor.rate_cnt.task6++;
		if(flag_route_plan)
    {
			Step_State();
			Route_Plan_choose();
			Block_Plan();
			flag_route_plan=0;
		}
		if(flag_path_to_zero)
    {
			memset(&step_state, 0, sizeof(STEP_STATE));
			memset(&route_plan, 0, sizeof(ROUTE_PLAN));
			flag_path_to_zero=0;
		}

    Action_choose();
    osDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END TASK6 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
