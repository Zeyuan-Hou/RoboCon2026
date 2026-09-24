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
#include "Global_Variables.h"
#include "CAN_Bsp.h"
#include "State_Machine.h"
#include "Gravity_Feedforward.h"
#include "Communication.h"
#include "Debug.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "J60_Motor.h"
#include "Unitree_A1.h"
#include "Gyro.h"
#include "Robstride.h"
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
uint8_t mode = 0;
/* USER CODE END Variables */
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
  .name = "Task4",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task5 */
osThreadId_t Task5Handle;
const osThreadAttr_t Task5_attributes = {
  .name = "Task5",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task6 */
osThreadId_t Task6Handle;
const osThreadAttr_t Task6_attributes = {
  .name = "Task6",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void State_Machine(void *argument);
void Gravity_Feedforward(void *argument);
void Motor_Driving(void *argument);
void Communication(void *argument);
void Others(void *argument);
void StartTask06(void *argument);

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
  /* creation of Task1 */
  Task1Handle = osThreadNew(State_Machine, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(Gravity_Feedforward, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(Motor_Driving, NULL, &Task3_attributes);

  /* creation of Task4 */
  Task4Handle = osThreadNew(Communication, NULL, &Task4_attributes);

  /* creation of Task5 */
  Task5Handle = osThreadNew(Others, NULL, &Task5_attributes);

  /* creation of Task6 */
  Task6Handle = osThreadNew(StartTask06, NULL, &Task6_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_State_Machine */
/**
 * @brief  Function implementing the Task1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_State_Machine */
void State_Machine(void *argument)
{
  /* USER CODE BEGIN State_Machine */
    /* Infinite loop */
    for (;;)
    {
        StateMachine();

        PackDataToZGT();
        HAL_UART_Transmit_DMA(&huart4, TxBufToZGT, sizeof(TxBufToZGT));

        system_monitor.cntMonitor.Task1++;
        osDelay(1);
    }
  /* USER CODE END State_Machine */
}

/* USER CODE BEGIN Header_Gravity_Feedforward */
/**
 * @brief Function implementing the Task2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Gravity_Feedforward */
void Gravity_Feedforward(void *argument)
{
  /* USER CODE BEGIN Gravity_Feedforward */
    /* Infinite loop */
    for (;;)
    {
        G_Feedforward_Calc();
        system_monitor.cntMonitor.Task2++;
        osDelay(1);
    }
  /* USER CODE END Gravity_Feedforward */
}

/* USER CODE BEGIN Header_Motor_Driving */
/**
 * @brief Function implementing the Task3 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Motor_Driving */
void Motor_Driving(void *argument)
{
  /* USER CODE BEGIN Motor_Driving */
    /* Infinite loop */
    for (;;)
    {
        if (ace.act_state == 255)
        {
            mode = 0;
            m3508_ctrl_flag = 0;
        }
        else if (ace.act_state == 20)
        {
            mode = 1;
            m3508_ctrl_flag = 1;
        }

        // DM_param_set(mode);
        // mit_ctrl(&hfdcan1, &stretch_DM, stretch_DM.id, stretch_DM.ctrl.pos_set, stretch_DM.ctrl.vel_set, stretch_DM.ctrl.kp_set, stretch_DM.ctrl.kd_set, stretch_DM.ctrl.tor_set);

        RS_param_set(mode);
        RobStride_Motor_MIT_Control(&RobStride_01, RS_des, 0.0f, RS_kp, RS_kd, RS_tor); 

        J60_param_set(mode);
        SetMotionCMD(&Gimbal_J60_CMD, Gimbal_J60.motor_id_, 4, Gimbal_J60_CMD.position_, Gimbal_J60_CMD.velocity_, Gimbal_J60_CMD.torque_, Gimbal_J60_CMD.kp_, Gimbal_J60_CMD.kd_);
        CAN_Send_DeepMsg(&Gimbal_J60_CMD);

        A1_param_set(mode);
        motor_A1_cmd(&Motor_A1);

        if (m3508_ctrl_flag == 0)
        {
          CAN_Sendcurrent(&hfdcan2, 0x200, 0, 0, 0, 0);
        }
        else
        {
            M3508_param_set(m3508_ctrl_flag);
            DJIMotorControl(&DJI_3508);
            CAN_Sendcurrent(&hfdcan2, 0x200, DJI_3508.motor_current + G_ff.M3508, 0, 0, 0);
        }

        M2006_param_set(mode);
        DJIMotorControl(&stretch_2006);
        CAN_Sendcurrent(&hfdcan1, 0x200, 0, stretch_2006.motor_current + G_ff.M2006, 0, 0);

        airOperator.airOperatorTx = bin_array_to_u8(airOperator.airOperatorTxBuf);
        CANx_SendstdData(&hfdcan3, 0x300, &airOperator.airOperatorTx, sizeof(uint8_t));

        system_monitor.cntMonitor.Task3++;
        osDelay(1);
    }
  /* USER CODE END Motor_Driving */
}

/* USER CODE BEGIN Header_Communication */
/**
 * @brief Function implementing the Task4 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Communication */
void Communication(void *argument)
{
  /* USER CODE BEGIN Communication */
    /* Infinite loop */
    for (;;)
    {
       VOFA_load_data();
       VOFA_transmit_data(vofa, 50);

       system_monitor.cntMonitor.Task4++;
       osDelay(1);
    }
  /* USER CODE END Communication */
}

/* USER CODE BEGIN Header_Others */
/**
 * @brief Function implementing the Task5 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Others */
void Others(void *argument)
{
  /* USER CODE BEGIN Others */
    /* Infinite loop */
    for (;;)
    {
        Observation();

        // 陀螺仪YS320
        if (g_uart_rx_cnt > 0)
        {
            memcpy(g_decode_data + g_decode_data_pos, g_uart_rx_buf, g_uart_rx_cnt);
            g_decode_data_pos += g_uart_rx_cnt;
            g_uart_rx_cnt = 0;
        }
        if (g_decode_data_pos > 0)
        {
            analysis_data(g_decode_data, g_decode_data_pos);
        }

        system_monitor.cntMonitor.Task5++;
        osDelay(1);
    }
  /* USER CODE END Others */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the Task6 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
    IR_tim++;
    if(IR_tim >= 250)
    {
      HAL_UART_Transmit_DMA(&huart2, Tx_IR, sizeof(Tx_IR));
      IR_tim = 0;
    }

    system_monitor.cntMonitor.Task6++;
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

