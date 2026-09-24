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
#include "locate.h"
#include "navigation.h"
#include "NRF24L01.h"
#include "can_bsp.h"
#include "chassis.h"
#include "steeringWheel.h"
#include <string.h>
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
int dt35_location = 0;
int time = 0;
int cnt = 0;
extern CAN_HandleTypeDef hcan1;
/* USER CODE END Variables */
osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osThreadId Task4Handle;
osThreadId Task5Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TASK1(void const * argument);
void TASK2(void const * argument);
void TASK3(void const * argument);
void TASK4(void const * argument);
void TASK5(void const * argument);

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
  /* definition and creation of Task1 */
  osThreadDef(Task1, TASK1, osPriorityNormal, 0, 128);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, TASK2, osPriorityNormal, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* definition and creation of Task3 */
  osThreadDef(Task3, TASK3, osPriorityAboveNormal, 0, 128);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  /* definition and creation of Task4 */
  osThreadDef(Task4, TASK4, osPriorityNormal, 0, 128);
  Task4Handle = osThreadCreate(osThread(Task4), NULL);

  /* definition and creation of Task5 */
  osThreadDef(Task5, TASK5, osPriorityNormal, 0, 128);
  Task5Handle = osThreadCreate(osThread(Task5), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_TASK1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TASK1 */
void TASK1(void const * argument)
{
  /* USER CODE BEGIN TASK1 */

  /* Infinite loop */
  for (;;)
  {
    // 陀螺仪+随动轮+DT35定位
    if (g_uart_rx_cnt > 0)
    {
      memcpy(g_decode_data + g_decode_data_pos, g_uart_rx_buf, g_uart_rx_cnt);
      g_decode_data_pos += g_uart_rx_cnt;
      g_uart_rx_cnt = 0;
    }
    if (g_decode_data_pos > 0)
    {
      analysis_data(g_decode_data, g_decode_data_pos);
      //data_filter();
    }

    dx = stRobot.stPos.fpPosX, dy = stRobot.stPos.fpPosY;
    v = sqrtf((dx - dx_pre) * (dx - dx_pre) + (dy - dy_pre) * (dy - dy_pre)); // mm/s
    dx_pre = dx, dy_pre = dy;

    v_filtered = 0.7f * v + 0.3f * v_pre;
    v_pre = v_filtered;


    Robot_Location(&stRobot, &stFollowerWheel);
    if (dt35_location > 9)
    {
      DT35_relocation_new(&stRobot, &stFollowerWheel, &dt35_save, &dt35_now);
      dt35_location = 0;
    }
    dt35_location++;
    dt35_relocation();

    UpdatePositionFeedback(&nav, &stRobot);
    PositionToVelt();
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
void TASK2(void const * argument)
{
  /* USER CODE BEGIN TASK2 */
  //nav.nav_state = 5;

  //nav.nav_state = 5;
  //nav.auto_path.number = 1;
  // point_end = (POINT){4500.0f, 700.0f, 0.0f};
  //nav.nav_state = 3;

  // nav.nav_state = 6;
  //nav.auto_path.number_permutation = 1;
  /* Infinite loop */
  for(;;)
  {
    // if (nav.nav_state == 2 && cnt == 0)
    // {
    //   // point_end = (POINT){4500.0f, 1500.0f, 0.0f};
    //   // nav.nav_state = 5;
    //   // end_flag=0;
    //   cnt = 1;
    // }
    // else if (nav.nav_state == 2 && cnt == 1){
    //   nav.nav_state = 5;
    //   point_end = (POINT){300.0f, 6000.0f, 0.0f};
    //   cnt = 2;
    // }
    // else if (nav.nav_state == 2 && cnt == 2){
    //   nav.nav_state = 5;
    //   point_end = (POINT){300.0f, -500.0f, 0.0f};
    //   cnt = 3;
    // }
    // else if (nav.nav_state == 2 && cnt == 3){
    //   nav.nav_state = 5;
    //   point_end = (POINT){-400.0f, -500.0f, 0.0f};
    //   cnt = 4;
    // }

      time++;
      if (time<5000)
        leftdown_motor_angle.Input = leftdown_init_angle, rightdown_motor_angle.Input = rightdown_init_angle;
      else if(time<=8000 && time>=5000)
        leftdown_motor_angle.Input = leftdown_init_angle+360.0f, rightdown_motor_angle.Input = rightdown_init_angle+360.0f;
      // else if(time>8000 && time<=11000)
      //   leftdown_motor_angle.Input = leftdown_init_angle+135.f, rightdown_motor_angle.Input = rightdown_init_angle+135.f;
      // else
      //   leftdown_motor_angle.Input = leftdown_init_angle+315.f, rightdown_motor_angle.Input = rightdown_init_angle+315.f;

    //   navigation();

    // if (nav.nav_state != NAV_OFF)
    // {
    //   PID_Calc(&chassis_run.rightdown, chassis_run.rightdown.fpDes, chassis_run.rightdown.fpFB);
    //   PID_Calc(&chassis_run.rightup, chassis_run.rightup.fpDes, chassis_run.rightup.fpFB);
    //   PID_Calc(&chassis_run.leftdown, chassis_run.leftdown.fpDes, chassis_run.leftdown.fpFB);
    //   PID_Calc(&chassis_run.leftup, chassis_run.leftup.fpDes, chassis_run.leftup.fpFB);

    //   chassis_run.leftup.fpU += friction_compensation_current.leftup + feed_forward_current.leftup;
    //   chassis_run.rightup.fpU += friction_compensation_current.rightup + feed_forward_current.rightup;
    //   chassis_run.rightdown.fpU += friction_compensation_current.rightdown + feed_forward_current.rightdown;
    //   chassis_run.leftdown.fpU += friction_compensation_current.leftdown + feed_forward_current.leftdown;

    // //   leftdown_motor_angle.Input = chassis_steer_angle.leftdown_td.aim;
    // //   rightdown_motor_angle.Input = chassis_steer_angle.rightdown_td.aim;
    // }
    //CAN_SendCurrent(&hcan1, 0x200, chassis_run.leftup.fpU, chassis_run.rightup.fpU, chassis_run.rightdown.fpU, chassis_run.leftdown.fpU);
    //CAN_SendCurrent(&hcan1, 0x200, chassis_run.rightup.fpU, chassis_run.rightdown.fpU, chassis_run.leftdown.fpU, chassis_run.leftup.fpU);

    ControlLoop(&leftdown_motor_angle);
    ControlLoop(&rightdown_motor_angle);
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
void TASK3(void const * argument)
{
  /* USER CODE BEGIN TASK3 */
  /* Infinite loop */
  for(;;)
  {

      // if (nav.nav_state == NAV_PERMUTATION_PATH || nav.nav_state == NAV_AUTO_PATH)
      // {
      //   nav.auto_path.run_time++;
      //   nav.auto_path.rotation_time++;
      //   nav.auto_path.run_Sumtime++;
      // }
    //遥控器发送接收
    // if (!NRF24L01_Check() && !FLAG_NRF)
    // {
    //   NRF24L01_RX_Mode();
    //   FLAG_NRF = 1;
    // }
    // if (FLAG_NRF == 1)
    // {
    //   nRF24L01_ack_pay.Ack_Channel = 0;
    //   nRF24L01_ack_pay.Ack_Status = 3;
    //   nRF24L01_ack_pay.Ack_Len = ACK_PLOAD_WIDTH;
    //   NRF24L01_RxPacket(nRF24L01_RxBuf);
    //   NRF24L01_Rx_AckPayload(nRF24L01_ack_pay);

    //   parseDataPacket(nRF24L01_RxBuf, &Js_Value);
    // }
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
void TASK4(void const * argument)
{
  /* USER CODE BEGIN TASK4 */
  /* Infinite loop */
  for(;;)
  { 
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
/* USER CODE END Header_TASK5 */
void TASK5(void const * argument)
{
  /* USER CODE BEGIN TASK5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TASK5 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
