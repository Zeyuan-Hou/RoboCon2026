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
#include "Robot.h"
#include "CAN_Bsp.h"
#include "USART_Bsp.h"
#include "J60_motor.h"
#include "DJI_motor.h"
#include "MathAlgorithm.h"
#include "usart.h"
#include "StateMachine.h"
#include "UpAction.h"
#include "GravityCompensation.h"
#include "tim.h"
#include "stm32h723xx.h"
#include "tjc_usart_hmi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TJC 1
//#define IRM 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for UpAction */
osThreadId_t UpActionHandle;
const osThreadAttr_t UpAction_attributes = {
  .name = "UpAction",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USART_TRANS */
osThreadId_t USART_TRANSHandle;
const osThreadAttr_t USART_TRANS_attributes = {
  .name = "USART_TRANS",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN_TRANS1 */
osThreadId_t CAN_TRANS1Handle;
const osThreadAttr_t CAN_TRANS1_attributes = {
  .name = "CAN_TRANS1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN_TRANS2 */
osThreadId_t CAN_TRANS2Handle;
const osThreadAttr_t CAN_TRANS2_attributes = {
  .name = "CAN_TRANS2",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ALGORITHM */
osThreadId_t ALGORITHMHandle;
const osThreadAttr_t ALGORITHM_attributes = {
  .name = "ALGORITHM",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for STATE_MACHINE */
osThreadId_t STATE_MACHINEHandle;
const osThreadAttr_t STATE_MACHINE_attributes = {
  .name = "STATE_MACHINE",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void up_action(void *argument);
void usartTransmit(void *argument);
void canTrans1(void *argument);
void canTrans2(void *argument);
void algorithm(void *argument);
void stateMachine(void *argument);

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
  /* creation of UpAction */
  UpActionHandle = osThreadNew(up_action, NULL, &UpAction_attributes);

  /* creation of USART_TRANS */
  USART_TRANSHandle = osThreadNew(usartTransmit, NULL, &USART_TRANS_attributes);

  /* creation of CAN_TRANS1 */
  CAN_TRANS1Handle = osThreadNew(canTrans1, NULL, &CAN_TRANS1_attributes);

  /* creation of CAN_TRANS2 */
  CAN_TRANS2Handle = osThreadNew(canTrans2, NULL, &CAN_TRANS2_attributes);

  /* creation of ALGORITHM */
  ALGORITHMHandle = osThreadNew(algorithm, NULL, &ALGORITHM_attributes);

  /* creation of STATE_MACHINE */
  STATE_MACHINEHandle = osThreadNew(stateMachine, NULL, &STATE_MACHINE_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_up_action */
/**
  * @brief  Function implementing the UpAction thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_up_action */
void up_action(void *argument)
{
  /* USER CODE BEGIN up_action */
	get_KFS_height=3;
  /* Infinite loop */
  for(;;)
  {
		UpAction_weapon();
		UpAction_KFS();
		UpAction_platform();
		systemMonitor.cntMonitor.UpActionTask++;
    osDelay(1);
  }
  /* USER CODE END up_action */
}

/* USER CODE BEGIN Header_usartTransmit */
u8 uart4_txCplt=1,usart1_txCplt=1,usart3_txCplt=1;
u16 tjc_cnt=0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance== UART4){
				uart4_txCplt=1;
		}
    if (huart->Instance == USART1)
    {
        usart1_txCplt = 1; 
    }
		if(huart->Instance == USART3){
				usart3_txCplt=1;
		}
}
/**
* @brief Function implementing the USART_TRANS thread.
* @param argument: Not used
* @retval None
*/
u8 buffer[9]={0};
/* USER CODE END Header_usartTransmit */
void usartTransmit(void *argument)
{
  /* USER CODE BEGIN usartTransmit */
	u8 ack[32]={0};
  /* Infinite loop */
  for(;;)
  {
		packDataToLower(uart4_txbuf);
		if(uart4_txCplt){
			HAL_UART_Transmit_DMA(&huart4,uart4_txbuf,23);
			uart4_txCplt=0;
		}
//		packVisionData(usart1_txbuf);
//		if(usart1_txCplt){
//			HAL_UART_Transmit_DMA(&huart1,usart1_txbuf,3);
//			usart1_txCplt=0;
//		}
		packRemoteData(usart2_txbuf);
		memcpy(ack,usart2_txbuf,20*sizeof(uint8_t));
		if(usart3_txCplt){
			HAL_UART_Transmit_DMA(&huart3,usart2_txbuf,24);
			usart3_txCplt=0;
		}
		tjc_cnt++;
#ifdef TJC		
		if(tjc_cnt==100){
			My_Send_LED(QD_show_1);
			tjc_cnt=0;
		}
#endif
#ifdef IRM
		if(tjc_cnt>250){
			tjc_cnt=0;
			packDataToIRModule(buffer);
			HAL_UART_Transmit_DMA(&huart2,buffer,9);
		}
#endif
		systemMonitor.cntMonitor.UsartCommunicateTask++;
    osDelay(1);
  }
  /* USER CODE END usartTransmit */
}

/* USER CODE BEGIN Header_canTrans1 */
/**
* @brief Function implementing the CAN_TRANS1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canTrans1 */
void canTrans1(void *argument)
{
  /* USER CODE BEGIN canTrans1 */
	
	gimbalJ60CMD.cmd_=4;
	gimbalJ60CMD.kp_=320;
 	gimbalJ60CMD.kd_=10;
	gimbalJ60CMD.position_=0.07f;
	gimbalJ60CMD.motor_id_=0x01;
	gimbalJ60CMD.torque_ =0;
	
  /* Infinite loop */
  for(;;)
  {
		SetMotionCMD(&gimbalJ60CMD,gimbalJ60.motor_id_,4,gimbalJ60CMD.position_,gimbalJ60CMD.velocity_,gimbalJ60CMD.torque_,gimbalJ60CMD.kp_,gimbalJ60CMD.kd_);
		CAN_Send_DeepMsg(&gimbalJ60CMD,&hfdcan1);
		CAN_Sendcurrent(&hfdcan1,0x200,move2006.motor_pid.inner.fpU,weapon_wrist_torque+claw3508.motor_pid.inner.fpU,friction3508.motor_pid.inner.fpU,wrist_torque+wrist3508.motor_pid.inner.fpU);
		
		CAN_SendStdData(&hfdcan3,0x300,airOperator.airOperatorTx,3);
		
		systemMonitor.cntMonitor.CAN_Trans1Task++;
    osDelay(1);
  }
  /* USER CODE END canTrans1 */
}

/* USER CODE BEGIN Header_canTrans2 */
/**
* @brief Function implementing the CAN_TRANS2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canTrans2 */
void canTrans2(void *argument)
{
  /* USER CODE BEGIN canTrans2 */
	
	shoulderJ60CMD.cmd_=4;
	shoulderJ60CMD.kp_=140;
	shoulderJ60CMD.kd_=10;
	shoulderJ60CMD.position_=-0.1f;
	shoulderJ60CMD.motor_id_=0x01;
	shoulderJ60CMD.torque_ =0;
	
	elbowJ60CMD.cmd_=4;
	elbowJ60CMD.kp_=140;
	elbowJ60CMD.kd_=10;
	elbowJ60CMD.position_=-0.1f;
	elbowJ60CMD.motor_id_=0x02;
	elbowJ60CMD.torque_ =0;
  /* Infinite loop */
  for(;;)
  {
		SetMotionCMD(&shoulderJ60CMD,shoulderJ60.motor_id_,4,shoulderJ60CMD.position_,shoulderJ60CMD.velocity_,shoulder_torque,shoulderJ60CMD.kp_,shoulderJ60CMD.kd_);
		CAN_Send_DeepMsg(&shoulderJ60CMD,&hfdcan2);		
		SetMotionCMD(&elbowJ60CMD,elbowJ60.motor_id_,4,elbowJ60CMD.position_,elbowJ60CMD.velocity_,elbow_torque,elbowJ60CMD.kp_,elbowJ60CMD.kd_);
		CAN_Send_DeepMsg(&elbowJ60CMD,&hfdcan2);
		CAN_Sendcurrent(&hfdcan2,0x200,platform_L2006.motor_pid.inner.fpU,platform_R2006.motor_pid.inner.fpU,0,0);
		systemMonitor.cntMonitor.CAN_Trans2Task++;
    osDelay(1);
  }
  /* USER CODE END canTrans2 */
}

/* USER CODE BEGIN Header_algorithm */
/**
* @brief Function implementing the ALGORITHM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_algorithm */
void algorithm(void *argument)
{
  /* USER CODE BEGIN algorithm */
	DJIMotorStart();
  /* Infinite loop */
  for(;;)
  {
		if(move2006_mode==0){
				DJIMotorControl(&move2006);
		}else{
				move2006_align=DJIMotorControlByVision(&move2006);
		}
		if(claw_mode){
			claw3508.outerTarget+=(claw3508.angle<=150.f)*(remoteRec.usJsRight_Y>3000)*((float)(remoteRec.usJsRight_Y-3000))/4000.f+(claw3508.angle>=40.f)*(remoteRec.usJsRight_Y<1000)*((float)(remoteRec.usJsRight_Y-1000))/4000.f;
		}
		DJIMotorControl(&claw3508);

		DJIMotorControl(&wrist3508);
		
		DJIVelControl(&platform_L2006);
		DJIVelControl(&platform_R2006);
		GravityCompensation_weaponPlayer(weapon_compensation_mode);
		GravityCompensation_KFSMaster(KFS_compensation_mode);
		
		bin_array_to_u8(airOperator.airOperatorTxBuf,airOperator.airOperatorTx);
		systemMonitor.cntMonitor.AlgorithmTask++;
    osDelay(1);
  }
  /* USER CODE END algorithm */
}

/* USER CODE BEGIN Header_stateMachine */
void delayTime(u8 *flag){
	switch(*flag){
		case 1:
			delay_timer++;
			if(delay_timer>300){
				QD_show_1=13;//二维码：递块成功
				delay_timer=0;
				*flag=0;
			}
			break;
		case 5:
			delay_timer++;
			if(delay_timer>100&&KFS_cplt){
				delay_timer=0;
				store_KFS_orientation=!store_KFS_orientation;
				*flag=0;
			}
			break;
		case 6:
			delay_timer++;
			if(delay_timer>1200){
				switch(nav_target){
					case PUSH_PLATFORM_FOR_SECOND_COLUMN_R2:
						nav_target=COMBINE_WITH_SECOND_COLUMN_R2;
						chassis_action=3;
						break;
					case PUSH_PLATFORM_FOR_FIRST_COLUMN_R2:
						nav_target=COMBINE_WITH_FIRST_COLUMN_R2;
						chassis_action=3;
						break;
					case MIRRORED_PUSH_PLATFORM_FOR_THIRD_COLUMN_R2:
						nav_target=MIRRORED_PUSH_PLATFORM_FOR_THIRD_COLUMN_R2;
						chassis_action=3;
						break;
					case MIRRORED_PUSH_PLATFORM_FOR_SECOND_COLUMN_R2:
						nav_target=MIRRORED_PUSH_PLATFORM_FOR_SECOND_COLUMN_R2;
						chassis_action=3;
						break;
					default:
						break;
				}
				delay_timer=0;
				*flag=0;
			}
			break;			
		default:
			break;
	}
}
/**
* @brief Function implementing the STATE_MACHINE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stateMachine */
void stateMachine(void *argument)
{
  /* USER CODE BEGIN stateMachine */
  /* Infinite loop */
  for(;;)
  {
		delayTime(&delay_timer_flag);
		StateMachine();
		systemMonitor.cntMonitor.StateMachineTask++;
    osDelay(1);
  }
  /* USER CODE END stateMachine */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

