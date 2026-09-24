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
#include "can.h"
#include "usart.h"
#include "StretchableArm.h"
#include "DoubleArm.h"
#include "Robot.h"
#include "CANBsp.h"
#include <string.h>
#include <math.h>
#include "dm_motor.h"
#include "LK_motor.h"
#include "DJI_motor.h"
#include "J60_motor.h"
#include "StateMachine.h"
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
osThreadId WithinBoardCommHandle;
osThreadId SendCANmsgHandle;
osThreadId CalculationHandle;
osThreadId StateMachineHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void withinBoardCommunicate(void const * argument);
void sendCANmsg(void const * argument);
void calculation(void const * argument);
void myStateMachine(void const * argument);

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
  /* definition and creation of WithinBoardComm */
  osThreadDef(WithinBoardComm, withinBoardCommunicate, osPriorityNormal, 0, 128);
  WithinBoardCommHandle = osThreadCreate(osThread(WithinBoardComm), NULL);

  /* definition and creation of SendCANmsg */
  osThreadDef(SendCANmsg, sendCANmsg, osPriorityIdle, 0, 1024);
  SendCANmsgHandle = osThreadCreate(osThread(SendCANmsg), NULL);

  /* definition and creation of Calculation */
  osThreadDef(Calculation, calculation, osPriorityIdle, 0, 512);
  CalculationHandle = osThreadCreate(osThread(Calculation), NULL);

  /* definition and creation of StateMachine */
  osThreadDef(StateMachine, myStateMachine, osPriorityIdle, 0, 1024);
  StateMachineHandle = osThreadCreate(osThread(StateMachine), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_withinBoardCommunicate */
	UBaseType_t this_stack_min=0;
	eTaskState this_task_state;
/**
  * @brief  Function implementing the WithinBoardComm thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_withinBoardCommunicate */
void withinBoardCommunicate(void const * argument)
{
  /* USER CODE BEGIN withinBoardCommunicate */
	TxBufToZGT[0]=0x33;
	TxBufToZGT[16]=0x55;

//	uint8_t calc_cnt=0;
  /* Infinite loop */
  for(;;)
  {
//		TxBufToZGT[1]=feedback_DA;
//		TxBufToZGT[2]=feedback_SA;
		for(uint16_t i=1;i<15;i++){
			TxBufToZGT[i]=0;
		}
		TxBufToZGT[15]=system_monitor.stretch_LK_fps/10;
		switch(feedback_DA){
			case 9:
				TxBufToZGT[1]=10;
				break;
			case 10:
				TxBufToZGT[2]=10;
				break;
			case 20:
				TxBufToZGT[3]=10;
				break;
			case 30:
				TxBufToZGT[4]=10;
				break;
			case 40:
				TxBufToZGT[5]=10;
				break;
			default:
				break;
		}
		switch(feedback_SA){
			case 10:
				TxBufToZGT[6]=10;
				break;
			case 20:
				if(height_flag){
					TxBufToZGT[7]=10;
				}else{
					TxBufToZGT[8]=10;
				}
				break;
			case 30:
				TxBufToZGT[10]=10;
				break;
			case 40:
				TxBufToZGT[12]=10;
				break;
			case 50:
				TxBufToZGT[14]=10;
			
				break;
			case 60:
				TxBufToZGT[13]=10;
				break;
			case 70:
				TxBufToZGT[11]=10;
				break;
			case 80:
				TxBufToZGT[9]=10;
				break;
			default:
				break;
		}
		if (Tx_Completed_Flag){
			//Manage Tx Msg
      HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, TxBufToZGT, sizeof(TxBufToZGT));
			if (status == HAL_OK) 
			{
        Tx_Completed_Flag = 0;  
				system_monitor.WBCT_cnt++;
			}
		}
//		this_stack_min=uxTaskGetStackHighWaterMark(WithinBoardCommHandle);
//		this_task_state=eTaskGetState(WithinBoardCommHandle);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END withinBoardCommunicate */
}

/* USER CODE BEGIN Header_sendCANmsg */
/**
* @brief Function implementing the SendCANmsg thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sendCANmsg */
void sendCANmsg(void const * argument)
{
  /* USER CODE BEGIN sendCANmsg */
	uint8_t cnt_send=0;
//	leftShoulderCMD.kd_=0;
//	leftShoulderCMD.kp_=0;
//	leftShoulderCMD.motor_id_=1;
//	leftShoulderCMD.torque_=0;
//	rightShoulderCMD.kd_=0;
//	rightShoulderCMD.kp_=0;
//	rightShoulderCMD.motor_id_=2;
//	rightShoulderCMD.torque_=0;
	leftShoulderCMD.kd_=4;
	leftShoulderCMD.kp_=98;
	leftShoulderCMD.position_=0.25;
	leftShoulderCMD.motor_id_=1;
	leftShoulderCMD.torque_=0;
	rightShoulderCMD.kd_=4;
	rightShoulderCMD.kp_=86;
	rightShoulderCMD.position_=-0.2;
	rightShoulderCMD.motor_id_=2;
	rightShoulderCMD.torque_=0;
	g_cur_LK=0;
	stretch_LK.outerTarget=-118;
	stretch_DM.ctrl.kp_set=30;
	stretch_DM.ctrl.kd_set=1.2;
	stretch_DM.ctrl.pos_set=2.59;
  /* Infinite loop */
  for(;;)
  {
		SetMotionCMD(&leftShoulderCMD,leftShoulder.motor_id_,4,leftShoulderCMD.position_,leftShoulderCMD.velocity_,gTorqueLeft,leftShoulderCMD.kp_,leftShoulderCMD.kd_);
    SetMotionCMD(&rightShoulderCMD,rightShoulder.motor_id_,4,rightShoulderCMD.position_,rightShoulderCMD.velocity_,gTorqueRight,rightShoulderCMD.kp_,rightShoulderCMD.kd_);
    leftShoulderCMD.cmd_=4;
//		CANx_SendstdData(&hcan1,0x300,AirOperaterCtrl,1);
		CAN_Send_DeepMsg(&leftShoulderCMD);
		rightShoulderCMD.cmd_=4;
		CAN_Send_DeepMsg(&rightShoulderCMD);
    CAN_8016_SendCurrent_Single(&hcan1,0x141,stretch_LK.lk_pid.inner.fpU+g_cur_LK);
//		CAN_8016_SendCurrent_Single(&hcan1,0x141,0);
		cnt_send++;
		if(cnt_send==2){
			CAN_Sendcurrent(&hcan2,0x200,left_2006_1.motor_pid.inner.fpU,left_2006_2.motor_pid.inner.fpU,right_2006_1.motor_pid.inner.fpU,right_2006_2.motor_pid.inner.fpU);
//			CAN_Sendcurrent(&hcan2,0x200,left_2006_1.motor_pid.inner.fpU,left_2006_2.motor_pid.inner.fpU,0,0);
			CAN_Sendcurrent(&hcan2,0x1FF,stretch_2006.motor_pid.inner.fpU,0,0,0);
			mit_ctrl(&hcan2,&stretch_DM,stretch_DM.id,stretch_DM.ctrl.pos_set,stretch_DM.ctrl.vel_set,stretch_DM.ctrl.kp_set,stretch_DM.ctrl.kd_set,gTorqueDM);
			cnt_send=0;
		}
//		    leftShoulderCMD.cmd_=4;
////		CANx_SendstdData(&hcan1,0x300,AirOperaterCtrl,1);
//		CAN_Send_DeepMsg(&leftShoulderCMD);
//		rightShoulderCMD.cmd_=4;
//		CAN_Send_DeepMsg(&rightShoulderCMD);
//    CAN_8016_SendCurrent_Single(&hcan2,0x141,stretch_LK.lk_pid.inner.fpU+g_cur_LK);
////		CAN_8016_SendCurrent_Single(&hcan1,0x141,0);
//		cnt_send++;
//		if(cnt_send==2){
//			CAN_Sendcurrent(&hcan1,0x200,left_2006_1.motor_pid.inner.fpU,left_2006_2.motor_pid.inner.fpU,right_2006_1.motor_pid.inner.fpU,right_2006_2.motor_pid.inner.fpU);
////			CAN_Sendcurrent(&hcan2,0x200,left_2006_1.motor_pid.inner.fpU,left_2006_2.motor_pid.inner.fpU,0,0);
//			CAN_Sendcurrent(&hcan1,0x1FF,stretch_2006.motor_pid.inner.fpU,0,0,0);
//			mit_ctrl(&hcan1,&stretch_DM,stretch_DM.id,stretch_DM.ctrl.pos_set,stretch_DM.ctrl.vel_set,stretch_DM.ctrl.kp_set,stretch_DM.ctrl.kd_set,gTorqueDM);
//			cnt_send=0;
//		}
    system_monitor.SCMT_cnt++;
		vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END sendCANmsg */
}

/* USER CODE BEGIN Header_calculation */
uint8_t big_flag,lil_flag;
fp32 roll_L0,pitch_L0,roll_R0,pitch_R0;
uint8_t test_mode=0,error_flag=0;
// ¶ÁÈ¡´íÎó¼ÆÊýÆ÷
uint32_t esr_value;
uint8_t tec;
uint8_t rec;
uint8_t can2_error_flag,can2_error_flag_passive,lec,boff;
uint32_t ESR,TEC,LEC,REC;
/**
* @brief Function implementing the Calculation thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_calculation */
void calculation(void const * argument)
{
  /* USER CODE BEGIN calculation */
	DJIMotorStart();
	LK_PID_Set();
	AirOperaterCtrlBuf[5]=1;
	pushAndPull(big_flag,0);
	pushAndPull(lil_flag,1);
  /* Infinite loop */
  for(;;)
  {
//		pushAndPull(big_flag,0);
//		pushAndPull(lil_flag,1);
		gravityCompensation_SAtest();
		gravityCompensation_DAtest(gravityCompensation_DA_state);
//		wristCtrl_L(roll_L0,pitch_L0,&wrist_L);
//		wristCtrl_L(roll_R0,pitch_R0,&wrist_R);
//		gravityCompensation_SA(gravityCompensation_SA_state);
//		gravityCompensation_DA(gravityCompensation_DA_state);
		DJI_Pos_Ctrl(wrist_L.motor1,2*wrist_L.rollCtrl+wrist_L.pitchCtrl);
		DJI_Pos_Ctrl(wrist_L.motor2,2*wrist_L.rollCtrl-wrist_L.pitchCtrl);
		DJI_Pos_Ctrl(wrist_R.motor1,2*wrist_R.rollCtrl+wrist_R.pitchCtrl);
		DJI_Pos_Ctrl(wrist_R.motor2,2*wrist_R.rollCtrl-wrist_R.pitchCtrl);
		DJI_Pos_Ctrl(&stretch_2006,stretch_2006.outerTarget);
		LK_Pos_Ctrl(&stretch_LK,stretch_LK.outerTarget);
		AirOperaterCtrl[0]=bin_array_to_u8(AirOperaterCtrlBuf);
		if(test_mode==1){
			if(system_monitor.stretch_DM_fps==0){
				error_flag=1;
			}
		}
		
		
		esr_value = hcan2.Instance->ESR;
		tec = (esr_value & CAN_ESR_TEC) >> 24;
		rec = (esr_value & CAN_ESR_REC) >> 16;
		lec = (esr_value & CAN_ESR_LEC) >> 4;
		boff = (esr_value & CAN_ESR_BOFF) >> 2;

		// ×´Ì¬ÅÐ¶Ï
//		if (boff) {
//				can2_error_flag=1;
//				HAL_CAN_DeInit(&hcan2);
//				HAL_CAN_Init(&hcan2);
//				can2_start();
//				boff=0;			
//		} else if (tec > 127 || rec > 127) {
//				can2_error_flag_passive=1;
//		}else{
//			can2_error_flag=0;
//		}
		system_monitor.CT_cnt++;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END calculation */
}

/* USER CODE BEGIN Header_myStateMachine */
/**
* @brief Function implementing the StateMachine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_myStateMachine */
void myStateMachine(void const * argument)
{
  /* USER CODE BEGIN myStateMachine */
	
		uint32_t high_tec_count = 0;
  /* Infinite loop */
  for(;;)
  {
 		stateMachine();
//		ESR=hcan1.Instance->ESR;
//		TEC= (ESR >> 16) & 0xFF;
//		uint32_t esr = CAN1->ESR;
//    uint32_t tec = (esr >> 16) & 0xFF;  
//    LEC=(ESR& CAN_ESR_LEC) >> 4;
//    
//    static uint32_t high_tec_count = 0;
//    
//    if (tec > 200) {
//        high_tec_count++;
//        
//        
//        if (high_tec_count >= 3) {            
//            
//            CAN1->MCR |= CAN_MCR_INRQ;
//            
//            
//            uint32_t timeout = 1000;
//            while (!(CAN1->MSR & CAN_MSR_INAK) && timeout--) {
//                
//            }
//            
//            vTaskDelay(1);
//            
//            CAN1->MCR &= ~CAN_MCR_INRQ;
//            
//            timeout = 1000;
//            while ((CAN1->MSR & CAN_MSR_INAK) && timeout--) {
//            }
//            
//            high_tec_count = 0;
//        }
//    } else {
//        high_tec_count = 0;
//    }
		system_monitor.MSMT_cnt++;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END myStateMachine */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
