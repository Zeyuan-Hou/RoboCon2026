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
#include "bsp_can.h"
#include "pid.h"
#include "ROBOT.h"
#include "navigation_task.h"
#include "chassis.h"
#include "locate.h"
#include "NRF24L01.h"
#include "remote_control.h"
#include "usart.h"
#include "air_operated.h"
#include "fun_task.h"
#include "servo.h"
#include "math_algorithm.h"
#include "LWIP_UDP_FreeRTOS.h"
#include "smc.h"
#include "vision_data_deal.h"
#include "vision_relocation.h"
#include "algorithm.h"
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
/* Definitions for LWIP_TASK */
osThreadId_t LWIP_TASKHandle;
const osThreadAttr_t LWIP_TASK_attributes = {
  .name = "LWIP_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SYSTEM */
osThreadId_t SYSTEMHandle;
const osThreadAttr_t SYSTEM_attributes = {
  .name = "SYSTEM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LOCATION */
osThreadId_t LOCATIONHandle;
const osThreadAttr_t LOCATION_attributes = {
  .name = "LOCATION",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for NAVIGATION */
osThreadId_t NAVIGATIONHandle;
const osThreadAttr_t NAVIGATION_attributes = {
  .name = "NAVIGATION",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for MOTOR_CONTROL */
osThreadId_t MOTOR_CONTROLHandle;
const osThreadAttr_t MOTOR_CONTROL_attributes = {
  .name = "MOTOR_CONTROL",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for REMOTE_CONTROL */
osThreadId_t REMOTE_CONTROLHandle;
const osThreadAttr_t REMOTE_CONTROL_attributes = {
  .name = "REMOTE_CONTROL",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FUN_TASK */
osThreadId_t FUN_TASKHandle;
const osThreadAttr_t FUN_TASK_attributes = {
  .name = "FUN_TASK",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for COMMUNICATION */
osThreadId_t COMMUNICATIONHandle;
const osThreadAttr_t COMMUNICATION_attributes = {
  .name = "COMMUNICATION",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for GYRO_TASK */
osThreadId_t GYRO_TASKHandle;
const osThreadAttr_t GYRO_TASK_attributes = {
  .name = "GYRO_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Lwip_Task(void *argument);
void System_Monitor(void *argument);
void Location_Task(void *argument);
void Navigation_Task(void *argument);
void Motor_Control(void *argument);
void Remote_Control(void *argument);
void Fun_Task(void *argument);
void Communication_Task(void *argument);
void Gyro_Task(void *argument);

extern void MX_LWIP_Init(void);
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
  /* creation of LWIP_TASK */
  LWIP_TASKHandle = osThreadNew(Lwip_Task, NULL, &LWIP_TASK_attributes);

  /* creation of SYSTEM */
  SYSTEMHandle = osThreadNew(System_Monitor, NULL, &SYSTEM_attributes);

  /* creation of LOCATION */
  LOCATIONHandle = osThreadNew(Location_Task, NULL, &LOCATION_attributes);

  /* creation of NAVIGATION */
  NAVIGATIONHandle = osThreadNew(Navigation_Task, NULL, &NAVIGATION_attributes);

  /* creation of MOTOR_CONTROL */
  MOTOR_CONTROLHandle = osThreadNew(Motor_Control, NULL, &MOTOR_CONTROL_attributes);

  /* creation of REMOTE_CONTROL */
  REMOTE_CONTROLHandle = osThreadNew(Remote_Control, NULL, &REMOTE_CONTROL_attributes);

  /* creation of FUN_TASK */
  FUN_TASKHandle = osThreadNew(Fun_Task, NULL, &FUN_TASK_attributes);

  /* creation of COMMUNICATION */
  COMMUNICATIONHandle = osThreadNew(Communication_Task, NULL, &COMMUNICATION_attributes);

  /* creation of GYRO_TASK */
  GYRO_TASKHandle = osThreadNew(Gyro_Task, NULL, &GYRO_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Lwip_Task */
/**
  * @brief  Function implementing the LWIP_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Lwip_Task */
void Lwip_Task(void *argument)
{
  /* init code for LWIP */
  //MX_LWIP_Init();
  /* USER CODE BEGIN Lwip_Task */
  /* Infinite loop */
  for(;;)
  {
		//udp_task_run();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Lwip_Task */
}

/* USER CODE BEGIN Header_System_Monitor */
/**
* @brief Function implementing the SYSTEM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_System_Monitor */
void System_Monitor(void *argument)
{
  /* USER CODE BEGIN System_Monitor */
  /* Infinite loop */
  for(;;)
  {
    // for(int i=0;i<2;i++)
	// 	{
	// 		system_monitor.can_rec_fps[i]=system_monitor.can_rec_cnt[i];
	// 		system_monitor.can_rec_cnt[i]=0;
	// 	}
		
	// 	system_monitor.motor_LU_fps=system_monitor.motor_LU_cnt;
	// 	system_monitor.motor_LU_cnt=0;
		
	// 	system_monitor.motor_RU_fps=system_monitor.motor_RU_cnt;
	// 	system_monitor.motor_RU_cnt=0;
		
	// 	system_monitor.motor_RD_fps=system_monitor.motor_RD_cnt;
	// 	system_monitor.motor_RD_cnt=0;
		
	// 	system_monitor.motor_LD_fps=system_monitor.motor_LD_cnt;
	// 	system_monitor.motor_LD_cnt=0;
		
	// 	system_monitor.Lift_Left_fps=system_monitor.Lift_Left_cnt;
	// 	system_monitor.Lift_Left_cnt=0;
		
	// 	system_monitor.Lift_Right_fps=system_monitor.Lift_Right_cnt;
	// 	system_monitor.Lift_Right_cnt=0;
		
	// 	system_monitor.Drib_Left_fps=system_monitor.Drib_Left_cnt;
	// 	system_monitor.Drib_Left_cnt=0;
		
	// 	system_monitor.Drib_Right_fps=system_monitor.Drib_Right_cnt;
	// 	system_monitor.Drib_Right_cnt=0;
		
	// 	system_monitor.motor_J60_fps=system_monitor.motor_J60_cnt;
	// 	system_monitor.motor_J60_cnt=0;
		
	// 	system_monitor.air_board_fps=system_monitor.air_board_cnt;
	// 	system_monitor.air_board_cnt=0;
		
	// 	system_monitor.uart1_fps=system_monitor.uart1_cnt;
	// 	system_monitor.uart1_cnt=0;
		
	// 	system_monitor.remote_control_fps=system_monitor.remote_control_cnt;
	// 	system_monitor.remote_control_cnt=0;
		
	// 	system_monitor.uart2_fps=system_monitor.uart2_cnt;
	// 	system_monitor.uart2_cnt=0;
		
	// 	system_monitor.gyro_fps=system_monitor.gyro_cnt;
	// 	system_monitor.gyro_cnt=0;
		
	// 	system_monitor.uart5_fps=system_monitor.uart5_cnt;
	// 	system_monitor.uart5_cnt=0;
		
	// 	if(system_monitor.motor_LU_fps > 1100 || system_monitor.motor_LU_fps < 900)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[0],0);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[0],0);
		
	// 	if(system_monitor.motor_RU_fps > 1100 || system_monitor.motor_RU_fps < 900)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[0],1);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[0],1);
		
	// 	if(system_monitor.motor_RD_fps > 1100 || system_monitor.motor_RD_fps < 900)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[0],2);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[0],2);
			
	// 	if(system_monitor.motor_LD_fps > 1100 || system_monitor.motor_LD_fps < 900)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[0],3);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[0],3);
			
	// 	if(system_monitor.Drib_Left_fps > 1100 || system_monitor.Drib_Left_fps < 900)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[0],4);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[0],4);
			
	// 	if(system_monitor.Drib_Right_fps > 1100 || system_monitor.Drib_Right_fps < 900)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[0],5);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[0],5);
			
	// 	if(system_monitor.motor_J60_fps > 1100 || system_monitor.motor_J60_fps < 900)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[0],6);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[0],6);
			
	// 	if(system_monitor.air_board_fps >220 || system_monitor.air_board_fps <180)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[0],7);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[0],7);
			
	// 	if(system_monitor.uart1_fps >1100 || system_monitor.uart1_fps <900)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],0);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],0);
			
	// 	if(system_monitor.lwip_rec_fps <180)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],1);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],1);
			
	// 	if(system_monitor.uart2_fps <180)
	// 		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],2);
	// 	else 
	// 		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],2);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END System_Monitor */
}

/* USER CODE BEGIN Header_Location_Task */
/**
* @brief Function implementing the LOCATION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Location_Task */
void Location_Task(void *argument)
{
  /* USER CODE BEGIN Location_Task */
  /* Infinite loop */
  for(;;)
  {
   Robot_Location(&stRobot,&stFollowerWheel);
		if(dt35_location>9)
		{
			DT35_relocation_new(&stRobot,&stFollowerWheel,&dt35_save,&dt35_now);
			dt35_location=0;
		}
		dt35_location++;
		Mid_360_location(&stRobot);
		UpdatePositionFeedback(&nav,&stRobot);
		PositionToVelt();
		system_monitor.location_cnt++;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Location_Task */
}

/* USER CODE BEGIN Header_Navigation_Task */
/**
* @brief Function implementing the NAVIGATION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Navigation_Task */
void Navigation_Task(void *argument)
{
  /* USER CODE BEGIN Navigation_Task */
  /* Infinite loop */
  for(;;)
  {
		navigation();
		system_monitor.navigation_cnt++;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Navigation_Task */
}

/* USER CODE BEGIN Header_Motor_Control */
/**
* @brief Function implementing the MOTOR_CONTROL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Control */
void Motor_Control(void *argument)
{
  /* USER CODE BEGIN Motor_Control */
  /* Infinite loop */
  for(;;)
  {
    if(nav.nav_state!=NAV_OFF)
		{
			PID_Calc_NEW(&chassis_run.rightdown);
			PID_Calc_NEW(&chassis_run.rightup);
			PID_Calc_NEW(&chassis_run.leftdown);
			PID_Calc_NEW(&chassis_run.leftup);
			
			chassis_run.leftup.fpU += friction_compensation_current.leftup + feed_forward_current.leftup;
			chassis_run.rightup.fpU += friction_compensation_current.rightup + feed_forward_current.rightup;
			chassis_run.rightdown.fpU += friction_compensation_current.rightdown + feed_forward_current.rightdown;
			chassis_run.leftdown.fpU += friction_compensation_current.leftdown + feed_forward_current.leftdown;
//			LESO_Order1(&leso_leftup,chassis_run.leftup.fpFB,chassis_run.leftup.fpU);
//			LESO_Order1(&leso_rightup,chassis_run.rightup.fpFB,chassis_run.rightup.fpU);
//			LESO_Order1(&leso_rightdown,chassis_run.rightdown.fpFB,chassis_run.rightdown.fpU);
//			LESO_Order1(&leso_leftdown,chassis_run.leftdown.fpFB,chassis_run.leftdown.fpU);
//			
//			compensation_leftup = leso_leftup.U - leso_leftup.U0;
//			compensation_rightup = leso_rightup.U - leso_rightup.U0;
//			compensation_rightdown = leso_rightdown.U - leso_rightdown.U0;
//			compensation_leftdown = leso_leftdown.U - leso_leftdown.U0;
//			
//			chassis_run.leftup.fpU += compensation_leftup + feed_forward_current.leftup;
//			chassis_run.rightup.fpU += compensation_rightup + feed_forward_current.rightup;
//			chassis_run.rightdown.fpU += compensation_rightdown + feed_forward_current.rightdown;;
//			chassis_run.leftdown.fpU += compensation_leftdown + feed_forward_current.leftdown;
		}
		CAN_SendCurrent(&hcan1,0x200,chassis_run.leftup.fpU,chassis_run.rightup.fpU,chassis_run.rightdown.fpU,chassis_run.leftdown.fpU);
		
		
//		Drib_Left_Smc.fpDes = Drib_Left_Targetv;
//		Drib_Right_Smc.fpDes = Drib_Right_Targetv;
//		Drib_Left_Smc.fpFB = Drib_Motor_Left.anglev;
//		Drib_Right_Smc.fpFB = Drib_Motor_Right.anglev;
//		CalSMC(&Drib_Left_Smc);
//		CalSMC(&Drib_Right_Smc);
		PID_Calc(&Drib_Left_Pid,Drib_Left_Targetv,Drib_Motor_Left.anglev);
		PID_Calc(&Drib_Right_Pid,Drib_Right_Targetv,Drib_Motor_Right.anglev);
		
		CAN_SendCurrent(&hcan2,0x200,Drib_Right_Pid.fpU,Drib_Left_Pid.fpU,0,0);
		CAN_SendCurrent(&hcan2,0x200,Drib_Right_Smc.fpU,Drib_Left_Smc.fpU,0,0);
		
		Lift_Left_td.aim = ClipFloat(Lift_target_pos,0,800);
		Lift_Right_td.aim = ClipFloat(-Lift_target_pos,-800,0);
		CalTD(&Lift_Left_td);
		CalTD(&Lift_Right_td);
		PID_CascadeCalc(&Lift_Left_Pid,Lift_Left_td.x1,Lift_Motor_Left.angle,Lift_Motor_Left.anglev);
		PID_CascadeCalc(&Lift_Right_Pid,Lift_Right_td.x1,Lift_Motor_Right.angle,Lift_Motor_Right.anglev);
		CAN_SendCurrent(&hcan1,0x1ff,Lift_Right_Pid.output,0,Lift_Left_Pid.output,0);
		
		
		J60_motor_control();
		ctrl_motor(&hcan2,0x81,&J60_Motor_Ctrl,8);
		if(fabs(J60_Motor_Data.torque_) > 25.f || J60_motor_temp > 85 || J60_board_temp >75)//电机持续扭矩过大或者过热时启动保护
		{
			J60_start_protect = 1;
		}
		else 
		{
			J60_start_protect = 0;
			J60_protect_cnt = 0;
		}
		if(J60_start_protect == 1)
		{
			J60_protect_cnt++;
		}
		if(J60_protect_cnt>100)//电机状态不正常持续100ms时，失能J60
		{
//			ctrl_motor(&hcan2,0x21,&J60_Motor_Ctrl,0);
			J60_Ctrl_Mode = NO_TORQUE_MODE;
		}
// 		system_monitor.can_send_cnt++;
		
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Motor_Control */
}

/* USER CODE BEGIN Header_Remote_Control */
/**
* @brief Function implementing the REMOTE_CONTROL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Control */
void Remote_Control(void *argument)
{
  /* USER CODE BEGIN Remote_Control */
	
  /* Infinite loop */
  for(;;)
  {
  	if(!NRF24L01_Check()&&!FLAG_NRF)
	  {
		  NRF24L01_RX_Mode();
		  FLAG_NRF = 1;
   	}
    if(FLAG_NRF==1)
		{
			nRF24L01_ack_pay.Ack_Channel = 0;
			nRF24L01_ack_pay.Ack_Status = 3;
			nRF24L01_ack_pay.Ack_Len = ACK_PLOAD_WIDTH;
			int rx_len = NRF24L01_RxPacket(nRF24L01_RxBuf);
			
			if(rx_len < 32) //持续断连则视为断连
				RC_Ctrl.normal_rc_disconnect_cnt++;
			else 
			{
				RC_Ctrl.normal_rc_disconnect_cnt = 0;
				RC_Ctrl.normal_rc_status = CONNECT;
			}
			if(RC_Ctrl.normal_rc_disconnect_cnt > 300)
			{
				RC_Ctrl.normal_rc_status = DISCONNECT;//已发生断连
				FLAG_NRF = 0;//自动尝试重连
			}
			NRF24L01_Rx_AckPayload(nRF24L01_ack_pay);
		 
			parseDataPacket(nRF24L01_RxBuf,&Js_Value);
			if(RC_Ctrl.rc_mode == NORMAL_RC_OPENED)
			{
				Read_Key_Task();
			}
			system_monitor.remote_control_cnt++;
		}
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Remote_Control */
}

/* USER CODE BEGIN Header_Fun_Task */
/**
* @brief Function implementing the FUN_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Fun_Task */
void Fun_Task(void *argument)
{
  /* USER CODE BEGIN Fun_Task */
  /* Infinite loop */
  for(;;)
  {
		// Fun_task();
		// system_monitor.fun_task_cnt++;    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Fun_Task */
}

/* USER CODE BEGIN Header_Communication_Task */
/**
* @brief Function implementing the COMMUNICATION thread.
* @param argument: Not used
* @retval None
*/
u8 uart1_tx[sizeof(uart1_eft)];
/* USER CODE END Header_Communication_Task */
void Communication_Task(void *argument)
{
  /* USER CODE BEGIN Communication_Task */
  /* Infinite loop */
  for(;;)
  {
// 		if(system_monitor.lwip_rec_fps <180)
// 			RC_Ctrl.vision_rc_status = DISCONNECT;
// 		else
// 			RC_Ctrl.vision_rc_status = CONNECT;
		
// 		RC_switch(&RC_Ctrl);
		
		
// 		static u8 radar_relocate;
// 		memcpy(&Vision_Data,UDP_rx_data,32);
// 		memcpy(&Vision_Data.nav_flag,&UDP_rx_data[32],1);
// 		memcpy(&Vision_Data.aim_chassis_yaw,&UDP_rx_data[33],40);
// 		vision_deal();
// 		radar_relocate++;
// 		if(radar_relocate > 4)
// 		{
// //			Vision_Relocation();
// 			radar_relocate = 0;
// 		}
// 		if(RC_Ctrl.rc_mode == VISION_RC_OPENED)
// 		{
// 			Vision_RC();
// 		}
		
//     uart1_eft.num[0] = Js_Value.indepen_usJsKey[0];
// 		uart1_eft.num[1] = Js_Value.indepen_usJsKey[1];
// 		uart1_eft.num[2] = Js_Value.usJsKey;
// 		memcpy(&uart1_eft.num[7],&Vision_Data.aim_gimbal_yaw,12);

// 		memcpy(uart1_tx,&uart1_eft,sizeof(uart1_eft));
// 		HAL_UART_Transmit(&huart1,uart1_tx,sizeof(uart1_eft),1);
		
// 		/************ң����*************/
// 		send_remote_control();
		
// //		/************����ͨ��***************/
// //		memcpy(UDP_tx_data,&dt35_now.dt35_x1,16);
// //		memcpy(&UDP_tx_data[16],&stRobot,12);
		
// //		/************VOFA+*************/
// 		G_vofa_watch();
// 		u8 vofa_tx[sizeof(vofa)];
// 		memcpy(vofa_tx,&vofa,sizeof(vofa));
// 		HAL_UART_Transmit_DMA(&huart6,vofa_tx,sizeof(vofa));
		
// 		system_monitor.communication_cnt++;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END Communication_Task */
}

/* USER CODE BEGIN Header_Gyro_Task */
/**
* @brief Function implementing the GYRO_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gyro_Task */
void Gyro_Task(void *argument)
{
  /* USER CODE BEGIN Gyro_Task */
  /* Infinite loop */
  for(;;)
  {
    if(g_uart_rx_cnt > 0)
		{
			memcpy(g_decode_data + g_decode_data_pos, g_uart_rx_buf, g_uart_rx_cnt);
			g_decode_data_pos += g_uart_rx_cnt;
			g_uart_rx_cnt = 0;
		}		
		if(g_decode_data_pos > 0)
		{
			analysis_data(g_decode_data, g_decode_data_pos);
			
			system_monitor.gyro_cnt++;
			
		}
		vTaskDelay(pdMS_TO_TICKS(2));
  }
  /* USER CODE END Gyro_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

