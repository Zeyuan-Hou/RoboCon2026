/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "gyro.h"

#include "HIPNUC_gyro.h"
#include "old_gyro.h"
#include "Vision.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	
//	  if (__HAL_UART_GET_FLAG(&huart1,UART_FLAG_ORE))
//    {
//        __HAL_UART_CLEAR_OREFLAG(&huart1);
//    }
//	
//	
//    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)) 
//    {
//        __HAL_UART_CLEAR_FEFLAG(&huart1);
//        //DMA
//    }
	if (__HAL_UART_GET_FLAG(&huart1,UART_FLAG_ORE))//清除ORE错误标志位
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
		// 1. 停止DMA
        HAL_UART_DMAStop(&huart1);
        // 2. 重新启动DMA接收
        HAL_UART_Receive_DMA(&huart1, uart1_rx_buff, sizeof(uart1_rx_buff));
    }
	
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE))//清除FE错误标志位
    {
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        // 1. 停止DMA
        HAL_UART_DMAStop(&huart1);
        // 2. 重新启动DMA接收
        HAL_UART_Receive_DMA(&huart1, uart1_rx_buff, sizeof(uart1_rx_buff));
    }
		
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); 
        HAL_UART_DMAStop (&huart1);
        Usart1_Receive_length = 20 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
			
			
			
        memcpy(&AirCtrl_1,&uart1_rx_buff[0],1);//赋值
			  memcpy(&AirCtrl_2,&uart1_rx_buff[1],1);//赋值
//			  memcpy(&AirCtrl_1,&uart1_rx_buff[2],1);//赋值

			

        HAL_UART_Receive_DMA(&huart1,uart1_rx_buff,3);
        system_monitor.communicate_rx_cnt++;
    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
//if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
//	{
//		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
//		__HAL_DMA_DISABLE(huart2.hdmarx);
//	
//		huart2.Instance->DR; //clear ore flg
//		huart2.Instance->SR;  
//		g_Usart2_Rx_cnt = 512 - __HAL_DMA_GET_COUNTER(huart2.hdmarx); //get length of received data 
//		g_Usart2_Rx_buf[g_Usart2_Rx_cnt]=0; //
//		
//		
//		
//		
//		
//		Gory_Data_Get();
//		
//		gyro_data.roll = g_output_info.roll * 10;
//		gyro_data.pitch = g_output_info.pitch * 10;
//		if(FLAG_REGION3_manual==0)
//		{gyro_data.yaw = g_output_info.yaw * 10;}
//		else if(FLAG_REGION3_manual==1)
//		{	gyro_data.yaw = g_output_info.yaw * 10+1800;}





//		
//		__HAL_DMA_CLEAR_FLAG(huart2.hdmarx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TEIF1_5);
//		__HAL_DMA_SET_COUNTER(huart2.hdmarx, 17); //reset length of rx dma
//    
//		__HAL_DMA_ENABLE(huart2.hdmarx); 	
//		
//		

//		system_monitor.gyro_cnt++;
//	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		__HAL_DMA_DISABLE(huart2.hdmarx);
	
		huart2.Instance->DR; //clear ore flg
		huart2.Instance->SR;  
		g_uart_rx_cnt = 512 - __HAL_DMA_GET_COUNTER(huart2.hdmarx); //get length of received data 
		g_uart_rx_buf[g_uart_rx_cnt]=0; //

		__HAL_DMA_CLEAR_FLAG(huart2.hdmarx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TEIF1_5);
		__HAL_DMA_SET_COUNTER(huart2.hdmarx, 512); //reset length of rx dma

		__HAL_DMA_ENABLE(huart2.hdmarx); 	
	}
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
//	if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) == 1)
//  {
//    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
//    __HAL_DMA_DISABLE(huart5.hdmarx);

//    if (Radar_RxBuf[0] == 0x66 && Radar_RxBuf[33] == 0x99)
//    {
//     Vision_Data_Deal(&Vision_Data);
//			system_monitor.Vision_Receive_cnt++;
//    }
//    HAL_UART_Receive_DMA(&huart5, Radar_RxBuf,34);
//    HAL_UART_DMAResume(&huart5);
//  }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) 
	{
		

		system_monitor.communicate_tx_cnt++;
  }
}




void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    __HAL_UART_CLEAR_OREFLAG(huart);//��������־λ
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);

    if(huart->Instance == UART4)
    {
//			  memcpy(ACK, rxdata, 5);
//		    ACK[3] = nrf_fps & 0xFF;//��λ
//	      ACK[4] = (nrf_fps >> 8) & 0xFF;//��λ
//        uart_cnt++;
			 system_monitor.remote_control_cnt++;
       Uart4_Receive_length = 20 - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, nRF24L01_RxBuf, 9);
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
    }
		
//		if (huart->Instance == USART2)
//    {
//        // 处理收到的数据
////        handle_usart_rx_idle(huart, Size);//超核电子
//			
//			
//			UART_Parse_Frame(g_Usart2_Rx_buf, 17);
//			
//			gyro_data.yaw=final_data.yaw*10;
//			
//			if(nav.nav_state==NAV_AREA_3_SINGLE)//在某些时候需要在最开始的时候就认为自己的角度是90度
//				{
//        gyro_data.yaw = fmodf(final_data.yaw * 10.0f + 900.0f + 1800.0f, 3600.0f) - 1800.0f;
//			}
//				
//			
//			gyro_data.pitch=final_data.pitch*10;
//			gyro_data.roll=final_data.roll*10;
//			
//				  // 开启 USART2 空闲中断 DMA 接收
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)g_Usart2_Rx_buf, 17);
//		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
//    }
		
		
		if(huart->Instance == UART5)
    { 
			
			
			if (Radar_RxBuf[0] == 0x66 && Radar_RxBuf[37] == 0x99)
    {
			Vision_Data_Deal(&Vision_Data);
			system_monitor.Vision_Receive_cnt++;
		}
					HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Radar_RxBuf, 38); 
      __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
		}
		
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, nRF24L01_RxBuf, 9);//encoder
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
    }
		
		
				if(huart->Instance == UART5)
    { 			
        		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Radar_RxBuf, 38); 
      __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
		}
		
		
		
		
		
		
		if(huart->Instance == USART1){
		HAL_UART_Receive_DMA(&huart1, uart1_rx_buff,3);
		}
		
		
				if(huart->Instance == USART2){
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)g_Usart2_Rx_buf, 17);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		
				}
		
		
}
/* USER CODE END 1 */
