/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global_declare.h"
#include "go1_sdk.h"
#include "board_communicate.h"
#include "vision.h"
#include "gyro.h"
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
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

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
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    TIM2_CNT++;                   // 1ms增加1
    TIM2_CNT_1 = TIM2_CNT / 1000; // 1s增加1

    tim_ms++;
    foot_motor_runtime++;
    location_filter.tim++;

		if(g_uart_rx_cnt > 0)
		{
			memcpy(g_decode_data + g_decode_data_pos, g_uart_rx_buf, g_uart_rx_cnt);
			g_decode_data_pos += g_uart_rx_cnt;
			g_uart_rx_cnt = 0;
		}		
		if(g_decode_data_pos > 0)
		{
			analysis_data(g_decode_data, g_decode_data_pos);			
		}
		
    if (monitor.time_base < 1000) // system_monitor时间基准
    {
      monitor.time_base++;
    }
    else
    {
      monitor.rate_fps = monitor.rate_cnt;
      memset(&monitor.rate_cnt, 0, sizeof(monitor.rate_cnt));
      monitor.time_base = 0;
    }

    // 实时计算小脚重力前馈
    if (!foot.up_tor_feedforward_flag)
    {
      foot_g_feedforward.leftup = 0.f;
      foot_g_feedforward.rightup = 0.f;
    }
    else
    {
      // 9.0515 = 1 / PI * 180 / 6.33
      // GO-M8010-6 减速比 6.33
      if (!foot_up_G_feedforward_flag)
      {
        foot_g_feedforward.leftup = feedforward_G_tor(FOOT_LEFTUP_TOR_MAX_1, (go1_recv_left.Pos - 21.33f - go1_left_0) * 9.0515f);     // +
        foot_g_feedforward.rightup = feedforward_G_tor(FOOT_RIGHTUP_TOR_MAX_1, (go1_recv_right.Pos + 21.33f - go1_right_0) * 9.0515f); // -
      }
      else
      {
        foot_g_feedforward.leftup = feedforward_G_tor(FOOT_LEFTUP_TOR_MAX_2, (go1_recv_left.Pos - 21.33f - go1_left_0) * 9.0515f);     // +
        foot_g_feedforward.rightup = feedforward_G_tor(FOOT_RIGHTUP_TOR_MAX_2, (go1_recv_right.Pos + 21.33f - go1_right_0) * 9.0515f); // -
      }
    }
    if (!foot.down_tor_feedforward_flag)
    {
      foot_g_feedforward.down = 0.f;
    }
    else
    {
      if (!foot_down_G_feedforward_flag)
      {
        foot_g_feedforward.down = feedforward_G_tor(FOOT_DOWN_TOR_MAX_1, (j60_motor_data_down.position_ - 1.866f) * 45.f); // +
      }
      else
      {
        foot_g_feedforward.down = feedforward_G_tor(FOOT_DOWN_TOR_MAX_2, (j60_motor_data_down.position_ - 1.866f) * 45.f); // +
      }
    }

    if (location_filter.tim >= 25){
      location_filter.tim = 0;
      location_filter.flag = 1;
    }

    if (nav.nav_state == NAV_LOCK)
    {
      nav.auto_path.run_time = 0;
      nav.auto_path.rotation_time = 0;
      point_tim = 0;
    }
    if (nav.nav_state == NAV_PERMUTATION_PATH) // 自动路径运行时间
    {
      nav.auto_path.run_time++;
      nav.auto_path.rotation_time++;
    }
    if (nav.nav_state == NAV_POINT_TO_POINT)
    {
      point_tim++;
    }
  }
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == 1)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_DMA_DISABLE(huart1.hdmarx);

    if (down_rx[0] == 0x33 && down_rx[17] == 0x55)
    {
      monitor.rate_cnt.UPPER_TO_LOWER++;
      Upper_Data_Recieve_Deal();
    }
    HAL_UART_Receive_DMA(&huart1, down_rx, 18);
    HAL_UART_DMAResume(&huart1);
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
//if  (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
//	{
//		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
//		__HAL_DMA_DISABLE(huart2.hdmarx);
//	  huart2.Instance->SR;  
//		huart2.Instance->DR; //clear ore flg
//		
//		
//		/***数据处理***/
//		//g_uart_rx_cnt = 512 - __HAL_DMA_GET_COUNTER(huart2.hdmarx); //get length of received data 
//		//g_uart_rx_buf[g_uart_rx_cnt]=0; //
//    parseDataPacket(UART2_Rx_Buf, &Js_Value);
//    Deal_Key_State(&Js_Value);
//    Send_To_RemoteControl();
//    HAL_UART_Transmit_DMA(&huart2, nRF24L01_ack_pay.Ack_Buf, 21);
//		
//		/***数据处理完成***/
//		monitor.rate_cnt.rc++;
//		
//		
//		
//		__HAL_DMA_CLEAR_FLAG(huart2.hdmarx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TEIF1_5);
//		__HAL_DMA_SET_COUNTER(huart2.hdmarx, 512); //reset length of rx dma

//		__HAL_DMA_ENABLE(huart2.hdmarx); 	

//	}
	  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) == 1)
  {
		monitor.rate_cnt.rc++;
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    __HAL_DMA_DISABLE(huart2.hdmarx);

   
     // monitor.rate_cnt.UPPER_TO_LOWER++;
      parseDataPacket(UART2_Rx_Buf, &Js_Value);//接收
	//		Deal_Key_State(&Js_Value);//按键赋值
			
    
    HAL_UART_Receive_DMA(&huart2, UART2_Rx_Buf, 9);
    HAL_UART_DMAResume(&huart2);
  }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

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
  if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) == 1)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    __HAL_DMA_DISABLE(huart5.hdmarx);

    if (vision_rec[0] == 0x66 && vision_rec[61] == 0x99)
    {
      monitor.rate_cnt.VISION_REC++;
      Vision_Data_Deal(&vision_data_recieve);
    }
    HAL_UART_Receive_DMA(&huart5, vision_rec, 62);
    HAL_UART_DMAResume(&huart5);
  }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
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

/* USER CODE BEGIN 1 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    monitor.rate_cnt.LOWER_TO_UPPER++;
  }

  if (huart->Instance == USART2)
  {
    monitor.rate_cnt.vofa++;
  }
	
	if (huart->Instance == UART4)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  }

  if (huart->Instance == UART5)
  {
    monitor.rate_cnt.VISION_TX++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == UART4)
	{
		switch (uart_rx_data.mode.id)
		{
			case 0x01:
				monitor.rate_cnt.go1_right++;
				memcpy(&go1_recv_right.motor_recv_data, &uart_rx_data, sizeof(RIS_MotorData_t));
				extract_data(&go1_recv_right);
				if (!go1_right_init_flag && go1_recv_right.correct)
				{
				  go1_right_0 = go1_recv_right.Pos;
				  go1_right_init_flag = 1;
				}	
				break;

			case 0x02:
				monitor.rate_cnt.go1_left++;
				memcpy(&go1_recv_left.motor_recv_data, &uart_rx_data, sizeof(RIS_MotorData_t));
				extract_data(&go1_recv_left);
				if (!go1_left_init_flag && go1_recv_left.correct)
				{
				  go1_left_0 = go1_recv_left.Pos;
				  go1_left_init_flag = 1;
				}
				break;

			default:
				break;
    }
    HAL_UART_Receive_DMA(&huart4, (uint8_t *)&uart_rx_data, sizeof(RIS_MotorData_t));
   }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4){
      __HAL_UART_CLEAR_OREFLAG(huart);
      __HAL_UART_CLEAR_NEFLAG(huart);
      __HAL_UART_CLEAR_FEFLAG(huart);

      HAL_UART_DMAStop(huart);

      HAL_UART_Receive_DMA(&huart4, (uint8_t *)&uart_rx_data, sizeof(RIS_MotorData_t));
    }
}
// 定时器计数写在main.c的定时器中断回调里了

/* USER CODE END 1 */
