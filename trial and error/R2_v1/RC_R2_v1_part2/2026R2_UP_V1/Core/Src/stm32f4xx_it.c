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
#include "Robot.h"
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
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
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
	system_monitor.AO_fps=system_monitor.AO_cnt;
	system_monitor.AO_cnt=0;
	system_monitor.leftShoulder_fps=system_monitor.leftShoulder_cnt;
	system_monitor.leftShoulder_cnt=0;
	system_monitor.left_2006_1_fps=system_monitor.left_2006_1_cnt;
	system_monitor.left_2006_1_cnt=0;
	system_monitor.left_2006_2_fps=system_monitor.left_2006_2_cnt;
	system_monitor.left_2006_2_cnt=0;
	system_monitor.rightShoulder_fps=system_monitor.rightShoulder_cnt;
	system_monitor.rightShoulder_cnt=0;
	system_monitor.right_2006_1_fps=system_monitor.right_2006_1_cnt;
	system_monitor.right_2006_1_cnt=0;
	system_monitor.right_2006_2_fps=system_monitor.right_2006_2_cnt;
	system_monitor.right_2006_2_cnt=0;
	system_monitor.stretch_2006_fps=system_monitor.stretch_2006_cnt;
	system_monitor.stretch_2006_cnt=0;
	system_monitor.stretch_DM_fps=system_monitor.stretch_DM_cnt;
	system_monitor.stretch_DM_cnt=0;
	system_monitor.stretch_LK_fps=system_monitor.stretch_LK_cnt;
	system_monitor.stretch_LK_cnt=0;
	system_monitor.withinBoardCommunicate_fps=system_monitor.withinBoardCommunicate_cnt;
	system_monitor.withinBoardCommunicate_cnt=0;
	system_monitor.WBCT_fps=system_monitor.WBCT_cnt;
	system_monitor.WBCT_cnt=0;
	system_monitor.SCMT_fps=system_monitor.SCMT_cnt;
	system_monitor.SCMT_cnt=0;
	system_monitor.CT_fps=system_monitor.CT_cnt;
	system_monitor.CT_cnt=0;
	system_monitor.MSMT_fps=system_monitor.MSMT_cnt;
	system_monitor.MSMT_cnt=0;
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	 if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == 1)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);  
        __HAL_DMA_DISABLE(huart1.hdmarx);
    
        huart1.Instance->DR; //clear ore flg
        huart1.Instance->SR;  
				if(RxBufFromZGT[0]==0x88&&RxBufFromZGT[16]==0x66){
					if(RxBufFromZGT[1]) action_DA=0;
					if(RxBufFromZGT[2]) action_DA=1;
					if(RxBufFromZGT[3]){
						if(feedback_DA==20||feedback_DA==3||feedback_DA==30){
							action_DA=3;
						}else{
							action_DA=2;
						}
					}
					if(RxBufFromZGT[4]) action_DA=3;
					if(RxBufFromZGT[5]) action_DA=4;
					
					if(RxBufFromZGT[6]) action_SA=1;
					if(RxBufFromZGT[7]){
						action_SA=2;
						height_flag=1;
					}
					if(RxBufFromZGT[8]){
						action_SA=2;
						height_flag=0;
					}
					if(RxBufFromZGT[9]) action_SA=8;
					if(RxBufFromZGT[10]) action_SA=3;
					if(RxBufFromZGT[11]) action_SA=7;
					if(RxBufFromZGT[12]) action_SA=4;
					if(RxBufFromZGT[13]) action_SA=6;
					if(RxBufFromZGT[14]) action_SA=5;//放中层方块
					if(RxBufFromZGT[15]) action_SA=11;//放上层方块
//					if(action_DA==2&&feedback_DA==20){
//						action_DA =3;
//					}
					system_monitor.withinBoardCommunicate_cnt++;
				}
        HAL_UART_Receive_DMA(&huart1,RxBufFromZGT,sizeof(RxBufFromZGT));
        HAL_UART_DMAResume(&huart1);
   }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
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
        Tx_Completed_Flag = 1;  // ��DMA������ϣ���������ɱ�־λ��1
    }
}
/* USER CODE END 1 */
