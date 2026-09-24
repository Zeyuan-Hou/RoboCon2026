/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Can_Bsp.h"
//#include "gyro.h"
#include "HIPNUC_gyro.h"
#include <string.h>
#include "remote_control_task.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */


//CAN初�?�化
  can1_start();
  can2_start();














//USART6初�?�化（向上位机VOFA发送数�?的串口）
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);










//�?动TIM3定时器用于System_Monitor计算
  HAL_TIM_Base_Start_IT(&htim3);










//USART2初�?�化（接收陀螺仪数据�?

//  app_init();
	 
	 
	  // 开启 USART2 空闲中断 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)g_Usart2_Rx_buf, 17);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		
//	__HAL_UART_DISABLE(&huart2);
//  __HAL_UART_FLUSH_DRREGISTER(&huart2);        
//  __HAL_UART_CLEAR_OREFLAG(&huart2);
//  HAL_UART_Receive_DMA(&huart2, (uint8_t *)g_Usart2_Rx_buf, 17);
//  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//  __HAL_UART_ENABLE(&huart2);







//USART1初�?�化（上下板通信�?
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, uart1_rx_buff,3);


		
uart1_tx_buffer[0]=0x11;
uart1_tx_buffer[25]=0x22;

////上下板通信部分
//		HAL_UART_Transmit_DMA(&huart1, uart1_tx_buffer, 26);




//USART4初�?�化（接收遥控模块数�?�?
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, nRF24L01_RxBuf, 9); 
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
	
	
	//UART5初�?�化（接收雷达数据）
//	   HAL_UART_Receive_DMA(&huart5, Radar_RxBuf, 34);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Radar_RxBuf, 38); 
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	
	
	
	
	
	
	
	
	
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
if (htim->Instance == TIM3) 
{   
	
	
	
    if(nav.nav_state==NAV_AREA_1||nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_AREA_3||nav.nav_state==NAV_AREA_3_RESET||nav.nav_state==NAV_AREA_3_SINGLE)
			{
				nav.auto_path.run_time++;

        //����������ʱû�þ���ע�͵���
				//nav.auto_path.rotation_time++;
				//nav.auto_path.run_Sumtime++;
			}	
	
	
	 location_filter.tim++;
	  nRF24L01_Tick++;
		gyro_tick++;
	  tim_sum_monitor++;
	
		if(tim_sum_monitor==1000)
		{
			system_monitor.flw_fps=system_monitor.flw_cnt;
			system_monitor.flw_cnt=0;
			
			system_monitor.can_send_fps_chassis=system_monitor.can_send_cnt_chassis;
			system_monitor.can_send_cnt_chassis=0; //底层航模电机CAN发送
			
			system_monitor.can_send_fps_3508=system_monitor.can_send_cnt_3508;
			system_monitor.	can_send_cnt_3508=0;   //3508的发送
			
			 for(int i=0;i<2;i++)
			{
				system_monitor.can_rec_fps[i]=system_monitor.can_rec_cnt[i];
				system_monitor.can_rec_cnt[i]=0;     //CAN1 CAN2接收
			}
			
			system_monitor.motor_LU_fps=system_monitor.motor_LU_cnt;
			system_monitor.motor_LU_cnt=0;
			
			system_monitor.motor_RU_fps=system_monitor.motor_RU_cnt;
			system_monitor.motor_RU_cnt=0;
			
			system_monitor.motor_RD_fps=system_monitor.motor_RD_cnt;
			system_monitor.motor_RD_cnt=0;
			
			system_monitor.motor_LD_fps=system_monitor.motor_LD_cnt;
			system_monitor.motor_LD_cnt=0;
			
		
			system_monitor.remote_control_fps=system_monitor.remote_control_cnt;
			system_monitor.remote_control_cnt=0;//接收遥控器数�?
			

			
			system_monitor.gyro_fps=system_monitor.gyro_cnt;
			system_monitor.gyro_cnt=0;//陀螺仪接收
			
			system_monitor.location_fps=system_monitor.location_cnt;
			system_monitor.location_cnt=0;//定位任务运�??
			
			
			system_monitor.navigation_fps=system_monitor.navigation_cnt;
			system_monitor.navigation_cnt=0;//导航任务运�??
			
			system_monitor.VOFA_fps=system_monitor.VOFA_cnt;
			system_monitor.VOFA_cnt=0;//VOFA发�?
			
			 for(int i=0;i<2;i++)
			{
      system_monitor.Crane_3508_fps[i]=system_monitor.Crane_3508_cnt[i];
				system_monitor.Crane_3508_cnt[i]=0;//起重机两�?3508
			}
			
			
      system_monitor.communicate_tx_fps=system_monitor.communicate_tx_cnt;
      system_monitor.communicate_tx_cnt=0;//向上板发�?
		
			system_monitor.communicate_rx_fps=system_monitor.communicate_rx_cnt;
			system_monitor.communicate_rx_cnt=0;//从上板接�?
			
			system_monitor.AirBoard_Receive_fps=system_monitor.AirBoard_Receive_cnt;
			system_monitor.AirBoard_Receive_cnt=0;
			
			system_monitor.Vision_Receive_fps=system_monitor.Vision_Receive_cnt;
			system_monitor.Vision_Receive_cnt=0;
			

			

			tim_sum_monitor=0;
			
			
			
			
	  }
		 if (location_filter.tim >= 25){
      location_filter.tim = 0;
      location_filter.flag = 1;}
	
}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
