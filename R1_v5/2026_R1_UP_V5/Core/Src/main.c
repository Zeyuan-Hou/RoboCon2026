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
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_Bsp.h"
#include "USART_Bsp.h"
#include "J60_motor.h"
#include "tjc_usart_hmi.h"
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
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
u8 zhu=0;
u8 board_rec[20]={0},remote_rec[11]={0},test_rec[9]={0};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
			size_t size = 24;
			for(uint8_t i = 0;i<size;i++){
				if(usart1_rxbuf[i] == 0x66&&usart1_rxbuf[(i+size-1)%size] == 0x99){	
					memcpy(&vision_rec,&usart1_rxbuf[i],size - i);
					memcpy(&vision_rec[size-i],&usart1_rxbuf,i);
					
					break;
				}
			}
      unpackVisionData(vision_rec);
			HAL_UART_Receive_DMA(&huart1, usart1_rxbuf, 24);
			systemMonitor.cntMonitor.visionCommunicate++;
    }
		else if(huart->Instance == USART2){
			size_t size = 9;
			for(uint8_t i = 0;i<size;i++){
				if(IRModuleRxBuffer[i] == 0xAA&&IRModuleRxBuffer[(i+size-1)%size] == 0xAA){	
					memcpy(&test_rec,&IRModuleRxBuffer[i],size - i);
					memcpy(&test_rec[size-i],&IRModuleRxBuffer,i);
					break;
				}
			}
			HAL_UART_Receive_DMA(&huart2,IRModuleRxBuffer,9);
		}
		else if(huart->Instance == USART3){
//			size_t size = 13;
//			for(uint8_t i = 0;i<size;i++){
//				if(usart2_rxbuf[i] == 0xAA&&usart2_rxbuf[(i+size-1)%size] == 0xAA){	
//					memcpy(&remote_rec,&usart2_rxbuf[i],size - i);
//					memcpy(&remote_rec[size-i],&usart2_rxbuf,i);
//					
//					break;
//				}
//			}
//			parseDataPacket(remote_rec+2,&remoteRec);
//			CalculateVelocities(&remoteRec,&remote_vel,800,1800,800,1800,1600,200);
//			DealKeyOfficial(remoteRec.usJsKey);
//				HAL_UART_Receive_IT(&huart3, usart2_rxbuf, 13);
//			if(usart2_rxbuf[0]==0xAA && usart2_rxbuf[1]==0x55 && usart2_rxbuf[11]==0x55 && usart2_rxbuf[12]==0xAA)
//			{
//				parseDataPacket(usart2_rxbuf+2,&remoteRec);
//				CalculateVelocities(&remoteRec,&remote_vel,800,1800,800,1800,1600,200);
//				DealKey(remoteRec.usJsKey);
//				
//				return;
//			}
//			HAL_UART_Receive_IT(&huart3, usart2_rxbuf, 13);
			size_t size = 11;
			for(uint8_t i = 0;i<size;i++){
				if(usart2_rxbuf[i] == 0xAA&&usart2_rxbuf[(i+size-1)%size] == 0xBB){	
					memcpy(&remote_rec,&usart2_rxbuf[i],size - i);
					memcpy(&remote_rec[size-i],&usart2_rxbuf,i);
					break;
				}
			}
			parseDataPacket(remote_rec+1,&remoteRec);
			CalculateVelocities(&remoteRec,&remote_vel,500,1750,500,1750,500,100);
			DealKeyTemp(remoteRec.usJsKey);
			HAL_UART_Receive_DMA(&huart3, usart2_rxbuf, 11);
			systemMonitor.cntMonitor.remoteControl++;
		}
		else if(huart->Instance == UART4)
		{
			size_t size = 18;
			for(uint8_t i = 0;i<size;i++){
				if(uart4_rxbuf[i] == 0x0b&&uart4_rxbuf[(i+size-1)%size] == 0x0a){	
					memcpy(&board_rec,&uart4_rxbuf[i],size - i);
					memcpy(&board_rec[size-i],&uart4_rxbuf,i);
					
					break;
				}
			}
			unpackDataFromLower(board_rec);
			HAL_UART_Receive_DMA(&huart4, uart4_rxbuf, 18);
			systemMonitor.cntMonitor.withinBoardCommunicate++;
		}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {

        HAL_UART_DMAStop(huart);

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        HAL_UART_Receive_DMA(&huart1, usart1_rxbuf, 24);
    }else if (huart->Instance == USART3) {

        HAL_UART_DMAStop(huart);

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        	HAL_UART_Receive_DMA(&huart3, usart2_rxbuf, 11);
    }
		else if (huart->Instance == UART4) {

        HAL_UART_DMAStop(huart);

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        	HAL_UART_Receive_DMA(&huart4, uart4_rxbuf, 20);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_DMA(&huart4,uart4_rxbuf,18);
	HAL_UART_Receive_DMA(&huart1,usart1_rxbuf,24);
	HAL_UART_Receive_DMA(&huart2,IRModuleRxBuffer,9);
	HAL_UART_Receive_DMA(&huart3, usart2_rxbuf, 11);
	HAL_TIM_Base_Start_IT(&htim7);
	initRingBuffer();                //
//	HAL_UART_Receive_IT(&huart2,&zhu,1);
	
	
	CAN_INIT();
	Deep_Init(&gimbalJ60,&gimbalJ60CMD,&hfdcan1);
	Deep_Init(&shoulderJ60,&shoulderJ60CMD,&hfdcan2);
	Deep_Init(&elbowJ60,&elbowJ60CMD,&hfdcan2);
	
	HAL_Delay(700);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	// �ж��Ƿ���TIM7�������ж�
  if (htim->Instance == TIM7)
  {
			SINGLE_MONITOR temp={0};
    memcpy(&systemMonitor.fpsMonitor,&systemMonitor.cntMonitor,sizeof(SINGLE_MONITOR));
		memcpy(&systemMonitor.cntMonitor,&temp,sizeof(SINGLE_MONITOR));
		const u16* ptr = (u16*) &systemMonitor.fpsMonitor;
		for(u8 i =0;i<19;i++){
			systemMonitor.error[i]=(ptr[i]>100);
		}
  }
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

#ifdef  USE_FULL_ASSERT
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
