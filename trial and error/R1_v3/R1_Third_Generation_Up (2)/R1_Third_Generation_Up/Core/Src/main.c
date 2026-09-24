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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_Bsp.h"
#include "Logic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern osThreadId_t TrajectoryHandle;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim3);
	
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1,RxMsg_USART1,sizeof(RxMsg_USART1)/sizeof(RxMsg_USART1[0]));
    
	CAN_BSP_Init();
	
	YSC_Motor_Init(&BlockArm_Joint1_J60CMD,1);
	YSC_Motor_Init(&BlockArm_Joint2_J60CMD,2);
	YSC_Motor_Init(&BlockArm_Gimbal_J60CMD,3);
    DM_Init();
	
	BlockArm_Init();
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

    if (htim->Instance == TIM6)
    {
        if (PoleArm_Task_State!=POLEARM_TASK_MANUAL_CONTROL)
        {
            PoleArm_Task_Timer.PoleArm_Task_Ticks++;
        }

		Traj_tLast+=0.001f;
		Traj_BlockArm_Joint3.Data.Angle.t_last += 0.001f; // 每1ms更新一次时间
        Traj_Gimbal.Data.Angle.t_last += 0.001f; // 每1ms更新一次时间
		if(Traj_Arm.Type==TRAJ_TYPE_LINE)
		{
			Traj_Arm.Data.Line.t_last +=0.001f;
		}
		if(Traj_Arm.Type==TRAJ_TYPE_ARC)
		{
			Traj_Arm.Data.Arc.t_last +=0.001f;
		}
        if (TrajectoryHandle != NULL&&(uint16_t)(2000*Traj_tLast)%2==0) 
        {
			Traj_tLast=0;
            osThreadFlagsSet(TrajectoryHandle, 0x01);   // 发送标志位
        }

    }

    if (htim->Instance==TIM3)
    {

        System_Monitor.Fps_DJI_Send= System_Monitor.Cnt_DJI_Send;
        System_Monitor.Cnt_DJI_Send=0;
    

    
        System_Monitor.FPS_J60_Send=System_Monitor.Cnt_J60_Send;
        System_Monitor.Cnt_J60_Send=0;
    

        System_Monitor.FPS_AirBoard_Receive=System_Monitor.Cnt_AirBoard_Receive;
        System_Monitor.Cnt_AirBoard_Receive=0;
    
    
        System_Monitor.FPS_BlockArm_Joint1_J60_Receive=System_Monitor.Cnt_BlockArm_Joint1_J60_Receive;
        System_Monitor.Cnt_BlockArm_Joint1_J60_Receive=0;
    
        System_Monitor.FPS_BlockArm_Joint2_J60_Receive=System_Monitor.Cnt_BlockArm_Joint2_J60_Receive;
        System_Monitor.Cnt_BlockArm_Joint2_J60_Receive=0;
    
        System_Monitor.Fps_BlockArm_Joint3_3508_Receive=System_Monitor.Cnt_BlockArm_Joint3_3508_Receive;
        System_Monitor.Cnt_BlockArm_Joint3_3508_Receive=0;
		
		System_Monitor.FPS_BlockArm_Gimbal_J60_Receive=System_Monitor.Cnt_BlockArm_Gimbal_J60_Receive;
        System_Monitor.Cnt_BlockArm_Gimbal_J60_Receive=0;
    
        System_Monitor.Fps_Can_Receive[0]=System_Monitor.Cnt_Can_Receive[0];
        System_Monitor.Cnt_Can_Receive[0]=0;
    
        System_Monitor.Fps_Can_Receive[1]=System_Monitor.Cnt_Can_Receive[1];
        System_Monitor.Cnt_Can_Receive[1]=0;
    
        System_Monitor.FPS_Communicate_Receive=System_Monitor.Cnt_Communicate_Receive;
        System_Monitor.Cnt_Communicate_Receive=0;
    
        System_Monitor.Fps_PoleArm_FrictionWheel_3508_Receive=System_Monitor.   Cnt_PoleArm_FrictionWheel_3508_Receive;
        System_Monitor.Cnt_PoleArm_FrictionWheel_3508_Receive=0;
    
        System_Monitor.Fps_PoleArm_Joint1_3508_Receive=System_Monitor.Cnt_PoleArm_Joint1_3508_Receive;
        System_Monitor.Cnt_PoleArm_Joint1_3508_Receive=0;
    
        System_Monitor.Fps_PoleArm_Joint2_2006_Receive=System_Monitor.Cnt_PoleArm_Joint2_2006_Receive;
        System_Monitor.Cnt_PoleArm_Joint2_2006_Receive=0;
		
		System_Monitor.Fps_PoleArm_Adjustment_2006_Receive=System_Monitor.Cnt_PoleArm_Adjustment_2006_Receive;
        System_Monitor.Cnt_PoleArm_Adjustment_2006_Receive=0;
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
