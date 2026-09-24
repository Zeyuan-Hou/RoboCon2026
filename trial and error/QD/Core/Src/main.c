/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "ADS1256.h"
#include "Valve.h"
#include "PositionSwitch.h"
#include "task.h"
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t RESET_COUNT = 0;//ADS1256_RESET
uint8_t ADS1256_INIT_FLAG = 0;//ADS1256_INIT
uint8_t CHANNEL_SWITCH =0;//

//ADS1256
long double CESHI0;
long double CESHI1;
long double CESHI2;
long double CESHI3;
long double CESHI4;
long double CESHI5;
long double CESHI6;
long double CESHI7;
uint16_t  ket0,ket1,ket2,ket3,ket4,ket5,ket6,ket7;

uint16_t ucLedCnt = 0;

uint16_t TempValveStat = 0;

uint8_t   ucKeyScanFlag = 0;


uint8_t   aucKeyScanCnt[12] = {0};
uint8_t   aucDisShakeCnt[12] = {0};
uint8_t   aucDisShakeFlag[12] = {0};
uint8_t   aucKeyStatTemp[12]={0};
uint16_t  usKeyStat = 0;

uint8_t ucKeyNum = 0;
uint8_t DT35_TxData[16] = {0};
uint8_t KEY_TxData[1] = {0};
uint8_t Valve_RxData[2] = {0};
//DT35 ID 0x21
int16_t DTID = 0x210;
int16_t XCID = 0x20;
int16_t VEID = 0x211;


uint16_t System_cnt = 0;
uint16_t CAN_cnt = 0;
uint16_t CAN_Fps = 0;
uint16_t VAL_COM_cnt = 0;
uint16_t VAL_COM_Fps = 0;
uint16_t DT35_cnt = 0;
uint16_t DT35_Fps = 0;
uint16_t VAL_CON_cnt = 0;
uint16_t VAL_CON_Fps = 0;


uint8_t key_states[8]={0};
//uint8_t Can_rxBuffer1[8] = {0};
//uint8_t RxData[8] = {0};

volatile uint8_t bCANDataTask = 0;
volatile uint8_t bGetDistanceTask = 0; 
volatile uint8_t bValveControlTask = 0;

uint16_t usCANDataCounter = 0;
uint16_t usGetDistanceCounter = 0;
uint16_t usValveControlCounter = 0;


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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
//	HAL_FDCAN_Start(&hfdcan1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);	
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	CAN_INIT();
	ADS1256_Init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		    if (bValveControlTask)
    {
        bValveControlTask = 0;
        VALVE_CONTROL();
	      VAL_CON_cnt++;
    }
    if (bCANDataTask)
    {
        bCANDataTask = 0;
        CAN_DATA();
			  CAN_cnt ++;				
    }
    
    if (bGetDistanceTask)//
    {
        bGetDistanceTask = 0;
        GET_DISTANCE();
        DT35_cnt ++;			
    }
    
//		HAL_Delay(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
    if (htim->Instance == TIM2)
    {

        usCANDataCounter++;
        if (usCANDataCounter >= 1)  // 1ms
        {
            bCANDataTask = 1;
            usCANDataCounter = 0;
        }
        
       
    }
		
		if (htim->Instance == TIM3)
		{
			System_cnt++;
			if(System_cnt>=1000)
			{
					CAN_Fps=CAN_cnt;
					CAN_cnt = 0;
					DT35_Fps=DT35_cnt;
					DT35_cnt = 0;
					VAL_CON_Fps = VAL_CON_cnt;
					VAL_CON_cnt=0;
				  VAL_COM_Fps = VAL_COM_cnt;
          VAL_COM_cnt =0;				
					System_cnt = 0;
			}
		}
		
		if (htim->Instance == TIM4){
			usValveControlCounter++;
        if (usValveControlCounter >= 1)
        {
            bValveControlTask = 1;
            usValveControlCounter = 0;
        }
		}
		if (htim->Instance == TIM5){
			 usGetDistanceCounter++;
        if (usGetDistanceCounter >= 1)  // 1ms
        {
            bGetDistanceTask = 1;
            usGetDistanceCounter = 0;
        }
		}
}
/* USER CODE END 4 */

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
