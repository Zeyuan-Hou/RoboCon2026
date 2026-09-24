/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern uint8_t RESET_COUNT;//ADS1256
extern uint8_t ADS1256_INIT_FLAG;//ADS1256
extern uint8_t CHANNEL_SWITCH;
	
extern long double CESHI0;
extern long double CESHI1;
extern long double CESHI2;
extern long double CESHI3;
extern long double CESHI4;
extern long double CESHI5;
extern long double CESHI6;
extern long double CESHI7;
extern uint16_t  ket0,ket1,ket2,ket3,ket4,ket5,ket6,ket7;//
extern uint16_t ucLedCnt;

extern SPI_HandleTypeDef hspi1;

//
extern uint16_t TempValveStat;//

extern uint8_t   ucKeyScanFlag;   //

//
extern uint8_t   aucKeyScanCnt[12];		  //
extern uint8_t   aucDisShakeCnt[12];		  //
extern uint8_t   aucDisShakeFlag[12];		  //
extern uint8_t   aucKeyStatTemp[12];		    //
extern uint16_t  usKeyStat;				        //

extern uint8_t ucKeyNum;
//

extern uint8_t DT35_TxData[16];
extern uint8_t KEY_TxData[1];
extern uint8_t Valve_RxData[2];


extern uint16_t System_cnt;
extern uint16_t CAN_cnt;
extern uint16_t CAN_Fps;
extern uint16_t VAL_COM_cnt;
extern uint16_t VAL_COM_Fps;
extern uint16_t DT35_cnt;
extern uint16_t DT35_Fps;
extern uint16_t VAL_CON_cnt;
extern uint16_t VAL_CON_Fps;

extern uint8_t key_states[8];
//extern uint8_t Can_rxBuffer1[8];

//DT35 ID 0x21
//extern short DTID=0x21;
//extern short XCID = 0x20;
//extern short VEID = 0x30;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ADS1256_DRDY_Pin GPIO_PIN_1
#define ADS1256_DRDY_GPIO_Port GPIOC
#define ADS1256_RESET_Pin GPIO_PIN_2
#define ADS1256_RESET_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define KEYSCAN_TIME    1 		   			//???????????????APPSCAN_TIME ms 
#define DISSHAKE_TIME   5		   	    //????????????DISSHAKE_TIME ms
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
