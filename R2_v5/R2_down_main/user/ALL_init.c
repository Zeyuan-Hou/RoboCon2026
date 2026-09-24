#include "ALL_INIT.h"

void all_init(void)
{
  CAN_INIT(); 
  HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim5);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)&uart_rx_data, sizeof(RIS_MotorData_t));
	
	
	HAL_UART_Receive_DMA(&huart6, vision_rec_test, sizeof(vision_rec_test));
	HAL_UART_Receive_DMA(&huart4, uart4_receive, sizeof(uart4_receive));

	HAL_UART_Receive_DMA(&huart2,remote_rec_uart,sizeof(remote_rec_uart));
	HAL_UART_Receive_DMA(&huart3,uart_ctrl_receive,sizeof(uart_ctrl_receive));
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   // 只需这一句启动 PWM 输出
	
	inner_send[1] = 20;
	
}

