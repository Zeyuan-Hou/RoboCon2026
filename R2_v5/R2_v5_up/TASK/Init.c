#include "Init.h"

void Init()
{
    CAN_INIT();
    HAL_TIM_Base_Start_IT(&htim3); 
    HAL_UART_Receive_DMA(&huart1, usartBufForA1, 78);
    HAL_UART_Receive_DMA(&huart4, uart4_rev, sizeof(uart4_rev));
    HAL_UART_Receive_DMA(&huart2, uart2_rev, sizeof(uart2_rev));

    __HAL_UART_DISABLE(&huart3);
    __HAL_UART_FLUSH_DRREGISTER(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)g_uart_rx_buf, 512);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    __HAL_UART_ENABLE(&huart3);

    J60_Enable(&Gimbal_J60, &Gimbal_J60_CMD);
    // dm_motor_enable(&hfdcan1, &stretch_DM);

    RobStride_Motor_Init(&RobStride_01, 0x01, 1);
    RobStride_Motor_Set_FDCAN(&hfdcan1);
    RobStride_Motor_MIT_SetMotorType(&RobStride_01, 0x00);
    RobStride_Motor_MIT_Enable(&RobStride_01);

    G_Feedforward_Init();
}
