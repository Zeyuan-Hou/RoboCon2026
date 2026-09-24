#include "robot.h"
#include "usart.h"
#include "delay.h"
#include "system_monitor.h"

Robot_t robot={0};



void robot_init(void)
{
    robot.leg[FL].en_gpio_port = RS485_EN1_GPIO_Port;
    robot.leg[FL].en_gpio_pin = RS485_EN1_Pin;
    robot.leg[FL].p_huart = &huart4;

    robot.leg[FR].en_gpio_port = RS485_EN2_GPIO_Port;
    robot.leg[FR].en_gpio_pin = RS485_EN2_Pin;
    robot.leg[FR].p_huart = &huart1;

    robot.leg[RL].en_gpio_port = RS485_EN3_GPIO_Port;
    robot.leg[RL].en_gpio_pin = RS485_EN3_Pin;
    robot.leg[RL].p_huart = &huart6;

    robot.leg[RR].en_gpio_port = RS485_EN4_GPIO_Port;
    robot.leg[RR].en_gpio_pin = RS485_EN4_Pin;
    robot.leg[RR].p_huart = &huart3;

    for(int i = 0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            robot.leg[i].motor[j].command.ID = j;
            robot.leg[i].motor[j].command.mode = DISABLE_MODE;
        }
        HAL_UART_Receive_DMA(robot.leg[i].p_huart, robot.leg[i].recv_buf, 78);
    }
}


void disable_all_motor(void)
{
    for(int i = 0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            robot.leg[i].motor[j].command.mode = DISABLE_MODE;
        }
    }
}

void set_all_motor_damping(void)
{
    for(int i = 0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            robot.leg[i].motor[j].command.mode = ENABLE_MODE;
            robot.leg[i].motor[j].command.K_P=0.0;
            robot.leg[i].motor[j].command.K_W=7.0;
            robot.leg[i].motor[j].command.Pos=0.0;
            robot.leg[i].motor[j].command.T=0.0;
            robot.leg[i].motor[j].command.W=0.0;
        }
    }
}

void enable_all_motor(void)
{
    for(int i = 0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            robot.leg[i].motor[j].command.mode = ENABLE_MODE;
        }
    }    
}


void set_robot_mode(Robot_Mode_e mode)
{
    switch (mode)
    {
    case MODE_DISABLE:
				robot.currentMode=MODE_DISABLE;
        disable_all_motor();
        break;
    case MODE_DAMPING:
				robot.currentMode=MODE_DAMPING;
        set_all_motor_damping();
        break;
    case MODE_CONTORL:
				robot.currentMode=MODE_CONTORL;
        enable_all_motor();
        break;
    default:
        break;
    }
}

void send_motor_command(Leg_t *leg,uint8_t joint_idx)
{
    modify(&(leg->motor[joint_idx].command), leg->send_buf);
    HAL_GPIO_WritePin(leg->en_gpio_port, leg->en_gpio_pin, GPIO_PIN_SET);
    HAL_UART_Transmit_DMA(leg->p_huart, leg->send_buf,34);
}

void receive_motor_feedback(Leg_t *leg)
{
    motor_receive_data_t temp_motor_data;
    extract(&temp_motor_data,leg->recv_buf);
    uint8_t idx = temp_motor_data.ID;
    leg->motor[idx].feedback = temp_motor_data;
		leg->motor[idx].temp_rate++;
}

void update_command(void)
{
		for(int j=0;j<3;j++)
		{
			for(int i=0;i<4;i++)
			{
					send_motor_command(&robot.leg[i],j);
			}
			delay_us(500);
		}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	for(int i=0;i<4;i++){
        if(huart==robot.leg[i].p_huart)
        {
            HAL_GPIO_WritePin(robot.leg[i].en_gpio_port, robot.leg[i].en_gpio_pin, GPIO_PIN_RESET);            
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	for(int i=0;i<4;i++){
        if (huart == robot.leg[i].p_huart)
        {
            receive_motor_feedback(&robot.leg[i]);
            HAL_UART_Receive_DMA(huart, robot.leg[i].recv_buf, 78);
						system_monitor.temp_rate[i]++;
				}
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for(int i=0; i<4; i++) {
        if (huart == robot.leg[i].p_huart) {
            // 1. 清除错误标志位
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            
            // 2. 强制停止当前的 DMA 传输
            HAL_UART_DMAStop(huart);
            
            // 3. 重新开启接收
            // 注意：这里需要根据你的实际协议重新启动接收
            HAL_UART_Receive_DMA(huart, robot.leg[i].recv_buf, 78); 
            
//            // 4. (可选) 记录错误次数，方便调试
//            system_monitor.uart_error_count[i]++; 
        }
    }
}

