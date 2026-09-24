#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "main.h"
#include "unitree_motor.h"

// 腿部索引
typedef enum {
    FL = 0, // Front Left
    FR = 1, // Front Right
    RL = 2, // Rear Left
    RR = 3  // Rear Right
} Leg_Index_e;

// 关节索引
typedef enum {
    HIP   = 0, // 侧摆关节
    THIGH = 1, // 大腿关节
    CALF  = 2  // 小腿关节
} Joint_Index_e;

typedef enum
{
    MODE_DISABLE=0, // 失能模式    mode=0
    MODE_DAMPING, // 阻尼模式    mode=10 W=0.0
    MODE_CONTORL // 正常控制模式  mode=10
}Robot_Mode_e;

typedef struct
{
    UART_HandleTypeDef *p_huart;
    GPIO_TypeDef *en_gpio_port;
    uint16_t en_gpio_pin;
    unitree_motor_t motor[3]; // 髋、大腿、小腿
    uint8_t send_buf[34];
    uint8_t recv_buf[78];
    // uint8_t recv_flag; 
}Leg_t;

typedef struct
{
    Leg_t leg[4];
    Robot_Mode_e currentMode;
}Robot_t;


void robot_init(void);
void set_robot_mode(Robot_Mode_e mode);
void update_command(void);

extern Robot_t robot;

#endif // ROBOT_H

