#ifndef __UNITREE_MOTOR_H__
#define __UNITREE_MOTOR_H__

#include "main.h"
#include "usart.h"

#define PI 3.1415926536f

typedef enum
{
    ENABLE_MODE = 10,
    DISABLE_MODE = 0,
} motor_mode_e;

typedef struct
{
    uint8_t ID; 
    uint8_t mode;
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
} motor_send_data_t;

typedef struct
{
    uint8_t ID;
    uint8_t mode;
    float T;
    float Temp;
    float Error;
    float W;
    float Pos;
    float Acc;
    float K_P;
    float K_W;
    float gyro[3];
    float acc[3];
} motor_receive_data_t;

typedef struct
{
    motor_send_data_t command;
    motor_receive_data_t feedback;
    uint32_t temp_rate, real_rate;
} unitree_motor_t;

extern uint32_t extract(motor_receive_data_t *motor_data, uint8_t *raw_data);
extern void modify(motor_send_data_t *motor_data, uint8_t *raw_data);
extern uint32_t crc32_core(volatile uint8_t *src, uint32_t len);

#endif // __UNITREE_MOTOR_H__
