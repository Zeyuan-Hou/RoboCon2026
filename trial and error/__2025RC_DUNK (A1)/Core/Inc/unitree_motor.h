#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include "ROBOT.h"
#include "main.h"
#include "usart.h"
uint32_t crc32_core(uint32_t* ptr, uint32_t len);
void modify(A1_CTRL_DATA *motor_data, uint8_t *raw_data);
uint32_t extract(A1_RECEIVE_DATA *motor_data, uint8_t *raw_data);
void receive_motor_feedback(uint8_t *raw_data);
void motor_A1_cmd(A1_STRUCTRUE *ctrl_data);
#endif


