#ifndef __J60_SDK_H__
#define __J60_SDK_H__

#include "global_declare.h"
#include "can.h"
#include "can_bsp.h"
#include <stdint.h>
#include <math.h>

void J60_SetNormalCMD(J60_MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd);
void J60_SetMotionCMD(J60_MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd, float position, float velocity, float torque, float kp, float kd);

void J60_Unpack_Feedback(const uint8_t can_rx[8], J60_MotorDATA *data, uint32_t StdId);
uint16_t J60_FormCanId(uint8_t cmd, uint8_t motor_id);

void J60_MOTOR_ENABLE(CAN_HandleTypeDef *hcan, J60_MotorCMD *data);
void J60_MOTOR_DISABLE(CAN_HandleTypeDef *hcan, J60_MotorCMD *data);
void J60_LOOP_CONTROL(CAN_HandleTypeDef *hcan, J60_MotorCMD *data, float position, float velocity, float torque, float kp, float kd, J60_MotorDATA *feedback);

float J60_feedforward_G_torque(float torque_k1, float pos_j60, float torque_k2, float pos_dm);

#endif
