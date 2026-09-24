#ifndef ___V6_MOTOR_H___
#define ___V6_MOTOR_H___

#include "robot.h"

void V6_ReceiveData(V6_MOTOR *motor, uint8_t *data);
HAL_StatusTypeDef V6_SendData(FDCAN_HandleTypeDef *hcan, int16_t id, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4);













#endif // ___V6_MOTOR_H___






