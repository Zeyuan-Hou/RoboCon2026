#ifndef __GO1_SDK_H__
#define __GO1_SDK_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "global_declare.h"
#include "algorithm.h"

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

static uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);
static uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

void modify_data(MotorCmd_t *motor_s);
void extract_data(MotorData_t *motor_r);
void GO1_Tx(MotorCmd_t *cmd, UART_HandleTypeDef *huart, float pos, float vel, float tor, float kp, float kd);

#endif
