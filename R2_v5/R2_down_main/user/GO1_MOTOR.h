#ifndef _GO1_MOTOR_H_
#define _GO1_MOTOR_H_

#include "Type.h"
#include <string.h>
#include "usart.h"
#include <math.h>

#define SATURATE(_IN, _MIN, _MAX) \
	{                             \
		if ((_IN) <= (_MIN))      \
			(_IN) = (_MIN);       \
		else if ((_IN) >= (_MAX)) \
			(_IN) = (_MAX);       \
	}
	
uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);
void modify_data(MotorCmd_t *motor_s);
void extract_data(MotorData_t *motor_r);
void motor_go1_cmd(MotorCmd_t *ctrl_data,ST_TD *td);
void receive_motor_feedback(RIS_MotorData_t *receive_data);
int Sgn(float x);
void CalTD(ST_TD *pStTD);
#endif

