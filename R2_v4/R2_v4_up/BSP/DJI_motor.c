#include "DJI_motor.h"
#include "MathAlgorithm.h"
#include "ROBOT.h"

uint8_t DJIMotorStart(void)
{
	// set pid params
	// PID_Init(&move_2006.motor_pid.outer, 6.f, 0.015f, 0.1f, 0.f, 200.f, 1000.f, 800.f, 800.f, 400.f);
	// PID_Init(&move_2006.motor_pid.inner, 250, 0, 28, 0, 800, 1000, 8000, 8000, 4000);
	PID_Init(&DJI_3508.motor_pid.outer, 8.f, 0.015f, 1.f, 0.f, 100.f, 800.f, 400.f, 800.f, 100.f);
	PID_Init(&DJI_3508.motor_pid.inner, 140, 0.05, 14, 0, 800, 1000, 8000, 8000, 4000);

	// PID_Init(&stretch_2006.motor_pid.outer, 4.f, 0.05f, 0.1f, 0.f, 100.f, 350.f, 800.f, 800.f, 200.f);
	// PID_Init(&stretch_2006.motor_pid.inner, 170, 0, 30, 0, 800, 1000, 8000, 8000, 4000);
	PID_Init(&stretch_2006.motor_pid.outer, 0.f, 0.0f, 0.f, 0.f, 100.f, 350.f, 800.f, 800.f, 200.f);
	PID_Init(&stretch_2006.motor_pid.inner, 0, 0, 0, 0, 800, 1000, 8000, 8000, 4000);
	return 1;
}

void DJIMotorControl(ST_DJI_MOTOR *motor) // 电机控制
{
	// motor->motor_td.aim = motor->outerTarget;
	// CalTD(&motor->motor_td);
	// PID_Calc(&motor->motor_pid.outer,motor->motor_td.x1,motor->angle);
	PID_Calc(&motor->motor_pid.outer, motor->outerTarget, motor->angle);
	motor->innerTarget = motor->motor_pid.outer.fpU;
	PID_Calc(&motor->motor_pid.inner, motor->innerTarget, motor->anglev);
	motor->motor_current = motor->motor_pid.inner.fpU;
}
/*****3508电机数据处理**********/
float GetEncoderNumber(ST_DJI_MOTOR *motor, uint8_t msg[8])
{
	motor->EncoderNum = (msg[0] << 8) | (msg[1]);
	return motor->EncoderNum;
}

float GetSpeed(FDCAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8])
{
	int32_t speed_temp;
	int32_t base_value = 0xFFFF;
	if (msg[2] & 0x01 << 7)
	{
		speed_temp = (base_value << 16 | msg[2] << 8 | msg[3]);
	}
	else
	{
		speed_temp = (msg[2] << 8) | (msg[3]);
	}
	return speed_temp;
}

float GetCurrent(FDCAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8])
{
	int32_t speed_temp;
	int32_t base_value = 0xFFFF;
	if (msg[4] & 0x01 << 7)
	{
		speed_temp = (base_value << 16 | msg[4] << 8 | msg[5]);
	}
	else
	{
		speed_temp = (msg[4] << 8) | (msg[5]);
	}
	return speed_temp;
}

void Abs_Encoder_Process(ST_ENCODER *pEncoder, uint32_t value)
{
	pEncoder->siPreRawValue = pEncoder->siRawValue;
	pEncoder->siRawValue = value;
	pEncoder->siDiff = pEncoder->siRawValue - pEncoder->siPreRawValue;
	if (pEncoder->siDiff > (pEncoder->siNumber) / 2)
	{
		pEncoder->siDiff -= pEncoder->siNumber;
	}
	else if (pEncoder->siDiff < -(pEncoder->siNumber) / 2)
	{
		pEncoder->siDiff += pEncoder->siNumber;
	}
	pEncoder->siSumValue += pEncoder->siDiff;
}
