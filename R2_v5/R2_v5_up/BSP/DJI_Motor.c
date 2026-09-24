#include "DJI_Motor.h"

void DJIMotorControl(ST_DJI_MOTOR *motor) // µç»ú¿ØÖÆ
{
	PID_Calc(&motor->motor_pid.outer, motor->outerTarget, motor->angle);
	motor->innerTarget = motor->motor_pid.outer.fpU;
	PID_Calc(&motor->motor_pid.inner, motor->innerTarget, motor->anglev);
	motor->motor_current = motor->motor_pid.inner.fpU;
}

float GetEncoderNumber(ST_DJI_MOTOR *motor, uint8_t msg[8])
{
	motor->EncoderNum = (msg[0] << 8) | (msg[1]);
	return motor->EncoderNum;
}

float GetSpeed(uint8_t msg[8])
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

float GetCurrent(uint8_t msg[8])
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
	if (pEncoder->state == 0)
	{
		pEncoder->siPreRawValue = value;
		pEncoder->siRawValue = value;
		pEncoder->siDiff = 0;
		pEncoder->siSumValue = 0;
		pEncoder->state = 1; // ???????
		return;
	}
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

void DJI_RxData(ST_DJI_MOTOR* motor, uint8_t msg[8])
{
	motor->EncoderNum = GetEncoderNumber(motor, msg);
	motor->encoder_speed = GetSpeed(msg);
	Abs_Encoder_Process(&motor->motor_encoder, motor->EncoderNum);
	motor->angle = motor->motor_encoder.siSumValue / (float)8192 * 360.f / (float)motor->motor_encoder.siGearRatio - motor->Start_Pos;
	if (!motor->getStartPos)
	{
		motor->Start_Pos = motor->angle;
		motor->getStartPos = 1;
	}
	motor->anglev = motor->encoder_speed / (float)motor->motor_encoder.siGearRatio;
	motor->tor_cur = GetCurrent(msg);
	motor->err = msg[7];
}
