#include "dji_motor.h"

float GetEncoderNumber_DJI(DJI_MOTOR *motor, uint8_t msg[8])
{
    motor->EncoderNum = (msg[0] << 8) | (msg[1]);
    return motor->EncoderNum;
}

float GetSpeed_DJI(uint8_t msg[8])
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

float GetCurrent_DJI(uint8_t msg[8])
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

void Abs_Encoder_Process_DJI(ST_ENCODER *pEncoder, uint32_t value)
{
    if (pEncoder->state == 0)
    {
        pEncoder->siPreRawValue = value;
        pEncoder->siRawValue = value;
        pEncoder->siDiff = 0;
        pEncoder->siSumValue = 0;
        pEncoder->state = 1; 
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

void DJI_ReceiveData(DJI_MOTOR* motor, uint8_t *data )
{

    motor->EncoderNum = GetEncoderNumber_DJI(motor, data);
  motor->encoder_speed = GetSpeed_DJI(data);
    motor->current=GetCurrent_DJI(data);
  Abs_Encoder_Process_DJI(&motor->motor_encoder, motor->EncoderNum);
  motor->angle = (float)motor->motor_encoder.siSumValue / 8192.f * 360.f / motor->uiGearRatio;
  motor->speed = motor->encoder_speed / motor->uiGearRatio*RPM_TO_RADS;
        
}




