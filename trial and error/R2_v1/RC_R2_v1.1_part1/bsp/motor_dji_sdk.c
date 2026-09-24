#include "motor_dji_sdk.h"
// pid with td
// cons: 一个can-tx-mail只能发送一个大疆电机的数据，浪费can资源

float GetEncoderNumber_DJI(ST_MOTOR *motor, uint8_t msg[8])
{
    motor->EncoderNum = (msg[0] << 8) | (msg[1]);
    return motor->EncoderNum;
}

float GetSpeed_DJI(CAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8])
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

float GetCurrent_DJI(CAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8])
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
        pEncoder->state = 1; // 标记为已初始化
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

void DJI_ControlLoop(ST_MOTOR *motor)
{
    motor->innerFeedback = motor->anglev;
    motor->outerFeedback = motor->angle;

    switch (motor->ControlLoop_State)
    {
    case SPEED_LOOP:
        PID_Calc(&motor->pid_inner, motor->Input_v, motor->innerFeedback);
        motor->motor_current = motor->pid_inner.fpU;
        break;
    case MULTIPLE_LOOP:
        PID_Calc(&motor->pid_outer, motor->Input, motor->outerFeedback);
        PID_Calc(&motor->pid_inner, motor->pid_outer.fpU, motor->innerFeedback);
        motor->motor_current = motor->pid_inner.fpU;
        break;
    case MULTIPLE_LOOP_TD:
        motor->td.aim = motor->Input;
        CalTD(&(motor->td)); // smoothen input curve
        PID_Calc(&motor->pid_outer, motor->td.x1, motor->outerFeedback);
        PID_Calc(&motor->pid_inner, motor->pid_outer.fpU * 0.7f + motor->td.x2 * 0.3f, motor->innerFeedback); // smoothen inner_output(anglev) curve
        motor->motor_current = motor->pid_inner.fpU;
        break;
    default:
        break;
    }

    // switch (motor->motor_type)
    // {
    // case M6020: // 电压输入 限幅25000
    //     switch (motor->motor_id)
    //     {
    //     case 0x001:
    //         CAN_SendCurrent(&hcan1, 0X1FF, motor->motor_current, 0, 0, 0);
    //         break;
    //     case 0x002:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, motor->motor_current, 0, 0);
    //         break;
    //     case 0x003:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, 0, motor->motor_current, 0);
    //         break;
    //     case 0x004:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, 0, 0, motor->motor_current);
    //         break;
    //     case 0x005:
    //         CAN_SendCurrent(&hcan1, 0X2FF, motor->motor_current, 0, 0, 0);
    //         break;
    //     case 0x006:
    //         CAN_SendCurrent(&hcan1, 0X2FF, 0, motor->motor_current, 0, 0);
    //         break;
    //     case 0x007:
    //         CAN_SendCurrent(&hcan1, 0X2FF, 0, 0, motor->motor_current, 0);
    //         break;
    //     default:
    //         break;
    //     }
    //     break;

    // case M3508: // 电流输入 限幅16384
    //     switch (motor->motor_id)
    //     {
    //     case 0x001:
    //         CAN_SendCurrent(&hcan1, 0X200, motor->motor_current, 0, 0, 0);
    //         break;
    //     case 0x002:
    //         CAN_SendCurrent(&hcan2, 0X200, 0, motor->motor_current, 0, 0);
    //         break;
    //     case 0x003:
    //         CAN_SendCurrent(&hcan1, 0X200, 0, 0, motor->motor_current, 0);
    //         break;
    //     case 0x004:
    //         CAN_SendCurrent(&hcan1, 0X200, 0, 0, 0, motor->motor_current);
    //         break;
    //     case 0x005:
    //         CAN_SendCurrent(&hcan1, 0X1FF, motor->motor_current, 0, 0, 0);
    //         break;
    //     case 0x006:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, motor->motor_current, 0, 0);
    //         break;
    //     case 0x007:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, 0, motor->motor_current, 0);
    //         break;
    //     case 0x008:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, 0, 0, motor->motor_current);
    //         break;
    //     default:
    //         break;
    //     }
    //     break;

    // case M2006: // 电流输入 限幅10000
    //     switch (motor->motor_id)
    //     {
    //     case 0x001:
    //         CAN_SendCurrent(&hcan1, 0X200, motor->motor_current, 0, 0, 0);
    //         break;
    //     case 0x002:
    //         CAN_SendCurrent(&hcan2, 0X200, 0, motor->motor_current, 0, 0);
    //         break;
    //     case 0x003:
    //         CAN_SendCurrent(&hcan2, 0X200, 0, 0, motor->motor_current, 0);
    //         break;
    //     case 0x004:
    //         CAN_SendCurrent(&hcan1, 0X200, 0, 0, 0, motor->motor_current);
    //         break;
    //     case 0x005:
    //         CAN_SendCurrent(&hcan1, 0X1FF, motor->motor_current, 0, 0, 0);
    //         break;
    //     case 0x006:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, motor->motor_current, 0, 0);
    //         break;
    //     case 0x007:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, 0, motor->motor_current, 0);
    //         break;
    //     case 0x008:
    //         CAN_SendCurrent(&hcan1, 0X1FF, 0, 0, 0, motor->motor_current);
    //         break;
    //     default:
    //         break;
    //     }
    //     break;
        
    // default:
    //     break;
    // }
}
