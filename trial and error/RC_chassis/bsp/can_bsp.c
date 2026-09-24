#include "can_bsp.h"

int flag1=0, flag2=0;

void can1_start(void)
{
    // 滤波器初始化
    CAN1_FILTER_CONFIG(&hcan1);

    // 启动CAN1
    HAL_CAN_Start(&hcan1);

    // 使能中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// @brief 配置过滤器
// @param CAN1 or CAN2
// @retval None
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 0;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x7FFE;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void can2_start(void)
{
    // 滤波器初始化
    CAN2_FILTER_CONFIG(&hcan2);

    // 启动CAN1
    HAL_CAN_Start(&hcan2);

    // 使能中断
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN2_FILTER_CONFIG(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 14;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x7FFE;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 28;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief 发送标准ID的数据帧
 * @param hcan CAN的句柄
 * @param ID 数据帧ID
 * @param pData 数组指针
 * @param Len 字节数0~8，请选择0x08
 */
void CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
    static CAN_TxHeaderTypeDef Tx_Header;

    Tx_Header.StdId = ID;
    //   Tx_Header.ExtId = CAN_ID_STD;
    Tx_Header.IDE = CAN_ID_STD;
    Tx_Header.RTR = CAN_RTR_DATA;
    Tx_Header.DLC = Len;

    /* 找到空的发送邮箱，把数据发送出去 */
    if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX2);
        }
    }
}

void CAN_SendCurrent_V6(CAN_HandleTypeDef *hcan, uint32_t id, uint16_t current, uint32_t motorID)
{
    uint8_t TxCurrent[8];
    if(current > 25000) current = 25000;
    if(current < -25000) current = -25000;
    current = current * 0.3f;
    TxCurrent[2*motorID-2] = (current >> 8) & 0xFF;
    TxCurrent[2*motorID-1] = current & 0xFF;
    CANx_SendStdData(hcan, id, TxCurrent, 8);
}

float GetSpeed_V6(CAN_RxHeaderTypeDef *pCanRxMsg, uint8_t msg[8])
{
    float speed_temp;
    memcpy(&speed_temp, &msg[4], 4);
    return speed_temp;
}

float GetAngle_V6(CAN_RxHeaderTypeDef *pCanRxMsg, uint8_t msg[8])
{
    float angle_temp;
    memcpy(&angle_temp, &msg[0], 4);
    return angle_temp;
}

void CAN_SendCurrent(CAN_HandleTypeDef *hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    uint8_t TxCurrent[8];
    TxCurrent[0] = (current1 >> 8);
    TxCurrent[1] = current1;
    TxCurrent[2] = (current2 >> 8);
    TxCurrent[3] = current2;
    TxCurrent[4] = (current3 >> 8);
    TxCurrent[5] = current3;
    TxCurrent[6] = (current4 >> 8);
    TxCurrent[7] = current4;
    CANx_SendStdData(hcan, id, TxCurrent, 8);
}

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

uint8_t RxMsg_CAN1[8];
uint8_t RxMsg_CAN2[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        CAN_RxHeaderTypeDef RxHeader;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN1);
        switch (RxHeader.StdId){
        case 0x201:// 0x200+id M6C18 全向轮/舵轮速度轮
            leftup_motor.angle = GetAngle_V6(&RxHeader, RxMsg_CAN1);
            wheel_encoder_velt_filter.leftup_velt.in = GetSpeed_V6(&RxHeader, RxMsg_CAN1);
            LpFilter(&wheel_encoder_velt_filter.leftup_velt);
            leftup_motor.anglev = wheel_encoder_velt_filter.leftup_velt.out;

            chassis_run.leftup.fpFB = leftup_motor.anglev;
            break;
        case 0x202:
            rightup_motor.angle = GetAngle_V6(&RxHeader, RxMsg_CAN1);
            wheel_encoder_velt_filter.rightup_velt.in = GetSpeed_V6(&RxHeader, RxMsg_CAN1);
            LpFilter(&wheel_encoder_velt_filter.rightup_velt);
            rightup_motor.anglev = wheel_encoder_velt_filter.rightup_velt.out;

            chassis_run.rightup.fpFB = rightup_motor.anglev;
            break;
        case 0x203:
            rightdown_motor.angle = GetAngle_V6(&RxHeader, RxMsg_CAN1);
            wheel_encoder_velt_filter.rightdown_velt.in = GetSpeed_V6(&RxHeader, RxMsg_CAN1);
            LpFilter(&wheel_encoder_velt_filter.rightdown_velt);
            rightdown_motor.anglev = wheel_encoder_velt_filter.rightdown_velt.out;

            chassis_run.rightdown.fpFB = rightdown_motor.anglev;
            break;
        case 0x204:
            leftdown_motor.angle = GetAngle_V6(&RxHeader, RxMsg_CAN1);
            wheel_encoder_velt_filter.leftdown_velt.in = GetSpeed_V6(&RxHeader, RxMsg_CAN1);
            LpFilter(&wheel_encoder_velt_filter.leftdown_velt);
            leftdown_motor.anglev = wheel_encoder_velt_filter.leftdown_velt.out;

            chassis_run.leftdown.fpFB = leftdown_motor.anglev;
            break;

        case 0x205:// 0x204+id M6020 舵轮角度轮
            flag2++;
            memcpy(rightdown_motor_angle.Motor_RxMsg, RxMsg_CAN1, sizeof(RxMsg_CAN1));
            rightdown_motor_angle.EncoderNum = GetEncoderNumber_DJI(&rightdown_motor_angle, rightdown_motor_angle.Motor_RxMsg);
            rightdown_motor_angle.encoder_speed = GetSpeed_DJI(&RxHeader, rightdown_motor_angle.Motor_RxMsg);
            Abs_Encoder_Process_DJI(&rightdown_motor_angle.motor_encoder, rightdown_motor_angle.EncoderNum);
            rightdown_motor_angle.angle = rightdown_motor_angle.motor_encoder.siSumValue / (float)8192 * 360.f / (float)rightdown_motor_angle.uiGearRatio;
            rightdown_motor_angle.anglev = rightdown_motor_angle.encoder_speed / (float)rightdown_motor_angle.uiGearRatio;

            chassis_steer_angle.rightdown_out.fpFB = rightdown_motor_angle.angle;
            // chassis_steer_angle.rightdown_out.fpFB = fmod(chassis_steer_angle.rightdown_out.fpFB, 360.f);
            // if (chassis_steer_angle.rightdown_out.fpFB > 180.f) chassis_steer_angle.rightdown_out.fpFB -= 360.f;
            break;
        // case 0x206:
        //     memcpy(rightup_motor_angle.Motor_RxMsg, RxMsg_CAN2, sizeof(RxMsg_CAN2));
        //     rightup_motor_angle.EncoderNum = GetEncoderNumber_DJI(&rightup_motor_angle, rightup_motor_angle.Motor_RxMsg);
        //     rightup_motor_angle.encoder_speed = GetSpeed_DJI(&RxHeader, rightup_motor_angle.Motor_RxMsg);
        //     Abs_Encoder_Process_DJI(&rightup_motor_angle.motor_encoder, rightup_motor_angle.EncoderNum);
        //     rightup_motor_angle.angle = rightup_motor_angle.motor_encoder.siSumValue / (float)8192 * 360.f / (float)rightup_motor_angle.uiGearRatio;
        //     rightup_motor_angle.anglev = rightup_motor_angle.encoder_speed / (float)rightup_motor_angle.uiGearRatio;
        //     break;
        // case 0x207:
        //     memcpy(leftdown_motor_angle.Motor_RxMsg, RxMsg_CAN2, sizeof(RxMsg_CAN2));
        //     rightdown_motor_angle.EncoderNum = GetEncoderNumber_DJI(&rightdown_motor_angle, rightdown_motor_angle.Motor_RxMsg);
        //     rightdown_motor_angle.encoder_speed = GetSpeed_DJI(&RxHeader, rightdown_motor_angle.Motor_RxMsg);
        //     Abs_Encoder_Process_DJI(&rightdown_motor_angle.motor_encoder, rightdown_motor_angle.EncoderNum);
        //     rightdown_motor_angle.angle = rightdown_motor_angle.motor_encoder.siSumValue / (float)8192 * 360.f / (float)rightdown_motor_angle.uiGearRatio;
        //     rightdown_motor_angle.anglev = rightdown_motor_angle.encoder_speed / (float)rightdown_motor_angle.uiGearRatio;
        //     break;
        // case 0x208:
        //     memcpy(leftdown_motor_angle.Motor_RxMsg, RxMsg_CAN2, sizeof(RxMsg_CAN2));
        //     leftdown_motor_angle.EncoderNum = GetEncoderNumber_DJI(&leftdown_motor_angle, leftdown_motor_angle.Motor_RxMsg);
        //     leftdown_motor_angle.encoder_speed = GetSpeed_DJI(&RxHeader, leftdown_motor_angle.Motor_RxMsg);
        //     Abs_Encoder_Process_DJI(&leftdown_motor_angle.motor_encoder, leftdown_motor_angle.EncoderNum);
        //     leftdown_motor_angle.angle = leftdown_motor_angle.motor_encoder.siSumValue / (float)8192 * 360.f / (float)leftdown_motor_angle.uiGearRatio;
        //     leftdown_motor_angle.anglev = leftdown_motor_angle.encoder_speed / (float)leftdown_motor_angle.uiGearRatio;
        //     break;
        // case 0x356:
        //     memcpy(&degreeA, &RxMsg_CAN2[0], 4);
        //     memcpy(&degreeB, &RxMsg_CAN2[4], 4);
        //     break;
        case 0x20A:
            flag1++;
            memcpy(leftdown_motor_angle.Motor_RxMsg, RxMsg_CAN1, sizeof(RxMsg_CAN1));
            leftdown_motor_angle.EncoderNum = GetEncoderNumber_DJI(&leftdown_motor_angle, leftdown_motor_angle.Motor_RxMsg);
            leftdown_motor_angle.encoder_speed = GetSpeed_DJI(&RxHeader, leftdown_motor_angle.Motor_RxMsg);
            Abs_Encoder_Process_DJI(&leftdown_motor_angle.motor_encoder, leftdown_motor_angle.EncoderNum);
            leftdown_motor_angle.angle = leftdown_motor_angle.motor_encoder.siSumValue / (float)8192 * 360.f / (float)leftdown_motor_angle.uiGearRatio;
            leftdown_motor_angle.anglev = leftdown_motor_angle.encoder_speed / (float)leftdown_motor_angle.uiGearRatio;

            chassis_steer_angle.leftdown_out.fpFB = leftdown_motor_angle.angle;
            // chassis_steer_angle.leftdown_out.fpFB = fmod(chassis_steer_angle.leftdown_out.fpFB, 360.f);
            // if (chassis_steer_angle.leftdown_out.fpFB > 180.f) chassis_steer_angle.leftdown_out.fpFB -= 360.f;
            break;
        default:
            break;
        }
    }
    if (hcan->Instance == CAN2)
    {
        CAN_RxHeaderTypeDef RxHeader;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN2);
        switch (RxHeader.StdId)
        {
        case 0x210:
            memcpy(dt35_distance, RxMsg_CAN2, 16);
            dt35_y1 = dt35_distance[0]; 
            dt35_x2 = dt35_distance[1]; 
            dt35_x1 = dt35_distance[2]; 
            dt35_y2 = dt35_distance[3]; 
            dt35_now.dt35_voltage_x1 = dt35_x1;
            dt35_now.dt35_voltage_x2 = dt35_x2;
            dt35_now.dt35_voltage_y1 = dt35_y1;
            dt35_now.dt35_voltage_y2 = dt35_y2;
            break;
        case 0x356:
            memcpy(&degreeA, &RxMsg_CAN2[0], 4);
            memcpy(&degreeB, &RxMsg_CAN2[4], 4);
            break;
        default:
            break;
        }
    }
}
