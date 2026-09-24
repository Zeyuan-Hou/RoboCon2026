#include "can_bsp.h"

// 包括can初始化、J60can发送、达妙can发送、can接收中断回调

void can1_start(void)
{
    CAN1_FILTER_CONFIG(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

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
    CAN2_FILTER_CONFIG(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    //HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
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

void CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
    static CAN_TxHeaderTypeDef Tx_Header;

    Tx_Header.StdId = ID;
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

void TravelSwitchDataDeal(const uint8_t travel_switch, uint8_t *key_states)
{
    for (uint8_t i = 0; i < 8; i++){
        key_states[i] = (travel_switch >> i) & 0x01;
    }
}

void CAN_SendData_J60(CAN_HandleTypeDef *hcan, J60_MotorCMD *data)
{
    static CAN_TxHeaderTypeDef Tx_Header;
    Tx_Header.StdId = J60_FormCanId(data->cmd_, data->motor_id_);
    Tx_Header.ExtId = 0;
    Tx_Header.IDE = CAN_ID_STD;
    Tx_Header.RTR = CAN_RTR_DATA;

    switch (data->cmd_)
    {
    case J60_DISABLE_CMD:
        Tx_Header.DLC = J60_SEND_DLC_DISABLE;
        if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, NULL, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        {
            if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, NULL, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
            {
                HAL_CAN_AddTxMessage(hcan, &Tx_Header, NULL, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
        break;
    case J60_ENABLE_CMD:
        Tx_Header.DLC = J60_SEND_DLC_ENABLE;
        if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, NULL, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        {
            if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, NULL, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
            {
                HAL_CAN_AddTxMessage(hcan, &Tx_Header, NULL, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
        break;
    case J60_CONTROL_CMD:
        Tx_Header.DLC = J60_SEND_DLC_CONTROL;
        uint16_t _position = float_to_uint(data->position_, J60_POSITION_MIN, J60_POSITION_MAX, J60_SEND_POSITION_LENGTH);
        uint16_t _velocity = float_to_uint(data->velocity_, J60_VELOCITY_MIN, J60_VELOCITY_MAX, J60_SEND_VELOCITY_LENGTH);
        uint16_t _torque = float_to_uint(data->torque_, J60_TORQUE_MIN, J60_TORQUE_MAX, J60_SEND_TORQUE_LENGTH);
        uint16_t _kp = float_to_uint(data->kp_, J60_KP_MIN, J60_KP_MAX, J60_SEND_KP_LENGTH);
        uint16_t _kd = float_to_uint(data->kd_, J60_KD_MIN, J60_KD_MAX, J60_SEND_KD_LENGTH);
        j60_Tx[0] = _position;
        j60_Tx[1] = _position >> 8;
        j60_Tx[2] = _velocity;
        j60_Tx[3] = ((_velocity >> 8) & 0x3f) | ((_kp & 0x03) << 6);
        j60_Tx[4] = _kp >> 2;
        j60_Tx[5] = _kd;
        j60_Tx[6] = _torque;
        j60_Tx[7] = _torque >> 8;
        if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, j60_Tx, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        {
            if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, j60_Tx, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
            {
                HAL_CAN_AddTxMessage(hcan, &Tx_Header, j60_Tx, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
        break;
    default:
        break;
    }
}

uint8_t RxMsg_CAN1[8];
uint8_t RxMsg_CAN2[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        CAN_RxHeaderTypeDef RxHeader;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN1);
        uint32_t StdId = RxHeader.StdId;
        switch (StdId)
        {
					case 0x205:
						memcpy(backfoot_motor.Motor_RxMsg, RxMsg_CAN1, sizeof(RxMsg_CAN1));
            backfoot_motor.EncoderNum = GetEncoderNumber_DJI(&backfoot_motor, backfoot_motor.Motor_RxMsg);
            backfoot_motor.encoder_speed = GetSpeed_DJI(&RxHeader, backfoot_motor.Motor_RxMsg);
            Abs_Encoder_Process_DJI(&backfoot_motor.motor_encoder, backfoot_motor.EncoderNum);
            backfoot_motor.angle = backfoot_motor.motor_encoder.siSumValue / (float)8192 * 360.f / (float)backfoot_motor.uiGearRatio;
            backfoot_motor.anglev = backfoot_motor.encoder_speed / (float)backfoot_motor.uiGearRatio;
            
						break;
        case 0x202: // 0x200+id M6C18 全向轮/舵轮速度轮
            monitor.rate_cnt.can1_ld++;
            memcpy(&leftdown_motor.angle, &RxMsg_CAN1[0], 4);
            memcpy(&leftdown_motor.anglev, &RxMsg_CAN1[4], 4);
            //LpFilter(&wheel_encoder_velt_filter.leftdown_velt);
            //leftdown_motor.anglev = wheel_encoder_velt_filter.leftdown_velt.out;
            chassis_run.leftdown.fpFB = leftdown_motor.anglev;
            break;

        case 0x204:
            monitor.rate_cnt.can1_rd++;
            memcpy(&rightdown_motor.angle, &RxMsg_CAN1[0], 4);
            memcpy(&rightdown_motor.anglev, &RxMsg_CAN1[4], 4);
            //LpFilter(&wheel_encoder_velt_filter.rightdown_velt);
            //rightdown_motor.anglev = wheel_encoder_velt_filter.rightdown_velt.out;
            chassis_run.rightdown.fpFB = rightdown_motor.anglev;
            break;

        case 0x201:
            monitor.rate_cnt.can1_ld_turn++;
            memcpy(leftdown_turn_motor.Motor_RxMsg, RxMsg_CAN1, sizeof(RxMsg_CAN1));
            leftdown_turn_motor.EncoderNum = GetEncoderNumber_DJI(&leftdown_turn_motor, leftdown_turn_motor.Motor_RxMsg);
            leftdown_turn_motor.encoder_speed = GetSpeed_DJI(&RxHeader, leftdown_turn_motor.Motor_RxMsg);
            Abs_Encoder_Process_DJI(&leftdown_turn_motor.motor_encoder, leftdown_turn_motor.EncoderNum);
            leftdown_turn_motor.angle = leftdown_turn_motor.motor_encoder.siSumValue / (float)8192 * 360.f / (float)leftdown_turn_motor.uiGearRatio;
            leftdown_turn_motor.anglev = leftdown_turn_motor.encoder_speed / (float)leftdown_turn_motor.uiGearRatio;
            break;

        case 0x203:
            monitor.rate_cnt.can1_rd_turn++;
            memcpy(rightdown_turn_motor.Motor_RxMsg, RxMsg_CAN1, sizeof(RxMsg_CAN1));
            rightdown_turn_motor.EncoderNum = GetEncoderNumber_DJI(&rightdown_turn_motor, rightdown_turn_motor.Motor_RxMsg);
            rightdown_turn_motor.encoder_speed = GetSpeed_DJI(&RxHeader, rightdown_turn_motor.Motor_RxMsg);
            Abs_Encoder_Process_DJI(&rightdown_turn_motor.motor_encoder, rightdown_turn_motor.EncoderNum);
            rightdown_turn_motor.angle = rightdown_turn_motor.motor_encoder.siSumValue / (float)8192 * 360.f / (float)rightdown_turn_motor.uiGearRatio;
            rightdown_turn_motor.anglev = rightdown_turn_motor.encoder_speed / (float)rightdown_turn_motor.uiGearRatio;
            break;

        case 0x220:
            monitor.rate_cnt.can1_dt35++;
            memcpy(&travel_switch, RxMsg_CAN1, sizeof(uint8_t));
            TravelSwitchDataDeal(travel_switch, travel_switch_mode);
            break;

        case 0x210:
            monitor.rate_cnt.can1_travelSwitch++;
            memcpy(dt35_distance, RxMsg_CAN1, 16);
            // 小脚加速过程需要快速响应，故不做滤波处理
            //x轴坐标
            dt35_now.dt35_voltage_1 = K_DT35_X1 * dt35_distance[1]+ B_DT35_X1;
            
            //y轴坐标
            dt35_now.dt35_voltage_2 = K_DT35_Y1 * dt35_distance[0] + B_DT35_Y1;
            
						//for lil_feet
            dt35_now.dt35_voltage_3 = 1 * dt35_distance[2] + 0 * dt35_now.dt35_pre_voltage_3;
            dt35_now.dt35_pre_voltage_3 = dt35_now.dt35_voltage_3;
				
            dt35_now.dt35_voltage_4 = 1 * dt35_distance[3] + 0 * dt35_now.dt35_pre_voltage_4;
            dt35_now.dt35_pre_voltage_4 = dt35_now.dt35_voltage_4;
            break;

        default:
            break;
        }
    }

    if (hcan->Instance == CAN2)
    {
        CAN_RxHeaderTypeDef RxHeader;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN2);
        uint32_t StdId = RxHeader.StdId;
        switch (StdId)
        {
        case 0x202:    
            monitor.rate_cnt.can2_lu++;    
            memcpy(&leftup_motor.angle, &RxMsg_CAN2[0], 4);
            //memcpy(&wheel_encoder_velt_filter.leftup_velt.in, &RxMsg_CAN2[4], 4);
            //LpFilter(&wheel_encoder_velt_filter.leftup_velt);
            //leftup_motor.anglev = wheel_encoder_velt_filter.leftup_velt.out;
				    memcpy(&leftup_motor.anglev, &RxMsg_CAN2[4], 4);
            chassis_run.leftup.fpFB = leftup_motor.anglev;
            break;

        case 0x204:
            monitor.rate_cnt.can2_ru++;
            memcpy(&rightup_motor.angle, &RxMsg_CAN2[0], 4);
            memcpy(&rightup_motor.anglev, &RxMsg_CAN2[4], 4);
            //LpFilter(&wheel_encoder_velt_filter.rightup_velt);
            //rightup_motor.anglev = wheel_encoder_velt_filter.rightup_velt.out;
            chassis_run.rightup.fpFB = rightup_motor.anglev;
            break;

        case 0x201:
            monitor.rate_cnt.can2_lu_turn++;
            memcpy(leftup_turn_motor.Motor_RxMsg, RxMsg_CAN2, sizeof(RxMsg_CAN2));
            leftup_turn_motor.EncoderNum = GetEncoderNumber_DJI(&leftup_turn_motor, leftup_turn_motor.Motor_RxMsg);
            leftup_turn_motor.encoder_speed = GetSpeed_DJI(&RxHeader, leftup_turn_motor.Motor_RxMsg);
            Abs_Encoder_Process_DJI(&leftup_turn_motor.motor_encoder, leftup_turn_motor.EncoderNum);
            leftup_turn_motor.angle = leftup_turn_motor.motor_encoder.siSumValue / (float)8192 * 360.f / (float)leftup_turn_motor.uiGearRatio;
            leftup_turn_motor.anglev = leftup_turn_motor.encoder_speed / (float)leftup_turn_motor.uiGearRatio;
            break;

        case 0x203:
            monitor.rate_cnt.can2_ru_turn++;
            memcpy(rightup_turn_motor.Motor_RxMsg, RxMsg_CAN2, sizeof(RxMsg_CAN2));
            rightup_turn_motor.EncoderNum = GetEncoderNumber_DJI(&rightup_turn_motor, rightup_turn_motor.Motor_RxMsg);
            rightup_turn_motor.encoder_speed = GetSpeed_DJI(&RxHeader, rightup_turn_motor.Motor_RxMsg);
            Abs_Encoder_Process_DJI(&rightup_turn_motor.motor_encoder, rightup_turn_motor.EncoderNum);
            rightup_turn_motor.angle = rightup_turn_motor.motor_encoder.siSumValue / (float)8192 * 360.f / (float)rightup_turn_motor.uiGearRatio;
            rightup_turn_motor.anglev = rightup_turn_motor.encoder_speed / (float)rightup_turn_motor.uiGearRatio;
            break;

        case 0x91:
            monitor.rate_cnt.can2_j60++;
            J60_Unpack_Feedback(RxMsg_CAN2, &j60_motor_data_down, StdId);
            break;

        default:
            break;
        }
    }
}
